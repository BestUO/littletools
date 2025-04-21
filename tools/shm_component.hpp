#pragma once
#include <pthread.h>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>
#include <mutex>
#include <functional>
#include <atomic>
#include <type_traits>
#include <limits>
#include <map>

#define READER_SIZE 64
#define CIRCLE_INDEX_TYPE uint8_t
#define SHM_QUEUE_SIZE CIRCLE_INDEX_TYPE(-1) + 1
#define SHM_ALIGN(s, a) (((s - 1) | (a - 1)) + 1)
inline uint8_t* align_address(void* ptr, size_t alignment)
{
    auto int_ptr         = reinterpret_cast<uintptr_t>(ptr);
    auto aligned_int_ptr = SHM_ALIGN(int_ptr, alignment);
    return reinterpret_cast<uint8_t*>(aligned_int_ptr);
}

template <typename T>
class SHMFactory
{
public:
    SHMFactory(std::string&& name)
        : __name(std::move(name))
    {
        bool is_create = true;
        __fd = shm_open(__name.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
        if (__fd >= 0)
        {
            if (ftruncate(__fd, sizeof(T)) == -1)
                perror("ftruncate failed");
        }
        else if (errno == EEXIST)
        {
            __fd      = shm_open(__name.c_str(), O_RDWR, 0666);
            is_create = false;
        }

        auto ptr = (T*)mmap(
            NULL, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, __fd, 0);
        if (is_create)
        {
            __ptr = new (ptr) T();
        }
        else
        {
            __ptr = reinterpret_cast<T*>(ptr);
        }
    }
    ~SHMFactory()
    {
        close(__fd);
        munmap(&__ptr, sizeof(T));
        shm_unlink(__name.c_str());
    }
    T* operator->() const
    {
        return __ptr;
    }

    T& operator*() const
    {
        return *__ptr;
    }

private:
    int __fd;
    T* __ptr;
    std::string __name = "";
};

class MutexLock
{
public:
    MutexLock()
    {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_NORMAL);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
#ifdef __linux__
        pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
#endif
        pthread_mutex_init(&__mutex, &attr);
    }
    ~MutexLock()
    {
        pthread_mutex_destroy(&__mutex);
    }
    void lock()
    {
        auto r = pthread_mutex_lock(&__mutex);
        if (r == EOWNERDEAD)
            r = pthread_mutex_consistent(&__mutex);
    }
    bool try_lock()
    {
        return pthread_mutex_trylock(&__mutex);
    }
    void unlock()
    {
        pthread_mutex_unlock(&__mutex);
    }
    pthread_mutex_t* GetMutexPtr()
    {
        return &__mutex;
    }

private:
    pthread_mutex_t __mutex;
};

class CondVar
{
public:
    CondVar()
    {
        pthread_condattr_t attr;
        pthread_condattr_init(&attr);
        pthread_condattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_cond_init(&__cond, &attr);
    }
    ~CondVar()
    {
        pthread_cond_destroy(&__cond);
    }
    void wait(MutexLock& lock)
    {
        pthread_cond_wait(&__cond, lock.GetMutexPtr());
    }
    void wait_for(MutexLock& lock, const timespec& time)
    {
        pthread_cond_timedwait(&__cond, lock.GetMutexPtr(), &time);
    }
    void notify_one()
    {
        pthread_cond_signal(&__cond);
    }
    void notify_all()
    {
        pthread_cond_broadcast(&__cond);
    }

private:
    pthread_cond_t __cond;
};
template <typename T>
concept PodType = std::is_trivial_v<T> && std::is_standard_layout_v<T>;
// concept PodType = std::true_type::value;
namespace SHMV1
{

template <typename T>
concept DeQueueElement = requires(T t)
{
    {
        decltype(t.__prev)(), decltype(t.__next)()
        } -> std::same_as<int16_t>;
};

template <typename T>
concept QueueElement = requires(T t)
{
    {
        decltype(t.__next)()
        } -> std::same_as<int16_t>;
};

class SHMSharedObjBase;

class SHMObjBase
{
public:
    SHMObjBase()                             = default;
    SHMObjBase(const SHMObjBase&)            = delete;
    SHMObjBase& operator=(const SHMObjBase&) = delete;
    SHMObjBase(SHMObjBase&&)                 = delete;
    SHMObjBase& operator=(SHMObjBase&&)      = delete;
    virtual ~SHMObjBase()                    = default;

    uint8_t GetCount()
    {
        return __count;
    }

    int16_t GetIndex()
    {
        return __index;
    }

private:
    int16_t __index              = -1;
    std::atomic<uint8_t> __count = {0};
    uint8_t CountInc()
    {
        return __count++;
    }

    uint8_t CountDec()
    {
        return __count--;
    }

    void SetIndex(int16_t index)
    {
        __index = index;
    }
    friend SHMSharedObjBase;
    friend void SHMObjBaseSetIndex(SHMObjBase& obj, int16_t index);
};
inline void SHMObjBaseSetIndex(SHMObjBase& obj, int16_t index)
{
    obj.SetIndex(index);
}

template <typename T, size_t element_size = sizeof(T)>
class SHMObj : public SHMObjBase
{
public:
    T& GetData()
    {
        return __t;
    }

private:
    T __t;
};

template <size_t element_size>
class SHMObj<char, element_size> : public SHMObjBase
{
public:
    char* GetData()
    {
        return __t;
    }

private:
    char __t[element_size];
};

class SHMSharedObjBase
{
public:
    SHMSharedObjBase() = default;
    SHMSharedObjBase(SHMObjBase* obj_ptr, std::function<void()> del_fun)
        : __obj_ptr(obj_ptr)
        , __del_fun(del_fun)
    {
        if (__obj_ptr)
            __obj_ptr->CountInc();
    }

    SHMSharedObjBase(const SHMSharedObjBase& obj)
        : __obj_ptr(obj.__obj_ptr)
        , __del_fun(obj.__del_fun)
    {
        if (obj.__obj_ptr)
            __obj_ptr->CountInc();
    }

    SHMSharedObjBase(SHMSharedObjBase&& obj)
        : __obj_ptr(obj.__obj_ptr)
        , __del_fun(obj.__del_fun)
    {
        if (obj.__obj_ptr)
            obj.__obj_ptr = nullptr;
    }

    virtual ~SHMSharedObjBase()
    {
        if (__obj_ptr && __obj_ptr->CountDec() == 1 && __del_fun)
        {
            __del_fun();
        }
    }

    SHMSharedObjBase& operator=(const SHMSharedObjBase& obj)
    {
        __obj_ptr = obj.__obj_ptr;
        __del_fun = obj.__del_fun;
        if (__obj_ptr)
            __obj_ptr->CountInc();
        return *this;
    }

    uint8_t GetCount()
    {
        return __obj_ptr->GetCount();
    }

    int16_t GetIndex()
    {
        return __obj_ptr->GetIndex();
    }

protected:
    void CountInc()
    {
        if (__obj_ptr)
            __obj_ptr->CountInc();
    }

    void CountDec()
    {
        if (__obj_ptr)
            __obj_ptr->CountDec();
    }
    SHMObjBase* __obj_ptr;

private:
    std::function<void()> __del_fun;
};

template <PodType, bool SHAREDOBJT, size_t N>
class SHMMessageQueue;

template <typename T, size_t element_size = sizeof(T)>
class SHMSharedObj : public SHMSharedObjBase
{
public:
    using OBJTYPE  = SHMObj<T>;
    SHMSharedObj() = default;
    SHMSharedObj(OBJTYPE* obj_ptr, std::function<void()> del_fun)
        : SHMSharedObjBase(obj_ptr, std::move(del_fun))
    { }

    OBJTYPE* GetSHMObj()
    {
        return (OBJTYPE*)__obj_ptr;
    }

private:
    friend SHMMessageQueue<T, true, SHM_QUEUE_SIZE>;
};

template <size_t element_size>
class SHMSharedObj<char, element_size> : public SHMSharedObjBase
{
public:
    using OBJTYPE  = SHMObj<char, element_size>;
    SHMSharedObj() = default;
    SHMSharedObj(OBJTYPE* obj_ptr, std::function<void()> del_fun)
        : SHMSharedObjBase(obj_ptr, std::move(del_fun))
    { }

    OBJTYPE* GetSHMObj()
    {
        return (OBJTYPE*)__obj_ptr;
    }

private:
    friend SHMMessageQueue<char, true, SHM_QUEUE_SIZE>;
};

template <DeQueueElement T>
class SHMFIFOListObj
{
public:
    int16_t PopFront(T* data)
    {
        if (__head == -1)
            return -1;
        else
        {
            auto oldhead = __head;
            __head       = data[__head].__next;

            data[oldhead].__next = -1;
            data[oldhead].__prev = -1;

            if (__head == -1)
                __tail = -1;

            return oldhead;
        }
    }

    void PushBack(T* data, CIRCLE_INDEX_TYPE index)
    {
        if (__tail == -1)
        {
            __head = __tail = index;
        }
        else
        {
            data[index].__prev  = __tail;
            data[index].__next  = -1;
            data[__tail].__next = index;
            __tail              = index;
        }
    }

    bool IsEmpty()
    {
        return __head == -1;
    }

private:
    int32_t __head = -1;
    int32_t __tail = -1;
};

template <DeQueueElement T>
class SHMLIFOListObj
{
public:
    int16_t PopBack(T* data)
    {
        if (__tail == -1)
            return -1;
        else
        {
            auto oldtail = __tail;
            __tail       = data[__tail].__next;

            data[oldtail].__next = -1;
            data[oldtail].__prev = -1;

            return oldtail;
        }
    }

    void PushBack(T* data, CIRCLE_INDEX_TYPE index)
    {
        if (__tail == -1)
        {
            __tail = index;
        }
        else
        {
            data[index].__prev  = __tail;
            data[index].__next  = -1;
            data[__tail].__next = index;
            __tail              = index;
        }
    }

    bool IsEmpty()
    {
        return __tail == -1;
    }

private:
    int32_t __tail = -1;
};

template <QueueElement T>
class SHMCircle
{
public:
    SHMCircle(T* data, size_t N)
    {
        for (size_t i = 0; i < N - 1; i++)
            data[i].__next = i + 1;
        data[N - 1].__next = 0;
    }

    int32_t PushBack(T* data)
    {
        if (IsFull())
        {
            return -1;
        }
        else
        {
            auto oldtail = __tail;
            __tail       = data[__tail].__next;
            return oldtail;
        }
    }

    int32_t PopFront(T* data)
    {
        if (IsEmpty())
        {
            return -1;
        }
        else
        {
            auto oldhead = __head;
            __head       = data[__head].__next;
            return oldhead;
        }
    }

private:
    CIRCLE_INDEX_TYPE __head = 0;
    CIRCLE_INDEX_TYPE __tail = 0;

    bool IsFull()
    {
        return std::numeric_limits<CIRCLE_INDEX_TYPE>::max()
            == CIRCLE_INDEX_TYPE(__tail - __head);
    }

    bool IsEmpty()
    {
        return __head == __tail;
    }
};

template <typename T, size_t N = SHM_QUEUE_SIZE>
class SHMMemoryPoolBase
{
public:
    using SHAREDOBJYPE = T;
    SHMMemoryPoolBase()
    {
        for (size_t i = 0; i < N; i++)
        {
            SHMObjBaseSetIndex(__data[i].__obj, i);
            __FIFOList.PushBack(__data, i);
        }
    }

    SHAREDOBJYPE Allocate()
    {
        std::lock_guard<MutexLock> lck(__mutex);
        auto index = __FIFOList.PopFront(__data);
        if (index == -1)
            return SHAREDOBJYPE(nullptr, nullptr);
        else
            return SHAREDOBJYPE(&__data[index].__obj, [this, index = index]() {
                ReleaseObject(index);
            });
    }

    SHAREDOBJYPE GetSharedObject(uint16_t index)
    {
        std::lock_guard<MutexLock> lck(__mutex);
        if (__data[index].__next == -1 && __data[index].__prev == -1)
            return SHAREDOBJYPE(&__data[index].__obj, [this, index = index]() {
                ReleaseObject(index);
            });
        else
            return SHAREDOBJYPE(nullptr, nullptr);
    }

    typename SHAREDOBJYPE::OBJTYPE* GetObjectForce(uint16_t index)
    {
        std::lock_guard<MutexLock> lck(__mutex);
        return &__data[index].__obj;
    }

    void ReleaseObject(uint16_t index)
    {
        {
            std::lock_guard<MutexLock> lck(__mutex);
            __FIFOList.PushBack(__data, index);
        }
        __cond_var.notify_one();
    }

    void Wait()
    {
        std::lock_guard<MutexLock> lck(__mutex);
        __cond_var.wait(__mutex);
    }

private:
    struct SHMElement
    {
        int16_t __prev = -1;
        int16_t __next = -1;
        typename SHAREDOBJYPE::OBJTYPE __obj;
    };
    MutexLock __mutex;
    CondVar __cond_var;
    SHMFIFOListObj<SHMElement> __FIFOList;
    SHMElement __data[N];
};

template <PodType T, size_t element_size = sizeof(T), size_t N = SHM_QUEUE_SIZE>
class SHMMemoryPool : public SHMMemoryPoolBase<SHMSharedObj<T>, N>
{
public:
    using MPE = T;
};

template <size_t element_size, size_t N>
class SHMMemoryPool<char, element_size, N>
    : public SHMMemoryPoolBase<SHMSharedObj<char, element_size>, N>
{
public:
    using MPE = char;
};

template <PodType T, bool USESHAREDOBJ = false, size_t N = SHM_QUEUE_SIZE>
class SHMMessageQueue
{
public:
    using ELEMENTTYPE = std::conditional_t<USESHAREDOBJ, int16_t, T>;
    SHMMessageQueue()
    {
        for (size_t i = 0; i < N - 1; i++)
            __data[i].__next = i + 1;
        __data[N - 1].__next = 0;
    }

    bool PushBack(T&& t)
    {
        if (IsFull())
        {
            return false;
        }
        else
        {
            __data[__tail].__t = std::move(t);
            __tail             = __data[__tail].__next;
            return true;
        }
    }

    template <typename SHMSHAREDOBJTYPE>
    bool PushBackSharedObj(SHMSHAREDOBJTYPE& shared_obj)
    {
        if (IsFull())
        {
            return false;
        }
        else
        {
            shared_obj.CountInc();
            __data[__tail].__t = shared_obj.GetSHMObj()->GetIndex();
            __tail             = __data[__tail].__next;
            return true;
        }
    }

    std::tuple<bool, T> PopFront()
    {
        if (IsEmpty())
        {
            return {false, T()};
        }
        else
        {
            auto t = std::move(__data[__head].__t);
            __head = __data[__head].__next;
            return {true, t};
        }
    }

    template <typename MEMPOOL>
    std::tuple<bool, typename MEMPOOL::SHAREDOBJYPE> PopFront(MEMPOOL& pool)
    {
        if (IsEmpty())
        {
            return {false, typename MEMPOOL::SHAREDOBJYPE(nullptr, nullptr)};
        }
        else
        {
            auto obj_index  = __data[__head].__t;
            auto shared_obj = pool.GetSharedObject(obj_index);
            shared_obj.CountDec();
            __head = __data[__head].__next;
            return {true, shared_obj};
        }
    }

    MutexLock& GetMutex()
    {
        return __mutex;
    }

    CondVar& GetCondVar()
    {
        return __cond_var;
    }

private:
    struct SHMElement
    {
        int16_t __prev = -1;
        int16_t __next = -1;
        ELEMENTTYPE __t;
    };
    MutexLock __mutex;
    CondVar __cond_var;
    SHMElement __data[N];
    CIRCLE_INDEX_TYPE __head = 0;
    CIRCLE_INDEX_TYPE __tail = 0;

    bool IsFull()
    {
        return std::numeric_limits<CIRCLE_INDEX_TYPE>::max()
            == CIRCLE_INDEX_TYPE(__tail - __head);
    }

    bool IsEmpty()
    {
        return __head == __tail;
    }
};
}  // namespace SHMV1

namespace SHMV2
{
template <typename T>
concept DeQueElement = requires(T t)
{
    {
        decltype(t.__prev)(), decltype(t.__next)()
        } -> std::same_as<int32_t>;
};

template <typename T>
concept QueueElement = requires(T t)
{
    {
        decltype(t.__next)()
        } -> std::same_as<int32_t>;
};

template <PodType T>
class Element
{
public:
    T* GetData()
    {
        return (T*)__t;
    }

private:
    char __t[sizeof(T)] = {0};
};

template <typename T>
class IndexDataTypeCheck
{
public:
    int32_t __index = 0;
};

template <QueueElement T>
class SHMStack
{
public:
    int16_t Pop(T* data)
    {
        if (__head == -1)
            return -1;
        else
        {
            auto old_head         = __head;
            __head                = data[old_head].__next;
            data[old_head].__next = -1;

            return old_head;
        }
    }

    void Push(T* data, int32_t index)
    {
        if (__head == -1)
        {
            __head = index;
        }
        else
        {
            data[index].__next = __head;
            __head             = index;
        }
    }

    bool IsEmpty()
    {
        return __head == -1;
    }

private:
    int32_t __head = -1;
};

template <DeQueElement T>
class SHMDeque
{
public:
    int32_t Pop(T* data)
    {
        if (__head == -1)
            return -1;
        else
        {
            auto old_head = __head;
            __head        = data[old_head].__next;

            data[old_head].__next = -1;
            data[old_head].__prev = -1;

            if (__head == -1)
                __tail = -1;

            return old_head;
        }
    }

    void Push(T* data, int32_t index)
    {
        if (__tail == -1)
        {
            __head = __tail = index;
        }
        else
        {
            data[index].__prev  = __tail;
            data[index].__next  = -1;
            data[__tail].__next = index;
            __tail              = index;
        }
    }

    bool IsEmpty()
    {
        return __head == -1;
    }

private:
    int32_t __head = -1;
    int32_t __tail = -1;
};

template <PodType T, size_t N = SHM_QUEUE_SIZE>
class SHMMemoryPool
{
public:
    SHMMemoryPool()
    {
        for (size_t i = 0; i < N; i++)
            __shm_stack.Push(__data, i);
    }

    std::tuple<IndexDataTypeCheck<T>, T*> Allocate()
    {
        std::lock_guard<MutexLock> lck(__mutex);
        auto index = __shm_stack.Pop(__data);
        if (index == -1)
            return {{index}, nullptr};
        else
            return {{index}, __data[index].__t.GetData()};
    }

    void Free(IndexDataTypeCheck<T> index)
    {
        std::lock_guard<MutexLock> lck(__mutex);
        __shm_stack.Push(__data, index.__index);
    }

private:
    struct SHMMempoolElement
    {
        int32_t __next = -1;
        Element<T> __t;
    };
    MutexLock __mutex;
    SHMStack<SHMMempoolElement> __shm_stack;
    SHMMempoolElement __data[N];
};

template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
class SHMMsgQueue
{
public:
    SHMMsgQueue()
    {
        for (size_t i = 0; i < N; i++)
        {
            __shm_msgs.Push(__data, i);
        }
    }

    std::tuple<int32_t, T*> Allocate()
    {
        std::lock_guard<MutexLock> lck(__mutex);
        auto index = __shm_msgs.Pop(__data);
        if (index == -1)
            return {index, nullptr};
        else
            return {index, __data[index].__t.GetData()};
    }

    void SendMsg(int32_t index)
    {
        std::lock_guard<MutexLock> lck(__mutex);
        __shm_msgs_queue.Push(__data, index);
    }

    std::tuple<int32_t, T*> RecvMsg()
    {
        std::lock_guard<MutexLock> lck(__mutex);
        auto index = __shm_msgs_queue.Pop(__data);
        if (index == -1)
            return {index, nullptr};
        else
            return {index, __data[index].__t.GetData()};
    }

    void Free(int32_t index)
    {
        std::lock_guard<MutexLock> lck(__mutex);
        __shm_msgs.Push(__data, index);
    }

private:
    struct SHMMempoolElement
    {
        // bitset<64> __reader;
        int32_t __next = -1;
        int32_t __prev = -1;
        Element<T> __t;
    };
    struct ReaderInfo
    {
        int32_t __reader = -1;
        pid_t __pid      = 0;
    };
    MutexLock __mutex;
    SHMDeque<SHMMempoolElement> __shm_msgs;
    SHMDeque<SHMMempoolElement> __shm_msgs_queue;
    SHMMempoolElement __data[N]  = {};
    ReaderInfo __reader_infos[M] = {};
};

template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
class SHMMsgQueueManager
{
public:
    void Attach()
    {
        auto pid = getpid();
    }

private:
    struct SHMMsgQueueElement
    {
        int32_t __reader[64] = {0};
        T __t;
    };
    SHMMsgQueue<SHMMsgQueueElement, N> __queue;
    std::map<pid_t, int32_t> __pid2reader;
};

};  // namespace SHMV2