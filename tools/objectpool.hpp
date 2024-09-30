#pragma once
#include <atomic>
#include <thread>
#include <mutex>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>

#define NITEM 1024
#define CACHE_LINE 64
#ifdef _MSVC_LANG
    #define CACHE_ALIGN __declspec(align(CACHE_LINE))
#else
    #define CACHE_ALIGN __attribute__((__aligned__(CACHE_LINE)))
#endif
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

namespace v1
{
template <typename T>
class ObjectPool
{
public:
    static ObjectPool* GetInstance()
    {
        static ObjectPool instance;
        return &instance;
    }
    template <typename... Args>
    T* GetObject(Args... args)
    {
        if (!__local_pool)
            __local_pool = std::make_shared<LocalPool>(this);
        return __local_pool->GetObject(args...);
    }

    void PutObject(T* ptr)
    {
        if (!__local_pool)
            __local_pool = std::make_shared<LocalPool>(this);
        __local_pool->PutObject(ptr);
    }

private:
    ObjectPool()  = default;
    ~ObjectPool() = default;

    struct FreeChunk
    {
        unsigned int nfree = 0;
        T* ptrs[NITEM];
    };

    struct CACHE_ALIGN Block
    {
        std::shared_ptr<T[]> ptrs{new T[NITEM]};
        unsigned int nfree = NITEM;
    };
    std::shared_ptr<FreeChunk> GetFreeChunk()
    {
        std::lock_guard<std::mutex> lck(__mutex);
        if (__free_chunks.empty())
            return nullptr;
        else
        {
            auto freechunk = __free_chunks.back();
            __free_chunks.pop_back();
            return freechunk;
        }
    }

    void PutFreeChunk(std::shared_ptr<FreeChunk> freechunk)
    {
        std::lock_guard<std::mutex> lck(__mutex);
        __free_chunks.emplace_back(freechunk);
    }

    std::shared_ptr<Block> GetBlock()
    {
        if (__blocks.empty())
            __blocks.emplace_back(std::make_shared<Block>());
        else
        {
            auto block = __blocks.back();
            if (!block->nfree)
                __blocks.emplace_back(std::make_shared<Block>());
        }
        return __blocks.back();
    }

    T* GetPtrFromBlock()
    {
        std::lock_guard<std::mutex> lck(__mutex);
        auto block = GetBlock();
        return &block->ptrs[--block->nfree];
    }

    class LocalPool
    {
    public:
        explicit LocalPool(ObjectPool* pool)
            : __pool(pool){};
        template <typename... Args>
        T* GetObject(Args... args)
        {
            if (!__free_chunk || !__free_chunk->nfree)
                __free_chunk = __pool->GetFreeChunk();
            if (__free_chunk && __free_chunk->nfree)
                return new (__free_chunk->ptrs[--__free_chunk->nfree])
                    T(args...);
            else
            {
                auto ptr = __pool->GetPtrFromBlock();
                return new (ptr) T(args...);
            }
            return nullptr;
        }
        void PutObject(T* ptr)
        {
            if (__free_chunk)
            {
                if (__free_chunk->nfree == NITEM)
                {
                    __pool->PutFreeChunk(__free_chunk);
                    __free_chunk = std::make_shared<FreeChunk>();
                }
            }
            else
                __free_chunk = std::make_shared<FreeChunk>();
            __free_chunk->ptrs[__free_chunk->nfree++] = ptr;
        }

    private:
        std::shared_ptr<FreeChunk> __free_chunk = nullptr;
        ObjectPool* __pool;
    };
    static_assert(std::is_default_constructible<T>::value,
        "T must be default constructible");
    static thread_local std::shared_ptr<LocalPool> __local_pool;
    std::vector<std::shared_ptr<FreeChunk>> __free_chunks;
    std::vector<std::shared_ptr<Block>> __blocks;
    std::mutex __mutex;
};

template <typename T>
thread_local std::shared_ptr<typename ObjectPool<T>::LocalPool>
    ObjectPool<T>::__local_pool = nullptr;
}  // namespace v1

inline namespace v2
{

class SpinLock
{
public:
    SpinLock()  = default;
    ~SpinLock() = default;
    void lock()
    {
        while (__lock.test_and_set(std::memory_order_acquire))
            ;
    }
    void unlock()
    {
        __lock.clear(std::memory_order_release);
    }

private:
    std::atomic_flag __lock = ATOMIC_FLAG_INIT;
};

// class SpinLock
// {
// public:
//     SpinLock()  = default;
//     ~SpinLock() = default;
//     void lock()
//     {
//         __mutex.lock();
//     }
//     void unlock()
//     {
//         __mutex.unlock();
//     }

// private:
//     std::mutex __mutex;
// };

// class SpinLock
// {
// public:
//     explicit SpinLock(std::int32_t count = 1024) noexcept
//         : _spinCount(count)
//         , _locked(false)
//     { }

//     bool tryLock() noexcept
//     {
//         return !_locked.exchange(true, std::memory_order_acquire);
//     }

//     void lock() noexcept
//     {
//         auto counter = _spinCount;
//         while (!tryLock())
//         {
//             while (_locked.load(std::memory_order_relaxed))
//             {
//                 if (counter-- <= 0)
//                 {
//                     std::this_thread::yield();
//                     counter = _spinCount;
//                 }
//             }
//         }
//     }

//     void unlock() noexcept
//     {
//         _locked.store(false, std::memory_order_release);
//     }

// private:
//     std::int32_t _spinCount;
//     std::atomic<bool> _locked;
// };

template <typename T>
class CACHE_ALIGN ObjectPool
{
public:
    static ObjectPool* GetInstance()
    {
        static ObjectPool instance;
        return &instance;
    }
    template <typename... Args>
    T* GetObject(Args&&... args)
    {
        if (unlikely(!__local_pool_ptr))
        {
            // __local_pool_ptr = std::make_unique<LocalPool>(this);
            __local_pool_ptr = new LocalPool(this);
            __local_pools.push_back(__local_pool_ptr);
            // __local_pool_ptr_wrap.setptr(__local_pool_ptr);
        }
        return __local_pool_ptr->GetObject(std::forward<Args>(args)...);
    }

    void PutObject(T* ptr)
    {
        if (unlikely(!__local_pool_ptr))
        {
            // __local_pool_ptr = std::make_unique<LocalPool>(this);
            __local_pool_ptr = new LocalPool(this);
            __local_pools.push_back(__local_pool_ptr);
            // __local_pool_ptr_wrap.setptr(__local_pool_ptr);
        }
        __local_pool_ptr->PutObject(ptr);
    }

private:
    ObjectPool() = default;
    ~ObjectPool()
    {
        for (auto& block : __blocks)
            free(block);
        for (auto& freechunk : __free_chunks)
            free(freechunk);
        for (auto& localpool : __local_pools)
            delete localpool;
    }

    struct FreeChunk
    {
        T* ptrs[NITEM];
        unsigned int nfree = 0;
    };

    struct CACHE_ALIGN Block
    {
        T ptrs[NITEM];
        unsigned int nfree = NITEM;
    };

    FreeChunk* GetFreeChunk()
    {
        __spine_chunk_lock.lock();
        if (unlikely(__free_chunks.size() == 0))
        {
            __spine_chunk_lock.unlock();
            return nullptr;
        }
        else
        {
            auto freechunk = __free_chunks.back();
            __free_chunks.pop_back();
            __spine_chunk_lock.unlock();
            return freechunk;
        }
    }

    void PutFreeChunk(FreeChunk* freechunk)
    {
        __spine_chunk_lock.lock();
        __free_chunks.emplace_back(freechunk);
        __spine_chunk_lock.unlock();
    }

    Block* GetBlock()
    {
        auto block   = (Block*)malloc(sizeof(Block));
        block->nfree = NITEM;
        __spine_block_lock.lock();
        __blocks.emplace_back(block);
        __spine_block_lock.unlock();
        return block;
    }

    class CACHE_ALIGN LocalPool
    {
    public:
        explicit LocalPool(ObjectPool* pool)
            : __pool(pool){};
        ~LocalPool()
        {
            if (likely(__free_chunk))
            {
                if (likely(__free_chunk->nfree > 0))
                    __pool->PutFreeChunk(__free_chunk);
                else
                    free(__free_chunk);
            }
        }

        template <typename... Args>
        T* GetObject(Args&&... args)
        {
            if (likely(__free_chunk))
            {
                if (likely(__free_chunk->nfree > 0))
                { }
                else
                {
                    free(__free_chunk);
                    __free_chunk = __pool->GetFreeChunk();
                }
            }
            else
                __free_chunk = __pool->GetFreeChunk();

            if (likely(__free_chunk))
                return new (__free_chunk->ptrs[--__free_chunk->nfree])
                    T(std::forward<Args>(args)...);
            if (unlikely(!__block || !__block->nfree))
                __block = __pool->GetBlock();
            return new (&__block->ptrs[--__block->nfree])
                T(std::forward<Args>(args)...);
        }

        void PutObject(T* ptr)
        {
            if (likely(__free_chunk))
            {
                if (unlikely(__free_chunk->nfree == NITEM))
                {
                    __pool->PutFreeChunk(__free_chunk);
                    __free_chunk        = (FreeChunk*)malloc(sizeof(FreeChunk));
                    __free_chunk->nfree = 0;
                }
            }
            else
            {
                __free_chunk        = (FreeChunk*)malloc(sizeof(FreeChunk));
                __free_chunk->nfree = 0;
            }
            if (likely(std::is_destructible<T>::value))
                ptr->~T();
            __free_chunk->ptrs[__free_chunk->nfree++] = ptr;
        }

    private:
        FreeChunk* __free_chunk = nullptr;
        Block* __block          = nullptr;
        ObjectPool* __pool;
    };

    template <typename PTRTYPE>
    class ThreadLocalPtrWrap
    {
    public:
        ~ThreadLocalPtrWrap()
        {
            // delete __ptr;
        }
        // void setptr(PTRTYPE* ptr)
        // {
        //     // __ptr = ptr;
        // }

        // PTRTYPE* __ptr = nullptr;
    };
    static inline thread_local LocalPool* __local_pool_ptr = nullptr;
    // static inline thread_local std::unique_ptr<ObjectPool<T>::LocalPool>
    //     __local_pool_ptr;
    // static inline thread_local ThreadLocalPtrWrap<LocalPool>
    //     __local_pool_ptr_wrap = {};
    std::vector<FreeChunk*> __free_chunks;
    std::vector<Block*> __blocks;
    std::vector<LocalPool*> __local_pools;
    SpinLock __spine_chunk_lock;
    SpinLock __spine_block_lock;
};

// template <typename T>
// thread_local typename ObjectPool<T>::LocalPool*
// ObjectPool<T>::__local_pool_ptr
//     = nullptr;

// template <typename T>
// thread_local std::unique_ptr<typename ObjectPool<T>::LocalPool>
//     ObjectPool<T>::__local_pool_ptr;

// template <typename T>
// thread_local typename ObjectPool<T>::template ThreadLocalPtrWrap<
//     typename ObjectPool<T>::LocalPool>
//     ObjectPool<T>::__local_pool_ptr_wrap;
}  // namespace v2