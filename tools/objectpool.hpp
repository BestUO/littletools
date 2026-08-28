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
            // __local_pool_ptr_wrap = std::make_unique<LocalPool>(this);
            __local_pool_ptr = new LocalPool(this);
            __local_pool_ptr_wrap.setptr(__local_pool_ptr);
        }
        return __local_pool_ptr->GetObject(std::forward<Args>(args)...);
    }

    void PutObject(T* ptr)
    {
        if (unlikely(!__local_pool_ptr))
        {
            // __local_pool_ptr_wrap = std::make_unique<LocalPool>(this);
            __local_pool_ptr = new LocalPool(this);
            __local_pool_ptr_wrap.setptr(__local_pool_ptr);
        }
        __local_pool_ptr->PutObject(ptr);
    }

    void ClearObjectPool()
    {
        __spine_chunk_lock.lock();
        for (auto& freechunk : __free_chunks)
            free(freechunk);
        __free_chunks.clear();
        __spine_chunk_lock.unlock();
        __spine_block_lock.lock();
        for (auto& block : __blocks)
            free(block);
        __blocks.clear();
        __spine_block_lock.unlock();
    }

private:
    ObjectPool()  = default;
    ~ObjectPool() = default;

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
                {
                }
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
            delete __ptr;
        }
        void setptr(PTRTYPE* ptr)
        {
            __ptr = ptr;
        }

        PTRTYPE* __ptr = nullptr;
    };
    static inline thread_local LocalPool* __local_pool_ptr = nullptr;
    // static inline thread_local std::unique_ptr<ObjectPool<T>::LocalPool>
    //     __local_pool_ptr=nullptr;
    static inline thread_local ThreadLocalPtrWrap<LocalPool>
        __local_pool_ptr_wrap = {};
    std::vector<FreeChunk*> __free_chunks;
    std::vector<Block*> __blocks;
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

__attribute__((destructor)) void inline ClearObjectPool()
{
    ObjectPool<int>::GetInstance()->ClearObjectPool();
}
}  // namespace v2

namespace v3
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
        if (unlikely(!__local_pool))
        {
            __local_pool = new LocalPool(this);
            __local_pool_ptr_wrap.setptr(__local_pool);
        }
        return __local_pool->GetObject(std::forward<Args>(args)...);
    }

    void PutObject(T* ptr)
    {
        if (unlikely(!__local_pool))
        {
            __local_pool = new LocalPool(this);
            __local_pool_ptr_wrap.setptr(__local_pool);
        }
        __local_pool->PutObject(ptr);
    }

private:
    ObjectPool() = default;
    ~ObjectPool()
    {
        for (auto& block : __blocks)
            free(block);
        for (auto& freechunk : __free_chunks)
            free(freechunk);
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

    class LocalPool
    {
    public:
        explicit LocalPool(ObjectPool* pool)
            : __pool(pool){};
        ~LocalPool()
        {
            if (likely(__free_chunk))
            {
                // if (likely(__free_chunk->nfree > 0))
                //     __pool->PutFreeChunk(__free_chunk);
                // else
                free(__free_chunk);
            }
        }

        template <typename... Args>
        T* GetObject(Args&&... args)
        {
            if (likely(__free_chunk))
            {
                if (likely(__free_chunk->nfree > 0))
                {
                }
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
            delete __ptr;
        }
        void setptr(LocalPool* ptr)
        {
            __ptr = ptr;
        }

    private:
        PTRTYPE* __ptr;
    };
    static thread_local LocalPool* __local_pool;
    // static thread_local std::unique_ptr<LocalPool> __local_pool;
    std::vector<FreeChunk*> __free_chunks;
    std::vector<Block*> __blocks;
    SpinLock __spine_chunk_lock;
    SpinLock __spine_block_lock;
    static thread_local ThreadLocalPtrWrap<LocalPool> __local_pool_ptr_wrap;
};

template <typename T>
thread_local typename ObjectPool<T>::LocalPool* ObjectPool<T>::__local_pool
    = nullptr;

template <typename T>
thread_local typename ObjectPool<T>::template ThreadLocalPtrWrap<
    typename ObjectPool<T>::LocalPool>
    ObjectPool<T>::__local_pool_ptr_wrap;
}  // namespace v3

namespace v4
{

class SpinLock
{
public:
    SpinLock()                           = default;
    ~SpinLock()                          = default;
    SpinLock(const SpinLock&)            = delete;
    SpinLock& operator=(const SpinLock&) = delete;

    void lock()
    {
        // Test-and-test-and-set: spin on a plain load first so contending
        // cores don't hammer the cache line with atomic RMW traffic while
        // the lock is held; only attempt the real test_and_set once the
        // lock looks free. Standard, well-known spinlock optimization.
        while (__lock.test_and_set(std::memory_order_acquire))
        {
            while (__lock_flag_hint.load(std::memory_order_relaxed))
                std::this_thread::yield();
        }
        __lock_flag_hint.store(true, std::memory_order_relaxed);
    }
    void unlock()
    {
        __lock_flag_hint.store(false, std::memory_order_relaxed);
        __lock.clear(std::memory_order_release);
    }

private:
    std::atomic_flag __lock = ATOMIC_FLAG_INIT;
    std::atomic<bool> __lock_flag_hint{false};
};

template <typename T>
class CACHE_ALIGN ObjectPool
{
public:
    static ObjectPool* GetInstance()
    {
        static ObjectPool instance;
        return &instance;
    }

    ObjectPool(const ObjectPool&)            = delete;
    ObjectPool& operator=(const ObjectPool&) = delete;

    template <typename... Args>
    T* GetObject(Args&&... args)
    {
        if (unlikely(!__local_pool_ptr))
        {
            __local_pool_ptr = std::make_unique<LocalPool>(this);
        }
        return __local_pool_ptr->GetObject(std::forward<Args>(args)...);
    }

    // Safe to call with nullptr (no-op).
    void PutObject(T* ptr)
    {
        if (unlikely(!ptr))
            return;
        if (unlikely(!__local_pool_ptr))
        {
            __local_pool_ptr = std::make_unique<LocalPool>(this);
        }
        __local_pool_ptr->PutObject(ptr);
    }

private:
    static_assert(std::is_destructible<T>::value, "T must be destructible");

    ObjectPool() = default;
    ~ObjectPool()
    {
        // Signal to any LocalPool that is still being torn down on another
        // thread (see RecycleFreeChunk) that it must not touch our lists
        // anymore, and take both locks so we don't race a concurrent
        // recycle attempt while we walk and free the lists below.
        __alive.store(false, std::memory_order_release);
        std::lock_guard<SpinLock> block_lck(__block_registry_lock);
        std::lock_guard<SpinLock> chunk_lck(__free_chunk_lock);

        Block* block = __block_registry_head;
        while (block)
        {
            Block* next = block->next;
            std::free(block);
            block = next;
        }
        __block_registry_head = nullptr;

        FreeChunk* chunk = __free_chunk_head;
        while (chunk)
        {
            FreeChunk* next = chunk->next;
            std::free(chunk);
            chunk = next;
        }
        __free_chunk_head = nullptr;
    }

    using Storage = typename std::aligned_storage<sizeof(T), alignof(T)>::type;

    struct FreeChunk
    {
        FreeChunk* next    = nullptr;
        unsigned int nfree = 0;
        T* ptrs[NITEM];
    };

    struct CACHE_ALIGN Block
    {
        Storage storage[NITEM];
        unsigned int nfree = NITEM;
        Block* next        = nullptr;
    };

    bool IsAlive() const
    {
        return __alive.load(std::memory_order_acquire);
    }

    FreeChunk* AcquireFreeChunk()
    {
        std::lock_guard<SpinLock> lck(__free_chunk_lock);
        FreeChunk* chunk = __free_chunk_head;
        if (chunk)
            __free_chunk_head = chunk->next;
        return chunk;
    }

    void RecycleFreeChunk(FreeChunk* chunk)
    {
        if (unlikely(!IsAlive()))
        {
            // Pool has already been torn down (this can only legitimately
            // happen if a worker thread outlives the ObjectPool<T>
            // singleton during process shutdown). Nothing to recycle
            // into anymore - just release the memory.
            std::free(chunk);
            return;
        }
        std::lock_guard<SpinLock> lck(__free_chunk_lock);
        chunk->next       = __free_chunk_head;
        __free_chunk_head = chunk;
    }

    Block* AcquireBlock()
    {
        Block* block = static_cast<Block*>(std::malloc(sizeof(Block)));
        if (unlikely(!block))
            throw std::bad_alloc();
        block->nfree = NITEM;
        std::lock_guard<SpinLock> lck(__block_registry_lock);
        block->next           = __block_registry_head;
        __block_registry_head = block;
        return block;
    }

    class CACHE_ALIGN LocalPool
    {
    public:
        explicit LocalPool(ObjectPool* pool)
            : __pool(pool)
        { }

        ~LocalPool()
        {
            if (__free_chunk)
            {
                if (__free_chunk->nfree > 0)
                    __pool->RecycleFreeChunk(__free_chunk);
                else
                    std::free(__free_chunk);
            }
            // __block (if any) is already tracked in the pool's global
            // block registry and will be freed when the ObjectPool<T>
            // itself is destroyed - it must not be freed here.
        }

        LocalPool(const LocalPool&)            = delete;
        LocalPool& operator=(const LocalPool&) = delete;

        template <typename... Args>
        T* GetObject(Args&&... args)
        {
            T* raw = AcquireRaw();
            try
            {
                return ::new (static_cast<void*>(raw))
                    T(std::forward<Args>(args)...);
            } catch (...)
            {
                // Construction failed: give the slot back instead of
                // losing it forever (basic exception safety).
                ReleaseRaw(raw);
                throw;
            }
        }

        void PutObject(T* ptr)
        {
            ptr->~T();
            ReleaseRaw(ptr);
        }

    private:
        T* AcquireRaw()
        {
            if (likely(__free_chunk) && likely(__free_chunk->nfree > 0))
                return __free_chunk->ptrs[--__free_chunk->nfree];

            if (__free_chunk)
            {
                std::free(__free_chunk);
                __free_chunk = nullptr;
            }
            __free_chunk = __pool->AcquireFreeChunk();
            if (likely(__free_chunk) && likely(__free_chunk->nfree > 0))
                return __free_chunk->ptrs[--__free_chunk->nfree];

            if (unlikely(!__block || __block->nfree == 0))
                __block = __pool->AcquireBlock();
            Storage* slot = &__block->storage[--__block->nfree];
            return reinterpret_cast<T*>(slot);
        }

        void ReleaseRaw(T* ptr)
        {
            if (!__free_chunk)
            {
                __free_chunk = AllocFreeChunk();
            }
            else if (unlikely(__free_chunk->nfree == NITEM))
            {
                __pool->RecycleFreeChunk(__free_chunk);
                __free_chunk = AllocFreeChunk();
            }
            __free_chunk->ptrs[__free_chunk->nfree++] = ptr;
        }

        static FreeChunk* AllocFreeChunk()
        {
            auto* chunk
                = static_cast<FreeChunk*>(std::malloc(sizeof(FreeChunk)));
            if (unlikely(!chunk))
                throw std::bad_alloc();
            chunk->next  = nullptr;
            chunk->nfree = 0;
            return chunk;
        }

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
            delete __ptr;
            __ptr = nullptr;
        }
        void setptr(PTRTYPE* ptr)
        {
            __ptr = ptr;
        }

    private:
        PTRTYPE* __ptr = nullptr;
    };

    std::atomic<bool> __alive{true};
    FreeChunk* __free_chunk_head = nullptr;
    Block* __block_registry_head = nullptr;
    SpinLock __free_chunk_lock;
    SpinLock __block_registry_lock;

    static inline thread_local std::unique_ptr<LocalPool> __local_pool_ptr;
};

}  // namespace v4