#pragma once
#include <thread>
#include <mutex>
#include <memory>
#include <vector>

#define NITEM 1024
#define CACHE_LINE 64
#define CACHE_ALIGN __attribute__((__aligned__(CACHE_LINE)))

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
        if (__freechunks.empty())
            return nullptr;
        else
        {
            auto freechunk = __freechunks.back();
            __freechunks.pop_back();
            return freechunk;
        }
    }

    void PutFreeChunk(std::shared_ptr<FreeChunk> freechunk)
    {
        std::lock_guard<std::mutex> lck(__mutex);
        __freechunks.emplace_back(freechunk);
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
            if (!__freechunk || !__freechunk->nfree)
                __freechunk = __pool->GetFreeChunk();
            if (__freechunk && __freechunk->nfree)
                return new (__freechunk->ptrs[--__freechunk->nfree]) T(args...);
            else
            {
                auto ptr = __pool->GetPtrFromBlock();
                return new (ptr) T(args...);
            }
            return nullptr;
        }
        void PutObject(T* ptr)
        {
            if (__freechunk)
            {
                if (__freechunk->nfree == NITEM)
                {
                    __pool->PutFreeChunk(__freechunk);
                    __freechunk = std::make_shared<FreeChunk>();
                }
            }
            else
                __freechunk = std::make_shared<FreeChunk>();
            __freechunk->ptrs[__freechunk->nfree++] = ptr;
        }

    private:
        std::shared_ptr<FreeChunk> __freechunk = nullptr;
        ObjectPool* __pool;
    };
    static thread_local std::shared_ptr<LocalPool> __local_pool;
    std::vector<std::shared_ptr<FreeChunk>> __freechunks;
    std::vector<std::shared_ptr<Block>> __blocks;
    std::mutex __mutex;
};

template <typename T>
thread_local std::shared_ptr<typename ObjectPool<T>::LocalPool>
    ObjectPool<T>::__local_pool = nullptr;