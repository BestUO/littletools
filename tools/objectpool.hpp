#pragma once
#include <thread>
#include <mutex>

template <typename T>
class ObjectPool
{
public:
    static ObjectPool *GetInstance()
    {
        static ObjectPool instance;
        return &instance;
    }
    template <typename ...Args>
    T* GetObject(Args ...args)
    {
        return __local_pool.GetObject(args...);
    }

    void ReturnObject(T* ptr)
    {
        __local_pool.ReturnObject(ptr);
    }
private:
    ObjectPool()=default;
    ~ObjectPool()=default;
    static const int NITEM=1024;

    struct FreeChunk 
    {
        unsigned int nfree=0;
        T* ptrs[NITEM];
        FreeChunk()
        {
            nfree = NITEM;
            ptrs = malloc(sizeof(T)*NITEM);
        }
        ~FreeChunk()
        {
            free(ptrs);
        }
    }; 

    std::shared_ptr<FreeChunk> GetFreeChunk()
    {
        std::lock_guard<std::mutex> lck (__mutex);
        if(__v_freechunk.empty())
            __v_freechunk.emplacee_back(std::make_shared<FreeChunk>());

        auto freechunk = __v_freechunk.back();
        __v_freechunk.pop_back();
        return freechunk;
    }

    class LocalPool 
    {
    public:
        template <typename ...Args>
        T* GetObject(Args ...args)
        {
            if(!__freechunk || !__freechunk->nfree)
                __freechunk = GetFreeChunk();

            return new (__freechunk->ptrs[--__freechunk->nfree]) T(args...);
        }
        void ReturnObject(T* ptr)
        {
            
        }
    private:
        std::shared_ptr<FreeChunk> __freechunk=nullptr;
    };
    static thread_local LocalPool __local_pool;
    std::vector<std::shared_ptr<FreeChunk>> __v_freechunk;
    std::mutex __mutex;
};