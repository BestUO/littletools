#pragma once

#include "tools/threadpool.hpp"
#include "tools/jsonwrap.hpp"

class DMThreadPool
{
using QueueType = std::conditional_t<false, LockQueue<std::function<void()>>, FreeLockRingQueue<std::function<void()>>>;
public:
    static DMThreadPool *GetInstance()
    {
        static DMThreadPool instance;
        return &instance;
    }
    auto GetThreadPool()
    {
        return __threadpool;
    }
private:
    DMThreadPool()
    {
        std::shared_ptr<ThreadPool<QueueType>> threadpool(new ThreadPool<QueueType>(2, 10));
        __threadpool = threadpool;
    }
    
    ~DMThreadPool()=default;
    std::shared_ptr<ThreadPool<QueueType>> __threadpool;
};