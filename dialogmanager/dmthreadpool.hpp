#pragma once

#include "tools/threadpool.hpp"
#include "tools/jsonwrap.hpp"
class DMThreadPool
{
    using QueueType = std::conditional_t<false,
        LockQueue<std::function<void()>>,
        FreeLockRingQueue<std::function<void()>>>;

public:
    static DMThreadPool* GetInstance()
    {
        static DMThreadPool instance;
        return &instance;
    }
    auto GetThreadPool()
    {
        return __threadpool;
    }
    void StopThreadPool()
    {
        __threadpool->StopThreadPool();
    };

private:
    DMThreadPool()
    {
        auto config
            = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json")
                  .value();
        std::shared_ptr<threadpool::v1::ThreadPool<QueueType>> threadpool(
            new threadpool::v1::ThreadPool<QueueType>(
                config["thread_pool"]["min_size"].GetInt(),
                config["thread_pool"]["max_size"].GetInt()));

        __threadpool = threadpool;
    }

    ~DMThreadPool() = default;

    std::shared_ptr<threadpool::v1::ThreadPool<QueueType>> __threadpool;
};