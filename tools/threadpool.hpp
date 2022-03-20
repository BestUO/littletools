#pragma once

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>
#include "../queue/threadsafecontainer.hpp"
#include <atomic>

class ThreadPool {
public:
    ThreadPool(size_t minsize, size_t maxsize=32);
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type>;
    ~ThreadPool();
private:
    size_t minsize;
    size_t maxsize;
    // need to keep track of threads so we can join them
    std::vector< std::thread > workers;
    // the task queue
    // std::queue< std::function<void()> > tasks;
    LockQueue<std::function<void()>,10240> __queuetask;
    
    // synchronization
    // std::mutex mtx;
    // std::condition_variable cv;
    std::atomic<unsigned int> totalnum;
    std::thread CreateWorker(bool original);
};

inline std::thread ThreadPool::CreateWorker(bool original)
{
    std::thread t( [this,original]
    {
        totalnum++;
        for(;;)
        {
            auto [flag, e] = __queuetask.GetObjBulk();
            if(flag)
            {
                while(!e.empty())
                {
                    e.front()();
                    e.pop();
                }
            }
            else
            {
                if(!original)
                    break;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        totalnum--;
    });
    
    return t;
}
 
// the constructor just launches some amount of workers
inline ThreadPool::ThreadPool(size_t minsize, size_t maxsize):minsize(minsize),maxsize(maxsize)
{
    for(size_t i = 0;i<minsize;++i)
        workers.emplace_back(std::move(CreateWorker(true)));
}

// add new work item to the pool
template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args) 
    -> std::future<typename std::result_of<F(Args...)>::type>
{
    using return_type = typename std::result_of<F(Args...)>::type;

    // with -std=c++2b you can also code as:
    // auto task = std::packaged_task<return_type()>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
    // while(!__queuetask.AddObj([task=std::move(task)]()mutable{ (*task)(); }))
    // {...}
    
    auto task = std::make_shared< std::packaged_task<return_type()> >(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
    
    std::future<return_type> res = task->get_future();
    while(!__queuetask.AddObj([task](){ (*task)(); }))
    {
        if(totalnum < maxsize)
            CreateWorker(false).detach();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // condition.notify_one();
    return res;
}

// the destructor joins all threads
inline ThreadPool::~ThreadPool()
{
    // condition.notify_all();
    for(std::thread &worker: workers)
        worker.join();
}