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

class ThreadPool {
public:
    ThreadPool(size_t);
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type>;
    ~ThreadPool();
private:
    // need to keep track of threads so we can join them
    std::vector< std::thread > workers;
    // the task queue
    // std::queue< std::function<void()> > tasks;
    LockQueue<std::function<void()>,10240> __queuetask;
    
    // synchronization
    std::mutex queue_mutex;
    std::condition_variable condition;
};
 
// the constructor just launches some amount of workers
inline ThreadPool::ThreadPool(size_t threads)
{
    for(size_t i = 0;i<threads;++i)
        workers.emplace_back(
            [this]
            {
                for(;;)
                {
                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        this->condition.wait(lock,[this]{ return !__queuetask.IsEmpty(); });
                    }
                    
                    auto [flag, e] = __queuetask.GetObjBulk();
                    if(flag)
                    {
                        while(!e.empty())
                        {
                            e.front()();
                            e.pop();
                        }
                    }
                    // else
                    //     std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        );
}

// add new work item to the pool
template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args) 
    -> std::future<typename std::result_of<F(Args...)>::type>
{
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared< std::packaged_task<return_type()> >(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        
    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        __queuetask.AddObj([task](){ (*task)(); });
    }
    condition.notify_one();
    return res;
}

// the destructor joins all threads
inline ThreadPool::~ThreadPool()
{
    condition.notify_all();
    for(std::thread &worker: workers)
        worker.join();
}