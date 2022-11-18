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
#include "../queue/ringqueue.hpp"
#include <atomic>

template<class QueueType>
class ThreadPool2
{
public:
    using TTtype = typename QueueType::Type;
    ThreadPool2(size_t minsize, size_t maxsize=10, unsigned int expireduration=60):__minsize(minsize),__maxsize(maxsize),_expire_duration(expireduration)
    {
        if(!_queue)
            _queue = std::shared_ptr<QueueType>(new QueueType);

        for(size_t i = 0;i<__minsize;++i)
            __workerthreads.emplace_back(std::move(CreateWorker(true)));
    }

    ThreadPool2(std::shared_ptr<QueueType> queue, size_t minsize, size_t maxsize=10):
                _queue(queue),__minsize(minsize),__maxsize(maxsize)
    {
        for(size_t i = 0;i<__minsize;++i)
            __workerthreads.emplace_back(std::move(CreateWorker(true)));
    }

    template<class F, class... Args>
    auto EnqueueFun(F&& f, Args&&... args) -> std::future<typename std::result_of<F(Args...)>::type>
    {
        using return_type = typename std::result_of<F(Args...)>::type;
        auto task = std::make_shared< std::packaged_task<return_type()> >(
                std::bind(std::forward<F>(f), std::forward<Args>(args)...)
            );
        
        std::future<return_type> res = task->get_future();
        while(!_queue->AddObj([task](){ (*task)(); }))
        {
            if(__totalnum < __maxsize)
                CreateWorker(false).detach();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return res;
    }

    template<class T>
    void EnqueueStr(T&& t)
    {
        while(!_queue->AddObj(std::move(t)))
        {
            if(__totalnum < __maxsize)
                CreateWorker(false).detach();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    virtual ~ThreadPool2()
    {
        for(std::thread &worker: __workerthreads)
            worker.join();
    }

    void StopThreadPool()
    {
        _stop = true;
        _queue->NoticeAllConsumer();
    }

    virtual void WorkerRun(bool original)
    {
        auto worktime = std::chrono::system_clock::now();
        while(!_stop)
        {
            auto e = _queue->GetObjBulk();
            if(e)
            {
                worktime = std::chrono::system_clock::now();
                while(!e->empty())
                {
                    DealElement(std::move(e->front()));
                    e->pop();
                }
            }
            else
            {
                if(!original)
                {
                    if(worktime + std::chrono::seconds(_expire_duration) > std::chrono::system_clock::now())
                    {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        continue;
                    }
                    else
                        break;
                }
                _queue->WaitComingObj();
            }
        }
    }

    // template<typename TTtype>
    // void DealElement(TTtype &&p)
    // {
    //     if constexpr (std::is_function_v<std::remove_pointer_t<TTtype>>)
    //         p();
    //     else
    //         return;
    // }

    template<typename TTtype>
    void DealElement(TTtype &&p)
    {
        if constexpr (std::is_invocable_v<TTtype>)
            p();
        else
        {
            int aaa=10;
            return;
        }
    }

    // virtual typename std::enable_if<std::is_same<TTtype, std::function<void()>>::value>::type
    // DealElement(TTtype &&p)
    // {
    //     p();
    // }
protected:
    std::shared_ptr<QueueType> _queue;
    unsigned int _expire_duration=60;
    bool _stop = false;

private:
    size_t __minsize;
    size_t __maxsize;
    std::atomic<unsigned int> __totalnum;
    std::vector< std::thread > __workerthreads;

    std::thread CreateWorker(bool original)
    {
        std::thread t([this,original]
        {
            __totalnum++;
            WorkerRun(original);
            __totalnum--;
        });
        
        return t;
    }
};