#pragma once

#include <cstdint>
#include <list>
#include <type_traits>
#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>
#include <atomic>
#include "../queue/threadsafecontainer.hpp"
#include "../queue/ringqueue.hpp"
#include "function_traits.hpp"

namespace threadpool
{

namespace v1
{

template <typename T>
class GetContainerType;

template <typename T>
struct GetContainerType<LockQueue<T>>
{
    typedef T Type;
};

template <typename T>
struct GetContainerType<FreeLockRingQueue<T>>
{
    typedef T Type;
};

template <class T>
class Worker
{
public:
    using ContainerType = typename T::Type;
    // using ContainerType = typename GetContainerType<T>::Type;
    Worker(std::shared_ptr<T> queue, unsigned int expireduration = 60)
        : _queue(queue)
        , _expire_duration(expireduration)
    { }
    void CreateWorker(bool original)
    {
        WorkerRun(original);
    }

    void StopWorker()
    {
        _stop = true;
        _queue->NoticeAllConsumer();
    }

protected:
    std::shared_ptr<T> _queue;
    bool _stop = false;
    unsigned int _expire_duration;
    virtual void WorkerRun(bool original)
    {
        auto worktime = std::chrono::system_clock::now();
        while (!_stop)
        {
            auto e = _queue->GetObjBulk();
            if (e)
            {
                worktime = std::chrono::system_clock::now();
                while (!e->empty())
                {
                    DealElement(std::move(e->front()));
                    e->pop();
                }
            }
            else
            {
                if (!original)
                {
                    if (worktime + std::chrono::seconds(_expire_duration)
                        > std::chrono::system_clock::now())
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
    virtual void DealElement(ContainerType&& p)
    {
        if constexpr (std::is_invocable_v<ContainerType>)
            p();
        else
            return;
    }
};

template <class QueueType>
class ThreadPool
{
public:
    ThreadPool(size_t minsize,
        size_t maxsize              = 10,
        unsigned int expireduration = 60)
        : __minsize(minsize)
        , __maxsize(maxsize)
    {
        if (!__queue)
            __queue = std::shared_ptr<QueueType>(new QueueType);

        if (!__worker)
            __worker = std::make_shared<WorkerDefault>(__queue, expireduration);

        for (size_t i = 0; i < __minsize; ++i)
            __workerthreads.emplace_back(std::move(CreateWorker(true)));
    }

    ThreadPool(std::shared_ptr<QueueType> queue,
        std::shared_ptr<Worker<QueueType>> worker,
        size_t minsize,
        size_t maxsize = 10)
        : __queue(queue)
        , __worker(worker)
        , __minsize(minsize)
        , __maxsize(maxsize)
    {
        for (size_t i = 0; i < __minsize; ++i)
            __workerthreads.emplace_back(std::move(CreateWorker(true)));
    }

    template <class F, class... Args>
    auto EnqueueFun(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type>
    {
        using return_type = typename std::result_of<F(Args...)>::type;
        auto task         = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));

        std::future<return_type> res = task->get_future();
        while (!__queue->AddObj([task]() {
            (*task)();
        }))
        {
            if (__totalnum < __maxsize)
                CreateWorker(false).detach();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return res;
    }

    template <class T>
    void EnqueueStr(T&& t)
    {
        while (!__queue->AddObj(std::move(t)))
        {
            if (__totalnum < __maxsize)
                CreateWorker(false).detach();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void StopThreadPool()
    {
        __worker->StopWorker();
    }

    ~ThreadPool()
    {
        for (std::thread& worker : __workerthreads)
            worker.join();
    }

private:
    size_t __minsize;
    size_t __maxsize;
    std::atomic<unsigned int> __totalnum;
    std::vector<std::thread> __workerthreads;
    std::shared_ptr<Worker<QueueType>> __worker;
    std::shared_ptr<QueueType> __queue;

    class WorkerDefault : public Worker<QueueType>
    {
    public:
        WorkerDefault(std::shared_ptr<QueueType> queue,
            unsigned int expireduration = 60)
            : Worker<QueueType>(queue, expireduration){};
    };

    std::thread CreateWorker(bool original)
    {
        std::thread t([this, original] {
            __totalnum++;
            __worker->CreateWorker(original);
            __totalnum--;
        });

        return t;
    }
};
}  // namespace v1
namespace v2
{
template <typename QueueType,
    std::enable_if_t<function_traits<typename QueueType::Type>::arity == 0, int>
        m
    = 0>
class ThreadPool
{
public:
    using FunType = typename QueueType::Type;
    using RetType = typename function_traits<FunType>::return_type;
    ThreadPool(size_t minsize,
        size_t maxsize              = 10,
        unsigned int expireduration = 60)
        : __minsize(minsize)
        , __maxsize(maxsize)
        , _expire_duration(expireduration)
    {
        auto a = function_traits<typename QueueType::Type>::arity;
        std::cout << a << std::endl;
        if (!_queue)
            _queue = std::shared_ptr<QueueType>(new QueueType);

        for (size_t i = 0; i < __minsize; ++i)
            __workerthreads.emplace_back(std::move(CreateWorker(true)));
    }

    ThreadPool(std::shared_ptr<QueueType> queue,
        size_t minsize,
        size_t maxsize = 10)
        : _queue(queue)
        , __minsize(minsize)
        , __maxsize(maxsize)
    {
        for (size_t i = 0; i < __minsize; ++i)
            __workerthreads.emplace_back(std::move(CreateWorker(true)));
    }

    auto EnqueueFun(FunType&& fun)
    {
        if constexpr (std::is_void_v<RetType>)
        {
            while (!_queue->AddObj(std::bind(std::forward<FunType>(fun))))
            {
                if (__totalnum < __maxsize)
                    CreateWorker(false).detach();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else
        {
            auto task = std::make_shared<std::packaged_task<RetType()>>(
                std::bind(std::forward<FunType>(fun)));
            while (!_queue->AddObj(std::function<RetType()>([task]() {
                (*task)();
                return RetType();
            })))
            {
                if (__totalnum < __maxsize)
                    CreateWorker(false).detach();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            return task->get_future();
        }
    }

    virtual ~ThreadPool()
    {
        for (std::thread& worker : __workerthreads)
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
        while (!_stop)
        {
            auto e = _queue->GetObjBulk();
            if (e)
            {
                worktime = std::chrono::system_clock::now();
                while (!e->empty())
                {
                    DealElement(std::move(e->front()));
                    e->pop();
                }
            }
            else
            {
                if (!original)
                {
                    if (worktime + std::chrono::seconds(_expire_duration)
                        > std::chrono::system_clock::now())
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

    template <typename TTtype>
    void DealElement(TTtype&& p)
    {
        if constexpr (std::is_invocable_v<TTtype>)
            p();
        else
        {
            return;
        }
    }

protected:
    std::shared_ptr<QueueType> _queue;
    unsigned int _expire_duration = 60;
    bool _stop                    = false;

private:
    size_t __minsize;
    size_t __maxsize;
    std::atomic<unsigned int> __totalnum;
    std::vector<std::thread> __workerthreads;

    std::thread CreateWorker(bool original)
    {
        std::thread t([this, original] {
            __totalnum++;
            WorkerRun(original);
            __totalnum--;
        });

        return t;
    }
};
}  // namespace v2

namespace v3
{
template <typename T>
class Worker
{
public:
    void Put(T&& t)
    {
        std::lock_guard<std::mutex> lock(__mutex);
        __vector.emplace_back(std::move(t));
        __cond.notify_one();
    }

    std::vector<T> Get()
    {
        std::unique_lock<std::mutex> lock(__mutex);
        __cond.wait(lock);
        return std::move(__vector);
    }

    void Stop()
    {
        std::lock_guard<std::mutex> lock(__mutex);
        __cond.notify_one();
    }

private:
    std::mutex __mutex;
    std::condition_variable __cond;
    std::vector<T> __vector;
};

template <typename T>
class ThreadPoll
{
public:
    void Put(uint64_t hash, T&& t)
    {
        __workers[hash % __workers.size()]->Put(std::move(t));
    }

    void Start(uint8_t threadnum, std::function<void(T&&)> func)
    {
        __stop = false;
        for (uint8_t i = 0; i < threadnum; ++i)
        {
            auto worker = std::make_shared<Worker<T>>();
            __workers.emplace_back(worker);
            __threads.emplace_back([this, worker, func] {
                while (!__stop)
                {
                    auto elements = worker->Get();
                    for (auto& element : elements)
                        func(std::move(element));
                }
            });
        }
    }

    void Stop()
    {
        __stop = true;
        for (auto& worker : __workers)
            worker->Stop();
        for (auto& thread : __threads)
            thread.join();
    }

private:
    bool __stop = true;
    std::vector<std::thread> __threads;
    std::vector<std::shared_ptr<Worker<T>>> __workers;
};

};  // namespace v3
}  // namespace threadpool