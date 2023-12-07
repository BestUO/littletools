#pragma once

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
template <class QueueType>
class ThreadPool
{
public:
    using FunType = typename QueueType::Type;
    ThreadPool(size_t minsize,
        size_t maxsize              = 10,
        unsigned int expireduration = 60)
        : __minsize(minsize)
        , __maxsize(maxsize)
        , _expire_duration(expireduration)
    {
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

    void EnqueueFun(FunType&& fun)
    {
        while (!_queue->AddObj(std::move(fun)))
        {
            if (__totalnum < __maxsize)
                CreateWorker(false).detach();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

    void DealElement(FunType&& fun)
    {
        if constexpr (std::is_invocable_v<FunType>)
            fun();
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
template <class QueueType>
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

    // template <class F, class... Args>
    // auto EnqueueFun(F&& f, Args&&... args)
    //     -> std::future<typename std::result_of<F(Args...)>::type>
    // {
    //     using return_type = typename std::result_of<F(Args...)>::type;
    //     auto task         =
    //     std::make_shared<std::packaged_task<return_type()>>(
    //         std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    //     std::future<return_type> res = task->get_future();
    //     while (!_queue->AddObj([task]() {
    //         (*task)();
    //     }))
    //     {
    //         if (__totalnum < __maxsize)
    //             CreateWorker(false).detach();
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    //     return res;
    // }

    // void EnqueueFun(FunType&& t)
    // {
    //     while (!_queue->AddObj(std::move(t)))
    //     {
    //         if (__totalnum < __maxsize)
    //             CreateWorker(false).detach();
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // }

    // template <std::enable_if_t<std::is_void_v<RetType>, int> = 1>
    // void EnqueueFun(FunType&& t)
    // {
    //     while (!_queue->AddObj(std::move(t)))
    //     {
    //         if (__totalnum < __maxsize)
    //             CreateWorker(false).detach();
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // }

    // template <std::enable_if_t<!std::is_void_v<RetType>, int> = 1>
    // RetType EnqueueFun(FunType&& t)
    // {
    //     //     while (!_queue->AddObj(std::move(t)))
    //     //     {
    //     //         if (__totalnum < __maxsize)
    //     //             CreateWorker(false).detach();
    //     // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     //     }
    //     return RetType();
    // }

    auto EnqueueFun(FunType&& fun)
    {
        if constexpr (std::is_void_v<RetType>)
        {
            while (!_queue->AddObj(std::move(fun)))
            {
                if (__totalnum < __maxsize)
                    CreateWorker(false).detach();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else
        {
            auto task = std::make_shared<std::packaged_task<RetType()>>(
                std::forward<FunType>(fun));
            while (!_queue->AddObj(std::function<RetType()>([task]() {
                RetType a = (*task)();
                return RetType();
            })))
            // auto task = std::packaged_task<RetType()>(fun);
            // while (!_queue->AddObj(std::function<RetType()>([task]() {
            //     RetType a = task();
            //     return RetType();
            // })))
            {
                if (__totalnum < __maxsize)
                    CreateWorker(false).detach();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            std::future<RetType> res = task->get_future();
            return res.get();
        }
    }

    // std::enable_if<!std::is_void_v<TTtype>, int> EnqueueFun(TTtype&& t)
    // {
    //     while (!_queue->AddObj(std::move(t)))
    //     {
    //         if (__totalnum < __maxsize)
    //             CreateWorker(false).detach();
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    //     return 1;
    // }

    // template <class T>
    // auto EnqueueFun(T&& t) -> std::future<typename std::result_of<T()>::type>
    // {
    //     using return_type = typename std::result_of<T()>::type;
    //     auto task         =
    //     std::make_shared<std::packaged_task<return_type()>>(
    //         std::forward<T>(t));

    //     while (!_queue->AddObj([task]() -> return_type {
    //         return (*task)();
    //     }))
    //     {
    //         if (__totalnum < __maxsize)
    //             CreateWorker(false).detach();
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    //     return task->get_future();
    // }

    // template <class T>
    // void EnqueueFun2(T&& t)
    // {
    //     while (!_queue->AddObj(std::move(t)))
    //     {
    //         if (__totalnum < __maxsize)
    //             CreateWorker(false).detach();
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // }

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

    // template<typename TTtype>
    // void DealElement(TTtype &&p)
    // {
    //     if constexpr (std::is_function_v<std::remove_pointer_t<TTtype>>)
    //         p();
    //     else
    //         return;
    // }

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

    // virtual typename std::enable_if<std::is_same<TTtype,
    // std::function<void()>>::value>::type DealElement(TTtype &&p)
    // {
    //     p();
    // }
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
}  // namespace v3
}  // namespace threadpool