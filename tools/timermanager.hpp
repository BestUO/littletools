#pragma once

#include <chrono>
#include <thread>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <map>
#include "../queue/threadsafecontainer.hpp"
#include "rbtree/rbtreewrap.hpp"
#include "objectpool.hpp"

namespace timermanager
{
namespace v1
{
template <typename T>
class TimerManager
{
public:
    static TimerManager* GetInstance()
    {
        static TimerManager instance;
        return &instance;
    }

    void AddAlarm(std::chrono::milliseconds alarm,
        T key,
        std::function<void()> fun,
        std::chrono::milliseconds interval = std::chrono::milliseconds(0),
        bool runinmainthread               = true)
    {
        TIMETYPE::time_point timepoint = TIMETYPE::now() + alarm;
        __timer_queue.AddObj(
            {{}, timepoint, fun, key, interval, runinmainthread});
        __cv.notify_one();
    }

    void DeleteAlarm(T&& key)
    {
        __timer_queue.DeleteObj([key = key](const TimerElement& timerelement) {
            return timerelement.key == key;
        });
        __cv.notify_one();
    }

    void DeleteAlarmCondition(std::function<bool(const T& key)> fun)
    {
        __timer_queue.DeleteObj([fun = fun](const TimerElement& timerelement) {
            return fun(timerelement.key);
        });
        __cv.notify_one();
    }

    void StopTimerManager()
    {
        __stop = true;
        __cv.notify_one();
        if (__timer_thread.joinable())
        {
            __timer_thread.join();
        }
    }

    void StartTimerManager()
    {
        if (!__stop)
        {
            return;
        }
        __stop         = false;
        __timer_thread = std::thread(&TimerManager::RunTimerManager, this);
    }

private:
    using TIMETYPE = std::chrono::steady_clock;
    struct TimerElement
    {
        rb_node rbNode = {nullptr, nullptr, nullptr, RB_RED};
        TIMETYPE::time_point alarm;
        std::function<void()> fun;
        T key;
        std::chrono::milliseconds interval = std::chrono::milliseconds(0);
        bool run_in_main_thread            = true;
        bool operator<(const TimerElement& t) const
        {
            return alarm < t.alarm;
        }
    };

    std::atomic<bool> __stop = {true};
    rbtreewrap::v1::RBTreeWrap<TimerElement> __timer_queue;
    std::thread __timer_thread;
    std::condition_variable __cv;
    std::mutex __mutex;

    TimerManager()
    {
        StartTimerManager();
    }

    ~TimerManager()
    {
        StopTimerManager();
    }

    void RunTimerManager()
    {
        pthread_setname_np(pthread_self(), "ACCM-TimerManager");
        while (!__stop)
        {
            std::unique_lock<std::mutex> lck(__mutex);
            auto timepointopt = __timer_queue.GetTopObjKey(
                std::function<TIMETYPE::time_point(TimerElement*)>(
                    [](TimerElement* element) {
                        return element->alarm;
                    }));

            if (timepointopt.has_value())
            {
                __cv.wait_until(lck, timepointopt.value());
            }
            else
            {
                __cv.wait_until(lck, TIMETYPE::now() + std::chrono::seconds(1));
            }

            auto elementopt
                = __timer_queue.GetTopObjAndDelete([](TimerElement* e) {
                      return TIMETYPE::now() > e->alarm;
                  });
            if (elementopt.has_value())
            {
                auto element = elementopt.value();
                if (element.interval > std::chrono::milliseconds(0))
                    AddAlarm(element.interval,
                        element.key,
                        element.fun,
                        element.interval,
                        element.run_in_main_thread);
                if (element.fun)
                {
                    if (element.run_in_main_thread)
                    {
                        element.fun();
                    }
                    else
                    {
                        std::thread tmp([fun = element.fun]() {
                            fun();
                        });
                        tmp.detach();
                    }
                }
            }
        }
        while (__timer_queue.PopTopObj())
        { };
    }
};
}  // namespace v1

inline namespace v2
{

template <typename T>
class TimerManager
{
public:
    enum class DeleteModel
    {
        ANY,
        ALL
    };
    static TimerManager* GetInstance()
    {
        static TimerManager instance;
        return &instance;
    }

    void AddAlarm(std::chrono::milliseconds alarm,
        const T& key,
        const std::string& additional,
        std::function<void()> fun,
        std::chrono::milliseconds interval = std::chrono::milliseconds(0),
        bool runinmainthread               = true)
    {
        TIMETYPE::time_point timepoint = TIMETYPE::now() + alarm;
        auto element = std::make_shared<TimerElement>(TimerElement{
            timepoint, fun, key, additional, interval, runinmainthread});

        std::lock_guard<std::recursive_mutex> lck(__data_mutex);
        auto rbtreenode     = __timer_queue.AddObj(element);
        element->rbtreenode = rbtreenode;
        __key_2_element.insert({key, element});

        __cv.notify_one();
    }

    bool DeleteAlarm(const T& key,
        const std::string& additional,
        DeleteModel deletemodel = DeleteModel::ALL)
    {
        bool flag = false;
        std::lock_guard<std::recursive_mutex> lck(__data_mutex);
        auto members = __key_2_element.equal_range(key);

        for (auto member = members.first; member != members.second;)
        {
            if (member->second->additional == additional)
            {
                __timer_queue.DeleteObj(member->second->rbtreenode);
                member = __key_2_element.erase(member);
                flag   = true;
                if (deletemodel == DeleteModel::ANY)
                {
                    break;
                }
            }
            else
            {
                member++;
            }
        }

        __cv.notify_one();
        return flag;
    }

    bool DeleteAlarm(const T& key)
    {
        bool flag = false;
        std::lock_guard<std::recursive_mutex> lck(__data_mutex);
        auto members = __key_2_element.equal_range(key);
        for (auto member = members.first; member != members.second;)
        {
            __timer_queue.DeleteObj(member->second->rbtreenode);
            member = __key_2_element.erase(member);
            flag   = true;
        }
        __cv.notify_one();
        return flag;
    }

    size_t GetSize()
    {
        return __timer_queue.GetSize();
    }

    void StopTimerManager()
    {
        __stop = true;
        __cv.notify_one();
        if (__timer_thread.joinable())
        {
            __timer_thread.join();
        }
        std::lock_guard<std::recursive_mutex> lck2(__data_mutex);
        __key_2_element.clear();
        __timer_queue.Clear();
    }

    void StartTimerManager()
    {
        if (!__stop)
        {
            return;
        }
        __stop         = false;
        __timer_thread = std::thread(&TimerManager::RunTimerManager, this);
    }

private:
    using TIMETYPE = std::chrono::steady_clock;
    struct TimerElement
    {
        TIMETYPE::time_point alarm;
        std::function<void()> fun;
        T key;
        std::string additional             = "";
        std::chrono::milliseconds interval = std::chrono::milliseconds(0);
        bool run_in_main_thread            = true;
        void* rbtreenode                   = nullptr;
        bool operator<(const TimerElement& t) const
        {
            return alarm < t.alarm;
        }
    };

    std::atomic<bool> __stop = {true};
    rbtreewrap::v2::RBTreeWrap<std::shared_ptr<TimerElement>> __timer_queue;
    std::thread __timer_thread;
    std::condition_variable __cv;
    std::mutex __cv_mutex;
    std::recursive_mutex __data_mutex;
    std::multimap<T, std::shared_ptr<TimerElement>> __key_2_element;

    TimerManager()
    {
        StartTimerManager();
    }

    ~TimerManager()
    {
        StopTimerManager();
    }

    void RunTimerManager()
    {
        pthread_setname_np(pthread_self(), "ACCM-TimerManager");
        while (!__stop)
        {
            Wait();
            auto element = PopTopObjIfTimeOut();
            DealWithTimerElement(element);
        }
    }

    void Wait()
    {
        std::unique_lock<std::mutex> lck(__cv_mutex);
        bool flag;
        TIMETYPE::time_point timepoint;
        std::tie(flag, timepoint) = __timer_queue.GetTopObjKeyByFunction(
            std::function<TIMETYPE::time_point(
                const std::shared_ptr<TimerElement>&)>(
                [](const std::shared_ptr<TimerElement>& e) {
                    return e->alarm;
                }));

        if (flag)
        {
            __cv.wait_until(lck, timepoint);
        }
        else
        {
            __cv.wait_until(lck, TIMETYPE::now() + std::chrono::seconds(1));
        }
    }

    bool __DeleteAlarm(std::shared_ptr<TimerElement> element)
    {
        std::lock_guard<std::recursive_mutex> lck(__data_mutex);
        auto members = __key_2_element.equal_range(element->key);
        for (auto member = members.first; member != members.second;)
        {
            if (member->second == element)
            {
                __timer_queue.DeleteObj(element->rbtreenode);
                member = __key_2_element.erase(member);
                return true;
            }
            else
            {
                member++;
            }
        }
        __cv.notify_one();
        return false;
    }

    std::shared_ptr<TimerElement> PopTopObjIfTimeOut()
    {
        bool flag;
        std::shared_ptr<TimerElement> element;
        std::lock_guard<std::recursive_mutex> lck(__data_mutex);
        std::tie(flag, element) = __timer_queue.GetTopObjByFunction(
            [](const std::shared_ptr<TimerElement>& e) {
                return TIMETYPE::now() > e->alarm;
            });
        if (flag)
        {
            if (__DeleteAlarm(element))
            {
                return element;
            }
        }
        return nullptr;
    }

    void DealWithTimerElement(std::shared_ptr<TimerElement> element)
    {
        if (element)
        {
            if (element->fun)
            {
                if (element->run_in_main_thread)
                {
                    element->fun();
                }
                else
                {
                    std::thread tmp([fun = element->fun]() {
                        fun();
                    });
                    tmp.detach();
                }
            }
            if (element->interval > std::chrono::milliseconds(0))
            {
                AddAlarm(element->interval,
                    element->key,
                    element->additional,
                    element->fun,
                    element->interval,
                    element->run_in_main_thread);
            }
        }
    }
};
}  // namespace v2

namespace v3
{
template <typename T>
class TimerManager
{
    using TIMETYPE = std::chrono::steady_clock;

public:
    enum class DeleteModel
    {
        ANY,
        ALL
    };
    struct TimerElement
    {
        rb_node rbnode = {nullptr, nullptr, nullptr, RB_RED};
        typename std::multimap<T, TimerElement*>::iterator iter;
        TIMETYPE::time_point alarm;
        std::function<void()> fun;
        T key;
        std::string additional             = "";
        std::chrono::milliseconds interval = std::chrono::milliseconds(0);
        bool runInMainThread               = true;
        bool operator<(const TimerElement& t) const
        {
            return alarm < t.alarm;
        }
        TimerElement(TIMETYPE::time_point _alarm,
            std::function<void()>&& _fun,
            const T& _key,
            const std::string& _additional,
            std::chrono::milliseconds _interval,
            bool _runinmainthread)
            : alarm(_alarm)
            , fun(std::move(_fun))
            , key(std::move(_key))
            , additional(std::move(_additional))
            , interval(_interval)
            , runInMainThread(_runinmainthread)
        { }
        ~TimerElement()
        {
            iter = {};
            fun  = nullptr;
        }
    };

    static TimerManager* GetInstance()
    {
        static TimerManager instance;
        return &instance;
    }

    TimerElement* AddAlarm(std::chrono::milliseconds alarm,
        const T& key,
        const std::string& additional,
        std::function<void()>&& fun,
        std::chrono::milliseconds interval = std::chrono::milliseconds(0),
        bool runinmainthread               = true)
    {
        TimerElement* element = nullptr;
        if (!__stop)
        {
            element = ObjectPool<TimerElement>::GetInstance()->GetObject(
                TIMETYPE::now() + alarm,
                std::move(fun),
                std::move(key),
                std::move(additional),
                interval,
                runinmainthread);

            std::lock_guard<std::mutex> lck(__mutex);
            __timerQueue.AddObj(std::move(*element));
            element->iter = __key2element.emplace(key, element);
            __cv.notify_one();
        }
        return element;
    }

    void DeleteAlarm(const T& key,
        const std::string& additional,
        DeleteModel deletemodel = DeleteModel::ANY)
    {
        std::lock_guard<std::mutex> lck(__mutex);
        __timerQueue.ExecuteAll([&](TimerElement* element) {
            if (element->key == key && element->additional == additional)
            {
                __timerQueue.DeleteObj((rb_node*)element);
                __key2element.erase(element->iter);
                ObjectPool<TimerElement>::GetInstance()->PutObject(
                    (TimerElement*)element);
                if (deletemodel == DeleteModel::ANY)
                    return true;
            }
            return false;
        });
        __cv.notify_one();
    }

    void DeleteAlarm(const T& key)
    {
        std::lock_guard<std::mutex> lck(__mutex);
        __timerQueue.ExecuteAll([&](TimerElement* element) {
            if (element->key == key)
            {
                __timerQueue.DeleteObj((rb_node*)element);
                __key2element.erase(element->iter);
                ObjectPool<TimerElement>::GetInstance()->PutObject(
                    (TimerElement*)element);
            }
            return false;
        });
        __cv.notify_one();
    }

    void DeleteAlarm(TimerElement* element)
    {
        // makesure element is available, if not sure, please use
        // deleteAlarm(key, additional, DeleteModel::ANY)
        std::lock_guard<std::mutex> lck(__mutex);
        __timerQueue.DeleteObj((rb_node*)element);
        __key2element.erase(element->iter);
        ObjectPool<TimerElement>::GetInstance()->PutObject(
            (TimerElement*)element);
        __cv.notify_one();
    }

    void StopTimerManager()
    {
        __stop = true;
        __cv.notify_one();
        if (__timerThread.joinable())
        {
            __timerThread.join();
        }
        std::unique_lock<std::mutex> lck(__mutex);
        auto timerQueue(std::move(__timerQueue));
        lck.unlock();
        timerQueue.ExecuteAll([&](TimerElement* element) {
            timerQueue.DeleteObj((rb_node*)element);
            __key2element.erase(element->iter);
            ObjectPool<TimerElement>::GetInstance()->PutObject(
                (TimerElement*)element);
            return false;
        });
    }

    void StartTimerManager()
    {
        if (!__stop)
        {
            return;
        }
        __stop        = false;
        __timerThread = std::thread(&TimerManager::RunTimerManager, this);
    }

private:
    std::atomic<bool> __stop = {true};
    rbtreewrap::v3::RBTreeWrap<TimerElement> __timerQueue;
    std::thread __timerThread;
    std::condition_variable __cv;
    std::mutex __mutex;
    std::multimap<T, TimerElement*> __key2element;

    TimerManager()
    {
        StartTimerManager();
    }

    ~TimerManager()
    {
        StopTimerManager();
    }

    void RunTimerManager()
    {
        pthread_setname_np(pthread_self(), "ACCM-TimerManager");
        while (!__stop)
        {
            Wait();
            DealWithTimerElement(GetTimerElement());
        }
    }

    void Wait()
    {
        bool flag;
        TIMETYPE::time_point timepoint;
        std::unique_lock<std::mutex> lck(__mutex);
        std::tie(flag, timepoint) = __timerQueue.GetTopObjKeyByFunction(
            std::function<TIMETYPE::time_point(const TimerElement*)>(
                [](const TimerElement* e) -> TIMETYPE::time_point {
                    return e->alarm;
                }));

        if (flag)
        {
            __cv.wait_until(lck, timepoint);
        }
        else
        {
            __cv.wait_until(lck, TIMETYPE::now() + std::chrono::seconds(1));
        }
    }

    void DealWithTimerElement(TimerElement* element)
    {
        if (element)
        {
            if (element->fun)
            {
                if (element->runInMainThread)
                {
                    element->fun();
                    if (element->interval > std::chrono::milliseconds(0))
                    {
                        element->alarm = TIMETYPE::now() + element->interval;
                        std::lock_guard<std::mutex> lck(__mutex);
                        element->iter
                            = __key2element.emplace(element->key, element);
                        __timerQueue.AddObj(std::move(*element));
                    }
                    else
                        ObjectPool<TimerElement>::GetInstance()->PutObject(
                            element);
                }
                else
                {
                    std::thread tmp([this, element = element]() {
                        element->fun();
                        if (element->interval > std::chrono::milliseconds(0))
                        {
                            element->alarm
                                = TIMETYPE::now() + element->interval;
                            std::lock_guard<std::mutex> lck(__mutex);
                            element->iter
                                = __key2element.emplace(element->key, element);
                            __timerQueue.AddObj(std::move(*element));
                        }
                        else
                            ObjectPool<TimerElement>::GetInstance()->PutObject(
                                element);
                    });
                    tmp.detach();
                }
            }
            else
            {
                ObjectPool<TimerElement>::GetInstance()->PutObject(element);
            }
        }
    }

    TimerElement* GetTimerElement()
    {
        bool flag;
        TimerElement* element;
        std::lock_guard<std::mutex> lck(__mutex);
        std::tie(flag, element) = __timerQueue.GetTopObjByFunctionAndDelete(
            [](const TimerElement* e) {
                return TIMETYPE::now() > e->alarm;
            });
        if (flag)
        {
            __key2element.erase(element->iter);
            return element;
        }
        else
            return nullptr;
    }

    void AddAlarmNoLock(std::chrono::milliseconds alarm,
        const T& key,
        const std::string& additional,
        std::function<void()>&& fun,
        std::chrono::milliseconds interval = std::chrono::milliseconds(0),
        bool runinmainthread               = true)
    {
        if (!__stop)
        {
            auto element = ObjectPool<TimerElement>::GetInstance()->GetObject();
            element->alarm           = TIMETYPE::now() + alarm;
            element->fun             = std::move(fun);
            element->key             = std::move(key);
            element->additional      = std::move(additional);
            element->interval        = interval;
            element->runInMainThread = runinmainthread;

            __timerQueue.addObj(std::move(*element));
            __cv.notify_one();
        }
    }
};
}  // namespace v3

}  // namespace timermanager