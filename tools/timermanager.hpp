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
        __timer_queue.DeleteObj(element->rbtreenode);
        auto members = __key_2_element.equal_range(element->key);
        for (auto member = members.first; member != members.second;)
        {
            if (member->second == element)
            {
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
        __timer_queue.DeleteObj(element->rbtreenode);
        auto members = __key_2_element.equal_range(element->key);
        for (auto member = members.first; member != members.second;)
        {
            if (member->second == element)
            {
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
}  // namespace v3

}  // namespace timermanager