#pragma once

#include <chrono>
#include <thread>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <map>
#include "../queue/threadsafecontainer.hpp"
#include "rbtree/RBTreeWrap.hpp"

namespace timermanager
{
inline namespace V1
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

    void AddAlarm(std::chrono::system_clock::time_point alarm,
        T key,
        std::function<void()> fun,
        std::chrono::seconds interval = std::chrono::seconds(0))
    {
        __timerqueue.AddObj({alarm, fun, key, interval});
        __cv.notify_one();
    }

    bool DeleteAlarm(std::function<bool(T)> comparefun)
    {
        bool result = __timerqueue.DeleteObj([comparefun](const auto& obj) {
            return comparefun(obj.key);
        });
        __cv.notify_one();
        return result;
    }

    bool DeleteAlarm(T key)
    {
        bool result = __timerqueue.DeleteObj([key](const auto& obj) {
            return key == obj.key;
        });
        __cv.notify_one();
        return result;
    }

    void StopTimerManager()
    {
        __stop = true;
        __cv.notify_one();
    }

    void SetModel(int model)
    {
        __model = model;
    }

private:
    struct TimerElement
    {
        std::chrono::system_clock::time_point alarm;
        std::function<void()> fun;
        T key;
        std::chrono::seconds interval = std::chrono::seconds(0);
        bool operator<(const TimerElement& t) const
        {
            return alarm > t.alarm;
        }
    };
    bool __stop = false;
    ThreadSafePriorityQueue<TimerElement> __timerqueue;
    std::thread __timerthread;
    std::condition_variable __cv;
    std::mutex __cv_m;
    int __model = 1;

    TimerManager()
    {
        __timerthread = std::thread(&TimerManager::RunTimerManager, this);
    }

    ~TimerManager()
    {
        __timerthread.join();
    }

    void RunTimerManager()
    {
        while (!__stop)
        {
            std::unique_lock<std::mutex> lck(__cv_m);
            auto timerelement = __timerqueue.GetTopObj();
            if (timerelement != std::nullopt)
                __cv.wait_until(lck, timerelement->alarm);
            else
                __cv.wait_until(lck,
                    std::chrono::system_clock::now() + std::chrono::hours(1));
            if (timerelement
                && std::chrono::system_clock::now() >= timerelement->alarm)
            {
                if (timerelement->interval > std::chrono::seconds(0))
                    AddAlarm(timerelement->alarm + timerelement->interval,
                        timerelement->key,
                        timerelement->fun,
                        timerelement->interval);
                __timerqueue.PopTop();
                if (timerelement->fun)
                {
                    if (__model)
                        timerelement->fun();
                    else
                    {
                        std::thread tmp(
                            [](std::function<void()> fun) {
                                fun();
                            },
                            timerelement->fun);
                        tmp.detach();
                    }
                }
            }
        }
    }
};
}  // namespace V1

namespace V2
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
    static TimerManager* getInstance()
    {
        static TimerManager instance;
        return &instance;
    }

    void addAlarm(std::chrono::milliseconds alarm,
        const T& key,
        const std::string& additional,
        std::function<void()> fun,
        std::chrono::milliseconds interval = std::chrono::milliseconds(0),
        bool runinmainthread               = true)
    {
        TIMETYPE::time_point timepoint = TIMETYPE::now() + alarm;
        auto element = std::make_shared<TimerElement>(TimerElement{
            timepoint, fun, key, additional, interval, runinmainthread});

        std::lock_guard<std::recursive_mutex> lck(m_datamutex);
        auto rbtreenode     = m_timerQueue.AddObj(element);
        element->rbtreenode = rbtreenode;
        m_key2element.insert({key, element});

        m_cv.notify_one();
    }

    bool deleteAlarm(const T& key,
        const std::string& additional,
        DeleteModel deletemodel = DeleteModel::ALL)
    {
        bool flag = false;
        std::lock_guard<std::recursive_mutex> lck(m_datamutex);
        auto members = m_key2element.equal_range(key);

        for (auto member = members.first; member != members.second;)
        {
            if (member->second->additional == additional)
            {
                m_timerQueue.DeleteObj(member->second->rbtreenode);
                member = m_key2element.erase(member);
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

        m_cv.notify_one();
        return flag;
    }

    bool deleteAlarm(const T& key)
    {
        bool flag = false;
        std::lock_guard<std::recursive_mutex> lck(m_datamutex);
        auto members = m_key2element.equal_range(key);
        for (auto member = members.first; member != members.second;)
        {
            m_timerQueue.DeleteObj(member->second->rbtreenode);
            member = m_key2element.erase(member);
            flag   = true;
        }
        m_cv.notify_one();
        return flag;
    }

    size_t getSize()
    {
        return m_timerQueue.getSize();
    }

    void stopTimerManager()
    {
        m_stop = true;
        m_cv.notify_one();
        if (m_timerThread.joinable())
        {
            m_timerThread.join();
        }
        std::lock_guard<std::recursive_mutex> lck2(m_datamutex);
        m_key2element.clear();
        m_timerQueue.clear();
    }

    void startTimerManager()
    {
        if (!m_stop)
        {
            return;
        }
        m_stop        = false;
        m_timerThread = std::thread(&TimerManager::runTimerManager, this);
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
        bool runInMainThread               = true;
        void* rbtreenode                   = nullptr;
        bool operator<(const TimerElement& t) const
        {
            return alarm < t.alarm;
        }
    };

    std::atomic<bool> m_stop = {true};
    rbtreewrap::v2::RBTreeWrap<std::shared_ptr<TimerElement>> m_timerQueue;
    std::thread m_timerThread;
    std::condition_variable m_cv;
    std::mutex m_cvmutex;
    std::recursive_mutex m_datamutex;
    std::multimap<T, std::shared_ptr<TimerElement>> m_key2element;

    TimerManager()
    {
        startTimerManager();
    }

    ~TimerManager()
    {
        stopTimerManager();
    }

    void runTimerManager()
    {
        pthread_setname_np(pthread_self(), "ACCM-TimerManager");
        while (!m_stop)
        {
            wait();
            auto element = popTopObjIfTimeOut();
            dealWithTimerElement(element);
        }
    }

    void wait()
    {
        std::unique_lock<std::mutex> lck(m_cvmutex);
        bool flag;
        TIMETYPE::time_point timepoint;
        std::tie(flag, timepoint) = m_timerQueue.GetTopObjKeyByFunction(
            std::function<TIMETYPE::time_point(
                const std::shared_ptr<TimerElement>&)>(
                [](const std::shared_ptr<TimerElement>& e) {
                    return e->alarm;
                }));

        if (flag)
        {
            m_cv.wait_until(lck, timepoint);
        }
        else
        {
            m_cv.wait_until(lck, TIMETYPE::now() + std::chrono::seconds(1));
        }
    }

    std::shared_ptr<TimerElement> popTopObjIfTimeOut()
    {
        bool flag;
        std::shared_ptr<TimerElement> element;
        std::lock_guard<std::recursive_mutex> lck(m_datamutex);
        std::tie(flag, element) = m_timerQueue.GetTopObjByFunction(
            [](const std::shared_ptr<TimerElement>& e) {
                return TIMETYPE::now() > e->alarm;
            });
        if (flag)
        {
            if (deleteAlarm(
                    element->key, element->additional, DeleteModel::ANY))
            {
                return element;
            }
        }
        return nullptr;
    }

    void dealWithTimerElement(std::shared_ptr<TimerElement> element)
    {
        if (element)
        {
            if (element->fun)
            {
                if (element->runInMainThread)
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
                addAlarm(element->interval,
                    element->key,
                    element->additional,
                    element->fun,
                    element->interval,
                    element->runInMainThread);
            }
        }
    }
};

}  // namespace V2

}  // namespace timermanager