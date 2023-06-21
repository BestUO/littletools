#pragma once

#include <linux/limits.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <functional>
#include <condition_variable>

#include "RBTreeWrap.hpp"
namespace Boray {

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
        std::chrono::seconds interval = std::chrono::seconds(0),
        bool runinmainthread          = true)
    {
        __timerqueue.AddObj({{}, alarm, fun, key, interval, runinmainthread});
        __cv.notify_one();
    }

    void DeleteAlarm(const T& key)
    {
        __timerqueue.DeleteObj([key = key](const TimerElement& timerelement) {
            return timerelement.key == key;
        });
        __cv.notify_one();
    }

    void StopTimerManager()
    {
        __stop = true;
        __cv.notify_one();
    }

private:
    struct TimerElement
    {
        rb_node rbnode = {nullptr, nullptr, nullptr, RB_RED};
        std::chrono::system_clock::time_point alarm;
        std::function<void()> fun;
        T key;
        std::chrono::seconds interval = std::chrono::seconds(0);
        bool runinmainthread          = true;
        bool operator<(const TimerElement& t) const
        {
            return alarm < t.alarm;
        }
    };

    bool __stop = false;
    RBTreeWrap<TimerElement> __timerqueue;
    std::thread __timerthread;
    std::condition_variable __cv;
    std::mutex __cv_m;

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
            auto timerelementptr = __timerqueue.GetTopObj();
            if (timerelementptr != nullptr)
                __cv.wait_until(lck, timerelementptr->alarm);
            else
                __cv.wait_until(lck,
                    std::chrono::system_clock::now() + std::chrono::hours(1));
            if (timerelementptr
                && std::chrono::system_clock::now() >= timerelementptr->alarm)
            {
                TimerElement timerelement = *timerelementptr;
                if (timerelement.interval > std::chrono::seconds(0))
                    AddAlarm(timerelement.alarm + timerelement.interval,
                        timerelement.key,
                        timerelement.fun,
                        timerelement.interval);
                __timerqueue.PopTopObj();
                if (timerelement.fun)
                {
                    if (timerelement.runinmainthread)
                        timerelement.fun();
                    else
                    {
                        std::thread tmp(
                            [timerelement = std::move(timerelement)]() {
                                timerelement.fun();
                            });
                        tmp.detach();
                    }
                }
            }
        }
    }
};
    
} // namespace Boray