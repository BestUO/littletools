#pragma once

#include <chrono>
#include <thread>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <atomic>

#include "RBTreeWrap.hpp"
namespace Boray
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
        std::chrono::milliseconds interval = std::chrono::milliseconds(0),
        bool runinmainthread               = true)
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
        if (__timerthread.joinable())
            __timerthread.join();
        while (__timerqueue.PopTopObj())
        { };
    }

    void StartTimerManager()
    {
        if (!__stop)
            return;
        __stop        = false;
        __timerthread = std::thread(&TimerManager::RunTimerManager, this);
    }

private:
    struct TimerElement
    {
        rb_node rbnode = {nullptr, nullptr, nullptr, RB_RED};
        std::chrono::system_clock::time_point alarm;
        std::function<void()> fun;
        T key;
        std::chrono::milliseconds interval = std::chrono::seconds(0);
        bool runinmainthread               = true;
        bool operator<(const TimerElement& t) const
        {
            return alarm < t.alarm;
        }
    };

    std::atomic<bool> __stop = {true};
    RBTreeWrap<TimerElement> __timerqueue;
    std::thread __timerthread;
    std::condition_variable __cv;
    std::mutex __cv_m;

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
        while (!__stop)
        {
            auto timepointopt = __timerqueue.GetTopObjKey(
                std::function<std::chrono::system_clock::time_point(
                    TimerElement*)>([](TimerElement* element) {
                    return element->alarm;
                }));

            std::unique_lock<std::mutex> lck(__cv_m);
            if (timepointopt.has_value())
                __cv.wait_until(lck, timepointopt.value());
            else
                __cv.wait_until(lck,
                    std::chrono::system_clock::now() + std::chrono::seconds(1));

            auto elementopt
                = __timerqueue.GetTopObjAndDelete([](TimerElement* e) {
                      return std::chrono::system_clock::now() > e->alarm;
                  });
            if (elementopt.has_value())
            {
                auto element = elementopt.value();
                if (element.interval > std::chrono::milliseconds(0))
                    AddAlarm(element.alarm + element.interval,
                        element.key,
                        element.fun,
                        element.interval,
                        element.runinmainthread);
                if (element.fun)
                {
                    if (element.runinmainthread)
                        element.fun();
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
    }
};

}  // namespace Boray