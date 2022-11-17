#pragma once

#include <chrono>
#include <thread>
#include "../queue/threadsafecontainer.hpp"
#include <mutex>

template <typename T>
class TimerManager
{
public:
    static TimerManager* GetInstance()
    {
        static TimerManager instance;
        return &instance;
    }

    void AddAlarm(std::chrono::system_clock::time_point alarm, T key, std::function<void()> fun, std::chrono::seconds interval = std::chrono::seconds(0))
    {
        __timerqueue.AddObj({alarm, fun, key, interval});
        __cv.notify_one();
    }

    bool DeleteAlarm(std::function<bool(T)> comparefun)
    {
        bool result = __timerqueue.DeleteObj([comparefun](const auto& obj){return comparefun(obj.key);});
        __cv.notify_one();
        return result;
    }

    bool DeleteAlarm(T key)
    {
        bool result = __timerqueue.DeleteObj([key](const auto& obj){return key == obj.key;});
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
        bool operator<(const TimerElement &t) const
        {
            return alarm > t.alarm;
        }
    };
    bool __stop=false;
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
        while(!__stop)
        {
            std::unique_lock<std::mutex> lck(__cv_m);
            auto timerelement = __timerqueue.GetTopObj();
            if(timerelement != std::nullopt)
                __cv.wait_until(lck, timerelement->alarm);
            else
                __cv.wait_until(lck,  std::chrono::system_clock::now() + std::chrono::hours(1));
            if(timerelement && std::chrono::system_clock::now() >= timerelement->alarm)
            {
                if(timerelement->interval > std::chrono::seconds(0))
                    AddAlarm(timerelement->alarm+timerelement->interval, timerelement->key, timerelement->fun, timerelement->interval);
                __timerqueue.PopTop();
                if(timerelement->fun)
                {
                    if(__model)
                        timerelement->fun();
                    else
                    {
                        std::thread tmp([](std::function<void()> fun)
                        {
                            fun();
                        },timerelement->fun);
                        tmp.detach();
                    }
                }
            }
        }
    }
};