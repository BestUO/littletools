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

    void AddAlarm(std::chrono::system_clock::time_point alarm, T t, std::function<void()> fun)
    {
        __timerqueue.AddObj({alarm, fun, t});
        __cv.notify_one();
    }

    void DeleteAlarm(std::function<bool(T)> comparefun)
    {
        __timerqueue.DeleteObj([comparefun](const auto& obj){return comparefun(obj.key);});
        __cv.notify_one();
    }

    void StopTimerManager()
    {
        __stop = true;
    }
private:
    TimerManager()
    {
        __timerthread = std::thread(&TimerManager::RunTimerManager, this);
    }

    ~TimerManager()
    {
        __timerthread.join();
    }

    bool __stop=false;
    struct TimerElement
    {
        std::chrono::system_clock::time_point alarm;
        std::function<void()> fun;
        T key;
        bool operator<(const TimerElement &t) const
        {
            return alarm > t.alarm;
        }
    };
    TimerElement __timerelement;
    ThreadSafePriorityQueue<TimerElement> __timerqueue;
    std::thread __timerthread;
    std::condition_variable __cv;
    std::mutex __cv_m;

    void RunTimerManager()
    {
        while(!__stop)
        {
            // std::this_thread::sleep_for(std::chrono::seconds(1));
            std::unique_lock<std::mutex> lck(__cv_m);
            if(__timerqueue.GetTopObj(__timerelement))
                __cv.wait_until(lck, __timerelement.alarm);
            else
                __cv.wait_until(lck,  std::chrono::system_clock::now() + std::chrono::hours(1));

            while(bool c = __timerqueue.GetObj(__timerelement, [](const TimerElement &obj) -> bool
            {
                return  std::chrono::system_clock::now() >= obj.alarm ? true:false;
            }))
            {
                std::thread tmp([](TimerElement timerelement)
                {
                    timerelement.fun();
                },__timerelement);
                tmp.detach();
            }
        }
    }
};