#pragma once

#include <future>
#include "../queue/threadsafecontainer.hpp"

class DealCommandCenter:public ThreadSafeQueue<std::function<void()>>
{
public:
    static DealCommandCenter* GetInstance()
    {
        static DealCommandCenter instance;
        return &instance;
    }

    template<typename K, typename T, typename... Args>
    void AsyncDealCommandAndGetResult(K &value, T fun, Args ...args)
    {
        using ret_t = decltype(fun(args...));
        std::promise<ret_t> p;
        std::future<ret_t> f = p.get_future();
        ThreadSafeQueue<std::function<void()>>::AddObj(std::bind([&](){p.set_value(fun(args...));}));
        value = f.get();
    }

    template<typename T, typename... Args>
    void AsyncDealCommand(T fun, Args ...args)
    {
        ThreadSafeQueue<std::function<void()>>::AddObj(std::bind([fun, args...](){fun(args...);}));
    }

    void StopCommandCenter()
    {
        __stop = true;
    }
private:
    std::once_flag __oc;
    DealCommandCenter():ThreadSafeQueue<std::function<void()>>(20480)
    {
        std::call_once(__oc, &DealCommandCenter::RunCommandCenterThreads, this, 1);
    }

    ~DealCommandCenter()
    {
        for(auto &thread:__commandcenterthreads)
            thread.join();
    }

    void RunCommandCenterThreads(int n)
    {
        for(int i = 0;i < n; i++)
            __commandcenterthreads.emplace_back(std::thread(&DealCommandCenter::RunCommandCenter, this));
    }

    bool __stop=false;
    std::vector<std::thread> __commandcenterthreads;
    // ThreadSafeQueue<std::function<void()>> __niubi;

    void RunCommandCenter()
    {
        std::function<void()> funobj;
        while(!__stop)
        {    
            while(ThreadSafeQueue<std::function<void()>>::GetObj(funobj))
                funobj();
            // std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
};