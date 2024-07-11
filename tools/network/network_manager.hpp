#pragma once

#include <thread>
#include "tools/cpu_bind.hpp"

namespace network
{
template <typename T>
class NetWorkManager : public T
{
public:
    NetWorkManager() = default;
    ~NetWorkManager()
    {
        if (__net_thread.joinable())
            __net_thread.join();
    }

    void Start()
    {
        if (__stop)
        {
            __stop       = false;
            __net_thread = std::thread([this]() {
                this->Run();
            });
        }
    }

    void Stop()
    {
        if (!__stop)
        {
            __stop = true;
            this->Continue();
            __net_thread.join();
        }
    }

    void Run()
    {
        pthread_setname_np(pthread_self(), "NetWork");
        CPUBind::BindCPU();
        while (!__stop)
        {
            this->NetWorkRunOnce();
        }
    }

private:
    bool __stop = true;
    std::thread __net_thread;
};
}  // namespace network