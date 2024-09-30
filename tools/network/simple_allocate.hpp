#pragma once
#include "network_global.hpp"

template <int BUF_SIZE = MAX_BUF_SIZE>
class SimpleAllocate
{
public:
    SimpleAllocate()                                 = default;
    SimpleAllocate(const SimpleAllocate&)            = delete;
    SimpleAllocate& operator=(const SimpleAllocate&) = delete;
    SimpleAllocate(SimpleAllocate&&)                 = delete;
    SimpleAllocate& operator=(SimpleAllocate&&)      = delete;
    ~SimpleAllocate()                                = default;

    char* GetBuf()
    {
        return __buf;
    }

    int GetBufSize()
    {
        return BUF_SIZE;
    }

private:
    char __buf[BUF_SIZE];
};