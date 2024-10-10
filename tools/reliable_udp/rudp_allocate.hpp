#pragma once
#include <netinet/in.h>
#include "tools/objectpool.hpp"

template <int BUF_SIZE>
class RudpAllocate
{
public:
    struct RudpAllocateTag
    {
        char __buf[BUF_SIZE];
        sockaddr addr;
    };

    RudpAllocate()                               = default;
    RudpAllocate(const RudpAllocate&)            = delete;
    RudpAllocate& operator=(const RudpAllocate&) = delete;
    RudpAllocate(RudpAllocate&&)                 = delete;
    RudpAllocate& operator=(RudpAllocate&&)      = delete;
    ~RudpAllocate()                              = default;

    char* GetBuf()
    {
        return (char*)v2::ObjectPool<RudpAllocateTag>::GetInstance()
            ->GetObject();
    }

    void FreeBuf(char* buf)
    {
        v2::ObjectPool<RudpAllocateTag>::GetInstance()->PutObject(
            (RudpAllocateTag*)buf);
    }

    int GetBufSize()
    {
        return BUF_SIZE;
    }
};