#pragma once
#include "tools/objectpool.hpp"

template <int BUF_SIZE>
class RudpAllocate
{
public:
    RudpAllocate()                               = default;
    RudpAllocate(const RudpAllocate&)            = delete;
    RudpAllocate& operator=(const RudpAllocate&) = delete;
    RudpAllocate(RudpAllocate&&)                 = delete;
    RudpAllocate& operator=(RudpAllocate&&)      = delete;
    ~RudpAllocate()                              = default;

    char* GetBuf()
    {
        auto aaaq = v2::ObjectPool<RudpAllocateTag>::GetInstance()->GetObject();
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

private:
    struct RudpAllocateTag
    {
        char __buf[BUF_SIZE];
    };
};