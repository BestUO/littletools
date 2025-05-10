#pragma once

#include <pthread.h>
#include <mutex>
#include "tools/shm/shm_global.hpp"
#include "tools/shm/shm_mutex.hpp"
#include "tools/shm/shm_stack.hpp"
#include "tools/shm/shm_global.hpp"

template <PodType T, size_t N = SHM_QUEUE_SIZE>
class SHMMemoryPool
{
public:
    SHMMemoryPool()
    {
        for (size_t i = 0; i < N; i++)
            __shm_stack.Push(__data, i);
    }

    std::tuple<IndexDataTypeCheck<T>, T*> Allocate()
    {
        std::lock_guard<SHMMutex> lck(__mutex);
        auto index = __shm_stack.Pop(__data);
        if (index == -1)
            return {{index}, nullptr};
        else
            return {{index}, __data[index].__t.GetData()};
    }

    void Free(IndexDataTypeCheck<T> index)
    {
        std::lock_guard<SHMMutex> lck(__mutex);
        __shm_stack.Push(__data, index.__index);
    }

private:
    struct SHMMempoolElement
    {
        int32_t __next = -1;
        Element<T> __t;
    };
    SHMMutex __mutex;
    SHMStack<SHMMempoolElement> __shm_stack;
    SHMMempoolElement __data[N];
};