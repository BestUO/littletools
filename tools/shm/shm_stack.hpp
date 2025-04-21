#pragma once

#include "tools/shm/global.hpp"

template <QueueElement T>
class SHMStack
{
public:
    int16_t Pop(T* data)
    {
        if (__head == -1)
            return -1;
        else
        {
            auto old_head         = __head;
            __head                = data[old_head].__next;
            data[old_head].__next = -1;

            return old_head;
        }
    }

    void Push(T* data, int32_t index)
    {
        if (__head == -1)
        {
            __head = index;
        }
        else
        {
            data[index].__next = __head;
            __head             = index;
        }
    }

    bool IsEmpty()
    {
        return __head == -1;
    }

private:
    int32_t __head = -1;
};