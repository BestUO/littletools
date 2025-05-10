#pragma once

#include "tools/shm/shm_global.hpp"

template <DeQueElement T>
class SHMDeque
{
public:
    int32_t GetHead()
    {
        return __head;
    }

    int32_t Pop(T* data)
    {
        if (__head == -1)
            return -1;
        else
        {
            auto old_head = __head;
            __head        = data[old_head].__next;

            data[old_head].__next = -1;
            data[old_head].__prev = -1;

            if (__head == -1)
                __tail = -1;

            return old_head;
        }
    }

    int32_t Pop(T* data, int32_t index)
    {
        if (index == __head)
            return Pop(data);
        else
        {
            auto prev = data[index].__prev;
            auto next = data[index].__next;

            if (prev != -1)
                data[prev].__next = next;
            if (next != -1)
                data[next].__prev = prev;

            if (index == __tail)
                __tail = prev;

            return index;
        }
    }

    void Push(T* data, int32_t index)
    {
        if (__tail == -1)
        {
            __head = __tail = index;
        }
        else
        {
            data[index].__prev  = __tail;
            data[index].__next  = -1;
            data[__tail].__next = index;
            __tail              = index;
        }
    }

    // template <typename R, typename... Args>
    // auto ExecuteAll(T* data, R& r, std::function<R(Args...)> func)
    // {
    //     for (int32_t i = __head; i != -1; i = data[i].__next)
    //     {
    //         func(r, i);
    //     }
    //     return r;
    // }

    template <typename R, typename F>
    auto ExecuteAll(T* data, R& r, F func)
    {
        for (int32_t i = __head; i != -1; i = data[i].__next)
        {
            func(r, i);
        }
        return r;
    }

    bool IsEmpty()
    {
        return __head == -1;
    }

private:
    int32_t __head = -1;
    int32_t __tail = -1;
};