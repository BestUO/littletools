#pragma once

#include <stddef.h>
#include <cstdint>
#include <functional>
#include <thread>
#include <unistd.h>
#include "tools/shm/shm_msgqueue.hpp"
#include "tools/shm/shm_global.hpp"
#include "tools/shm/shm_factory.hpp"

template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
class SHMMsgQueueManager
{
public:
    static SHMMsgQueueManager* GetInstance(std::string_view name)
    {
        constexpr auto type_name = TypeInfo<T>::GetTypeName();
        static SHMMsgQueueManager instance(name.data());
        return &instance;
    }

    std::tuple<int32_t, T*> Allocate()
    {
        return __queue->Allocate();
    }

    void Free(int32_t msg_index)
    {
        __queue->Free(msg_index, __reader_index);
    }

    void GabageCollect()
    {
        __queue->GabageCollect();
    }

    Result SendMsgToAny(int32_t index)
    {
        return __queue->SendMsgToAny(index);
    }

    Result SendMsgToAll(int32_t index)
    {
        return __queue->SendMsgToAll(index);
    }

    Result SendMsgToAllExceptReader(int32_t index, int32_t reader_index)
    {
        return __queue->SendMsgToAllExceptReader(index, reader_index);
    }

    void Attach(std::function<void(int32_t, T*)> callback)
    {
        if (__reader_index != -1)
        {
            return;
        }
        else
        {
            __reader_index = __queue->Attach();
            __callback     = callback;
        }
    }

    void Detach()
    {
        __queue->Detach(__reader_index);
        __reader_index = -1;
        __callback     = nullptr;
    }

    int32_t RecvTopMsgOnce()
    {
        auto [index, s_ptr] = RecvTopMsgWithReader(__reader_index);
        if (index != -1)
        {
            __callback(index, s_ptr);
            return index;
        }
        return -1;
    }

    void Start()
    {
        if (__reader_index == -1 || __callback == nullptr)
        {
            return;
        }
        __is_running = true;
        while (__is_running)
        {
            if (int32_t index = RecvTopMsgOnce(); index != -1)
            {
                Free(index, __reader_index);
            }
            else
            {
                std::this_thread::yield();
            }
        }
    }

    void Stop()
    {
        __is_running = false;
    }

private:
    std::atomic<bool> __is_running = false;
    SHMFactory<SHMMsgQueue<T, N, M>> __queue;
    int32_t __reader_index                      = -1;
    std::function<void(int32_t, T*)> __callback = nullptr;

    std::tuple<int32_t, T*> RecvTopMsgWithReader(uint32_t reader_index)
    {
        return __queue->RecvTopMsgWithReader(reader_index);
    }

    SHMMsgQueueManager(std::string_view name)
        : __queue(name.data())
    { }

    void Free(int32_t msg_index, int32_t reader_index)
    {
        __queue->Free(msg_index, reader_index);
    }
    ~SHMMsgQueueManager()                                    = default;
    SHMMsgQueueManager(const SHMMsgQueueManager&)            = delete;
    SHMMsgQueueManager& operator=(const SHMMsgQueueManager&) = delete;
    SHMMsgQueueManager(SHMMsgQueueManager&&)                 = delete;
    SHMMsgQueueManager& operator=(SHMMsgQueueManager&&)      = delete;
};