#pragma once

#include <cstdint>
#include <string_view>
#include <map>
#include <mutex>
#include "tools/shm/shm_global.hpp"
#include "tools/shm/shm_util.hpp"
#include "tools/shm/shm_msgqueue_manager.hpp"

class SHMMsgQueueManagerGroup
{
public:
    static SHMMsgQueueManagerGroup* GetInstance()
    {
        static SHMMsgQueueManagerGroup instance;
        return &instance;
    }

    template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
    std::tuple<int32_t, T*> Allocate(std::string_view name)
    {
        auto __queue = SHMMsgQueueManager<T, N, M>::GetInstance(name);
        return __queue->Allocate();
    }

    // template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
    // void Free(std::string_view name, int32_t msg_index)
    // {
    //     auto __queue = SHMMsgQueueManager<T, N, M>::GetInstance(name);
    //     __queue->Free(msg_index);
    // }

    template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
    void GabageCollect(std::string_view name)
    {
        auto __queue = SHMMsgQueueManager<T, N, M>::GetInstance(name);
        __queue->GabageCollect();
    }

    template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
    Result SendMsgToAny(std::string_view name, int32_t index)
    {
        auto __queue = SHMMsgQueueManager<T, N, M>::GetInstance(name);
        return __queue->SendMsgToAny(index);
    }

    template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
    Result SendMsgToAll(std::string_view name, int32_t index)
    {
        auto __queue = SHMMsgQueueManager<T, N, M>::GetInstance(name);
        return __queue->SendMsgToAll(index);
    }

    template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
    Result SendMsgToAllExceptReader(std::string_view name,
        int32_t index,
        int32_t reader_index)
    {
        auto __queue = SHMMsgQueueManager<T, N, M>::GetInstance(name);
        return __queue->SendMsgToAllExceptReader(index, reader_index);
    }

    template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
    void Attach(std::string_view name,
        std::function<void(int32_t, T*)> callback)
    {
        std::lock_guard<std::mutex> lock(__mutex);
        auto hash = std::hash<std::string_view>{}(
            std::string(TypeInfo<T>::GetTypeName()) + std::string(name));
        if (__recv_fun_map.find(hash) != __recv_fun_map.end())
        {
            return;
        }
        else
        {
            auto __queue = SHMMsgQueueManager<T, N, M>::GetInstance(name);
            __queue->Attach(callback);

            __recv_fun_map[hash] = [name = std::string(name)]() {
                return SHMMsgQueueManager<T, N, M>::GetInstance(name)
                    ->RecvTopMsgWithReader();
            };
            __callback_map[hash] = [name = std::string(name), callback](
                                       int32_t index, void* s_ptr) {
                callback(index, static_cast<T*>(s_ptr));
                SHMMsgQueueManager<T, N, M>::GetInstance(name)->Free(index);
            };
        }
    }

    template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
    void Detach(std::string_view name)
    {
        std::lock_guard<std::mutex> lock(__mutex);
        auto hash
            = std::hash<std::string_view>{}(TypeInfo<T>::GetTypeName() + name);
        if (__recv_fun_map.find(hash) != __recv_fun_map.end())
        {
            return;
        }
        else
        {
            auto __queue = SHMMsgQueueManager<T, N, M>::GetInstance(name);
            __queue->Detach();
            __recv_fun_map.erase(hash);
            __callback_map.erase(hash);
        }
    }

    void Start()
    {
        if (__is_running)
        {
            return;
        }
        __is_running = true;
        while (__is_running)
        {
            bool any_callback = false;
            {
                std::lock_guard<std::mutex> lock(__mutex);
                for (auto& [hash, recv_fun] : __recv_fun_map)
                {
                    auto [index, s_ptr] = recv_fun();
                    if (index != -1)
                    {
                        __callback_map[hash](
                            index, s_ptr);  // can add to work pool
                        any_callback = true;
                    }

                    if (any_callback)
                    {
                        std::this_thread::yield();
                    }
                }
            }

            if (!any_callback)
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
    SHMMsgQueueManagerGroup()                               = default;
    ~SHMMsgQueueManagerGroup()                              = default;
    SHMMsgQueueManagerGroup(const SHMMsgQueueManagerGroup&) = delete;
    SHMMsgQueueManagerGroup& operator=(const SHMMsgQueueManagerGroup&) = delete;
    SHMMsgQueueManagerGroup(SHMMsgQueueManagerGroup&&)                 = delete;
    SHMMsgQueueManagerGroup& operator=(SHMMsgQueueManagerGroup&&)      = delete;

    std::map<size_t, std::function<std::tuple<int32_t, void*>()>>
        __recv_fun_map;
    std::map<size_t, std::function<void(int32_t, void*)>> __callback_map;
    std::mutex __mutex;
    std::atomic<bool> __is_running = false;
};