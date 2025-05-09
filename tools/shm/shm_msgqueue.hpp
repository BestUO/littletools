#pragma once

#include <stddef.h>
#include <cstdint>
#include <mutex>
#include <unistd.h>
#include <vector>
#include "tools/shm/global.hpp"
#include "tools/shm/shm_mutex.hpp"
#include "tools/shm/shm_deque.hpp"
#include "tools/shm/bitset.hpp"

template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
class SHMMsgQueue
{
public:
    SHMMsgQueue()
    {
        for (size_t i = 0; i < N; i++)
        {
            __data[i].__index = i;
            __shm_msg_pool.Push(__data, i);
        }
    }

    std::tuple<int32_t, T*> Allocate()
    {
        std::lock_guard<SHMMutex> lck(__mutex);
        auto index = __shm_msg_pool.Pop(__data);
        if (index == -1)
        {
            return {index, nullptr};
        }
        else
        {
            Bitset<M>::ClearAllBit(__data[index].__reader);
            return {index, __data[index].__t.GetData()};
        }
    }

    Result SendMsgToAny(int32_t index)
    {
        std::lock_guard<SHMMutex> lck(__mutex);
        if (__reader_bitset.IsEmpty())
        {
            return Result::NOREADER;
        }
        else
        {
            __shm_msgs_queue.Push(__data, index);
            return Result::OK;
        }
    }

    Result SendMsgToAll(int32_t index)
    {
        std::lock_guard<SHMMutex> lck(__mutex);
        __reader_bitset.BitsetCopy(__data[index].__reader);
        if (Bitset<M>::IsEmpty(__data[index].__reader))
        {
            return Result::NOREADER;
        }
        else
        {
            __shm_msgs_queue.Push(__data, index);
            return Result::OK;
        }
    }

    Result SendMsgToAllExceptReader(int32_t index, int32_t reader_index)
    {
        std::lock_guard<SHMMutex> lck(__mutex);
        __reader_bitset.BitsetCopy(__data[index].__reader);
        Bitset<M>::ClearBit(__data[index].__reader, reader_index);
        if (Bitset<M>::IsEmpty(__data[index].__reader))
        {
            return Result::NOREADER;
        }
        else
        {
            __shm_msgs_queue.Push(__data, index);
            return Result::OK;
        }
    }

    std::tuple<int32_t, T*> RecvTopMsgWithReader(uint32_t reader_index)
    {
        std::lock_guard<SHMMutex> lck(__mutex);
        auto msg_index = __shm_msgs_queue.GetHead();

        if (msg_index == -1 || reader_index >= M)
        {
            return {-1, nullptr};
        }
        else if (Bitset<M>::IsEmpty(
                     __data[msg_index].__reader))  // from send any
        {
            Bitset<M>::SetBit(__data[msg_index].__reader, reader_index);
            return {msg_index, __data[msg_index].__t.GetData()};
        }
        else if (!Bitset<M>::GetBit(__data[msg_index].__reader, reader_index))
        {
            return {-1, nullptr};
        }
        else
        {
            return {msg_index, __data[msg_index].__t.GetData()};
        }
    }

    void Free(int32_t msg_index, int32_t reader_index)
    {
        std::lock_guard<SHMMutex> lck(__mutex);
        Bitset<M>::ClearBit(__data[msg_index].__reader, reader_index);
        if (Bitset<M>::IsEmpty(__data[msg_index].__reader))
        {
            __shm_msgs_queue.Pop(__data, msg_index);
            __shm_msg_pool.Push(__data, msg_index);
        }
    }

    int32_t Attach()
    {
        std::lock_guard<SHMMutex> lck(__mutex);
        auto reader_index = __reader_bitset.GetZeroIndex();
        if (reader_index > -1)
        {
            auto pid                           = getpid();
            __reader_infos[reader_index].__pid = pid;
            __reader_bitset.SetBit(reader_index);
        }
        return reader_index;
    }

    void Detach(int32_t reader_index)
    {
        std::lock_guard<SHMMutex> lck(__mutex);
        __reader_infos[reader_index].__pid = 0;
        __reader_bitset.ClearBit(reader_index);

        GabageCollectUnsafe(reader_index);
    }

private:
    struct SHMMempoolElement
    {
        int32_t __next   = -1;
        int32_t __prev   = -1;
        uint32_t __index = 0;
        Element<T> __t;
        Bitset<M>::BITSET_ARRAY __reader;
    };
    struct ReaderInfo
    {
        pid_t __pid = 0;
    };
    SHMMutex __mutex;
    SHMDeque<SHMMempoolElement> __shm_msg_pool;
    SHMDeque<SHMMempoolElement> __shm_msgs_queue;
    ReaderInfo __reader_infos[M] = {};
    Bitset<M> __reader_bitset;
    SHMMempoolElement __data[N] = {};

    void GabageCollectUnsafe(int32_t reader_index)
    {
        std::vector<int32_t> msg_indexes;
        __shm_msgs_queue.ExecuteAll(__data,
            msg_indexes,
            [data = __data, reader_index = reader_index](
                std::vector<int32_t>& msg_indexs, int32_t msg_index) {
                if (Bitset<M>::GetBit(data[msg_index].__reader, reader_index))
                    msg_indexs.push_back(msg_index);
            });
        for (auto msg_index : msg_indexes)
        {
            Bitset<M>::ClearBit(__data[msg_index].__reader, reader_index);
            if (Bitset<M>::IsEmpty(__data[msg_index].__reader))
            {
                __shm_msgs_queue.Pop(__data, msg_index);
                __shm_msg_pool.Push(__data, msg_index);
            }
        }
    }
};