#pragma once

#include <stddef.h>
#include <utility>
#include <unistd.h>
#include "tools/shm/shm_msgqueue.hpp"
#include "tools/shm/global.hpp"
#include "tools/shm/shm_factory.hpp"

template <PodType T, size_t N = SHM_QUEUE_SIZE, size_t M = READER_SIZE>
class SHMMsgQueueManager
{
public:
    void Attach()
    {
        auto pid = getpid();
    }

private:
    struct SHMMsgQueueElement
    {
        T __t;
    };
    SHMFactory<SHMMsgQueue<SHMMsgQueueElement, N, M>> __queue;
    std::pair<pid_t, int32_t> __pid2reader;
};