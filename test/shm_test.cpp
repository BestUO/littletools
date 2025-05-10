#include <cstdint>
#include <unistd.h>
#include <sys/wait.h>
#include <mutex>
#include "doctest/doctest.h"
#include "tools/shm/shm_bitset.hpp"
#include "tools/shm/shm_global.hpp"
#include "tools/shm/shm_factory.hpp"
#include "tools/shm/shm_mempool.hpp"
#include "tools/shm/shm_msgqueue.hpp"
#include "tools/shm/shm_deque.hpp"
#include "tools/shm/shm_mutex.hpp"
#include "tools/shm/shm_msgqueue_manager.hpp"
#include "tools/shm/shm_msgqueue_manager_group.hpp"

struct TestStruct
{
    int a;
};

struct DequeStruct
{
    int32_t __next   = -1;
    int32_t __prev   = -1;
    uint32_t __index = 0;
    Element<TestStruct> __t;
};

TEST_CASE("SHMV2_bitset")
{
    auto test_bitset = [](auto size) {
        constexpr auto SIZE = size;
        Bitset<SIZE> bitset;
        for (int i = 0; i < SIZE; i++)
        {
            CHECK(bitset.GetZeroIndex() == i);
            CHECK(bitset.GetBit(i) == false);
            bitset.SetBit(i);
            CHECK(bitset.GetBit(i) == true);
            CHECK(bitset.PopCount() == i + 1);
        }
        CHECK(bitset.PopCount() == SIZE);
        for (int i = 0; i < SIZE; i++)
        {
            bitset.ClearBit(i);
            CHECK(bitset.GetBit(i) == false);
            CHECK(bitset.PopCount() == SIZE - 1 - i);
        }
        CHECK(bitset.PopCount() == 0);
    };
    test_bitset(std::integral_constant<int, 63>{});
    test_bitset(std::integral_constant<int, 64>{});
    test_bitset(std::integral_constant<int, 65>{});
}

TEST_CASE("SHMV2_SHMMutex")
{
    SHMMutex mutex;
    mutex.lock();
    CHECK(mutex.try_lock() == false);
    mutex.unlock();
    CHECK(mutex.try_lock() == true);
    mutex.unlock();

    std::vector<pid_t> child_pids;
    int32_t processes = 2;
    int32_t times     = 1024 * 1024;
    SHMFactory<int32_t> count("count");
    (*count) = 0;
    for (int i = 0; i < processes; i++)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            {
                SHMFactory<SHMMutex> TTT("Mutex");
                SHMFactory<int32_t> count("count");
                while (times--)
                {
                    std::lock_guard<SHMMutex> lck(*TTT);
                    (*count)++;
                }
            }
            exit(0);
        }
        else
        {
            child_pids.push_back(pid);
        }
    }
    for (pid_t pid : child_pids)
    {
        int status;
        waitpid(pid, &status, 0);
    }
    CHECK(*count == processes * times);
}

TEST_CASE("SHMV2_SHMDeque")
{
    auto test_deque = [](auto size) {
        constexpr auto N      = size;
        DequeStruct __data[N] = {};
        SHMFactory<SHMDeque<DequeStruct>> __deque("Deque");

        CHECK(__deque->IsEmpty() == true);
        for (size_t i = 0; i < N; i++)
        {
            __data[i].__index = i;
            __deque->Push(__data, i);
        }
        CHECK(__deque->IsEmpty() == false);

        {
            std::vector<int32_t> msg_indexes;
            __deque->ExecuteAll(__data,
                msg_indexes,
                [](std::vector<int32_t>& msg_indexs, int32_t msg_index) {
                    msg_indexs.push_back(msg_index);
                });
            CHECK(msg_indexes.size() == N);
            for (size_t i = 0; i < N; i++)
            {
                CHECK(__data[i].__index == msg_indexes[i]);
            }
        }

        for (size_t i = 0; i < N; i++)
        {
            __deque->Pop(__data, i);

            std::vector<int32_t> msg_indexes;
            __deque->ExecuteAll(__data,
                msg_indexes,
                [](std::vector<int32_t>& msg_indexs, int32_t msg_index) {
                    msg_indexs.push_back(msg_index);
                });
            size_t count = 0;
            for (auto msg_index : msg_indexes)
            {
                CHECK(msg_index == i + (++count));
            }
            return;
        }
    };
    test_deque(std::integral_constant<int, 63>{});
    test_deque(std::integral_constant<int, 64>{});
    test_deque(std::integral_constant<int, 65>{});
}

TEST_CASE("SHMV2_SHMMsgQueue_SendMsgToAny")
{
    int32_t times = 1024 * 1024;
    std::vector<pid_t> child_pids;
    int32_t send_processes = 2;
    int32_t recv_processes = 2;
    SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
    for (int i = 0; i < send_processes; i++)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            {
                SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
                while (true)
                {
                    if (times <= 0)
                        break;

                    auto [index, s_ptr] = queue->Allocate();
                    if (index != -1)
                    {
                        times--;
                        s_ptr->a = index;
                        while (queue->SendMsgToAny(index) == Result::NOREADER)
                        {
                            usleep(1);
                        }
                    }
                }
            }
            exit(0);
        }
        else if (pid > 0)
        {
            child_pids.push_back(pid);
        }
    }
    for (int i = 0; i < recv_processes; i++)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            {
                SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
                int32_t reader_index = queue->Attach();
                while (true)
                {
                    if (times <= 0)
                        break;

                    auto [index, s_ptr]
                        = queue->RecvTopMsgWithReader(reader_index);
                    if (index != -1)
                    {
                        times--;
                        CHECK(s_ptr->a == index);
                        queue->Free(index, reader_index);
                    }
                }
                queue->Detach(reader_index);
            }
            exit(0);
        }
        else if (pid > 0)
        {
            child_pids.push_back(pid);
        }
    }

    for (pid_t pid : child_pids)
    {
        int status;
        waitpid(pid, &status, 0);
    }
}

TEST_CASE("SHMV2_SHMMsgQueue_SendMsgToAll")
{
    int32_t times = 1024 * 10;
    std::vector<pid_t> child_pids;
    int32_t send_processes = 2;
    int32_t recv_processes = 2;
    SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
    for (int i = 0; i < recv_processes; i++)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            {
                SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
                int32_t reader_index = queue->Attach();
                times                = times * send_processes;
                while (true)
                {
                    if (times <= 0)
                        break;

                    auto [index, s_ptr]
                        = queue->RecvTopMsgWithReader(reader_index);
                    if (index != -1)
                    {
                        times--;
                        CHECK(s_ptr->a == index);
                        queue->Free(index, reader_index);
                    }
                }
                queue->Detach(reader_index);
            }
            exit(0);
        }
        else if (pid > 0)
        {
            child_pids.push_back(pid);
        }
    }
    sleep(1);
    for (int i = 0; i < send_processes; i++)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            {
                SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
                while (true)
                {
                    if (times <= 0)
                        break;

                    auto [index, s_ptr] = queue->Allocate();
                    if (index != -1)
                    {
                        times--;
                        s_ptr->a = index;
                        while (queue->SendMsgToAll(index) == Result::NOREADER)
                        {
                            usleep(1);
                        }
                    }
                }
            }
            exit(0);
        }
        else if (pid > 0)
        {
            child_pids.push_back(pid);
        }
    }

    for (pid_t pid : child_pids)
    {
        int status;
        waitpid(pid, &status, 0);
    }
}

TEST_CASE("SHMV2_SHMMemPool")
{
    SHMFactory<SHMMemoryPool<TestStruct, 1>> pool("MemoryPool");
    int32_t times = 1024 * 1024;
    while (true)
    {
        if (times <= 0)
            break;

        auto [index, s_ptr] = pool->Allocate();
        if (index.__index != -1)
        {
            times--;
            s_ptr->a = times;
            pool->Free(index);
        }
    }
}

TEST_CASE("SHMV2_SHMMsgQueueManager")
{
    int32_t times = 1024 * 2;
    std::vector<pid_t> child_pids;
    int32_t send_processes = 2;
    int32_t recv_processes = 2;
    for (int i = 0; i < recv_processes; i++)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            {
                SHMMsgQueueManager<TestStruct, 1024>* queue
                    = SHMMsgQueueManager<TestStruct, 1024>::GetInstance(
                        "SHMV2_SHMMsgQueueManager_TestStruct");
                queue->Attach([](int32_t index, TestStruct* s_ptr) {
                    CHECK(s_ptr->a == index);
                });
                times = times * send_processes;
                while (true)
                {
                    if (times <= 0)
                        break;

                    if (int32_t index = queue->RecvTopMsgOnce(); index != -1)
                    {
                        times--;
                        queue->Free(index);
                    }
                }
                queue->Detach();
            }
            exit(0);
        }
        else if (pid > 0)
        {
            child_pids.push_back(pid);
        }
    }
    sleep(1);
    for (int i = 0; i < send_processes; i++)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            {
                SHMMsgQueueManager<TestStruct, 1024>* queue
                    = SHMMsgQueueManager<TestStruct, 1024>::GetInstance(
                        "SHMV2_SHMMsgQueueManager_TestStruct");
                while (true)
                {
                    if (times <= 0)
                        break;

                    auto [index, s_ptr] = queue->Allocate();
                    if (index != -1)
                    {
                        times--;
                        s_ptr->a = index;
                        while (queue->SendMsgToAll(index) == Result::NOREADER)
                        {
                            usleep(1);
                        }
                    }
                }
            }
            exit(0);
        }
        else if (pid > 0)
        {
            child_pids.push_back(pid);
        }
    }
    for (pid_t pid : child_pids)
    {
        int status;
        waitpid(pid, &status, 0);
    }
}

TEST_CASE("SHMV2_SHMMsgQueueManager_GC")
{
    int32_t times = 15;
    std::vector<pid_t> child_pids;
    int32_t send_processes = 1;
    int32_t recv_processes = 1;
    for (int i = 0; i < recv_processes; i++)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            {
                auto queue = SHMMsgQueueManager<TestStruct, 8>::GetInstance(
                    "SHMV2_SHMMsgQueueManager_GC");
                queue->Attach([](int32_t index, TestStruct* s_ptr) {
                    CHECK(s_ptr->a == index);
                });
                sleep(2);
                exit(0);
            }
        }
        else if (pid > 0)
        {
            child_pids.push_back(pid);
        }
    }
    sleep(1);
    for (int i = 0; i < send_processes; i++)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            {
                auto queue = SHMMsgQueueManager<TestStruct, 8>::GetInstance(
                    "SHMV2_SHMMsgQueueManager_GC");
                while (true)
                {
                    if (times <= 0)
                        break;

                    auto [index, s_ptr] = queue->Allocate();
                    if (index != -1)
                    {
                        times--;
                        s_ptr->a = index;
                        while (queue->SendMsgToAll(index) == Result::NOREADER)
                        {
                            CHECK(times == 6);
                            exit(0);
                        }
                    }
                    else
                    {
                        queue->GabageCollect();
                    }
                }
            }
        }
        else if (pid > 0)
        {
            child_pids.push_back(pid);
        }
    }
    for (pid_t pid : child_pids)
    {
        int status;
        waitpid(pid, &status, 0);
    }
}

TEST_CASE("SHMV2_SHMMsgQueueManager_group")
{
    SHMMsgQueueManagerGroup* queue_group
        = SHMMsgQueueManagerGroup::GetInstance();
}