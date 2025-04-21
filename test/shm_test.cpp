#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <mutex>

#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/shm/bitset.hpp"
#include "tools/shm/shm_factory.hpp"
#include "tools/shm/shm_mempool.hpp"
#include "tools/shm/shm_msgqueue.hpp"
#include "tools/shm/shm_deque.hpp"
#include "tools/shm/shm_mutex.hpp"
#include "tools/shm/shm_cv.hpp"

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
    int32_t times   = 1024 * 1024;
    auto client_pid = fork();
    if (client_pid == 0)
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
                queue->SendMsgToAny(index);
            }
        }
        exit(0);
    }
    else
    {
        SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
        while (true)
        {
            if (times <= 0)
                break;

            auto [index, s_ptr] = queue->RecvOneMsg();
            if (index != -1)
            {
                times--;
                CHECK(s_ptr->a == index);
                queue->Free(index);
            }
        }

        int ws = 0;
        waitpid(client_pid, &ws, 0);
    }
}

TEST_CASE("SHMV2_SHMMsgQueue_SendMsgToAll")
{
    int32_t times   = 1024 * 1024;
    auto client_pid = fork();
    if (client_pid == 0)
    {
        SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
        while (true)
        {
            if (times <= 0)
                break;

            auto [index, s_ptr] = queue->Allocate();
            if (index != -1)
            {
                s_ptr->a = times;
                times--;
                queue->SendMsgToAll(index);
            }
        }
        exit(0);
    }
    else
    {
        SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
        int32_t reader_index = queue->Attach();
        while (true)
        {
            if (times <= 0)
                break;

            auto [index, s_ptr] = queue->RecvTopMsgWithReader(reader_index);
            if (index != -1)
            {
                CHECK(s_ptr->a == times);
                times--;
                queue->Free(index);
            }
        }
        queue->Detach(reader_index);
        int ws = 0;
        waitpid(client_pid, &ws, 0);
    }
}

TEST_CASE("SHMV2_SHMMsgQueue_SendMsgToAll_2")
{
    std::vector<pid_t> child_pids;
    SHMFactory<SHMMsgQueue<TestStruct, 1024>> statistic("SHMMsgQueue");
    int32_t send_processes = 2;
    int32_t recv_processes = 1;
    int32_t times          = 1024 * 1024;

    for (int i = 0; i < send_processes; i++)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            {
                SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
                int count = 0;
                while (count < times)
                {
                    auto [index, s_ptr] = queue->Allocate();
                    if (index != -1)
                    {
                        s_ptr->a = index;
                        queue->SendMsgToAll(index);
                        count++;
                    }
                }
            }
            printf("%d send exit\n", getpid());
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
                int count            = 0;
                while (count < times * send_processes)
                {
                    auto [index, s_ptr]
                        = queue->RecvTopMsgWithReader(reader_index);
                    if (index != -1)
                    {
                        CHECK(s_ptr->a == index);
                        queue->Free(index);
                        count++;
                    }
                }
                queue->Detach(reader_index);
                std::cout << getpid() << " recv: " << count << reader_index
                          << std::endl;
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

// TEST_CASE("SHMV2_SHMMsgQueue_SendMsgToAll_2")
// {
//     std::vector<pid_t> child_pids;
//     int32_t send_processes = 1;
//     int32_t recv_processes = 1;

//     for (int i = 0; i < send_processes; i++)
//     {
//         pid_t pid = fork();
//         if (pid == 0)
//         {
//             SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
//             int count = 0;
//             while (count < 1000)
//             {
//                 auto [index, s_ptr] = queue->Allocate();
//                 if (index != -1)
//                 {
//                     s_ptr->a = index;
//                     queue->SendMsgToAll(index);
//                     count++;
//                 }
//             }
//             std::cout << getpid() << " send: " << count << std::endl;
//             exit(0);
//         }
//         else if (pid > 0)
//         {
//             child_pids.push_back(pid);
//         }
//     }

//     for (int i = 0; i < recv_processes; i++)
//     {
//         pid_t pid = fork();
//         if (pid == 0)
//         {
//             SHMFactory<SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
//             int32_t reader_index = queue->Attach();
//             int count            = 0;
//             while (count < 1000)
//             {
//                 auto [index, s_ptr] =
//                 queue->RecvTopMsgWithReader(reader_index); if (index != -1)
//                 {
//                     CHECK(s_ptr->a == index);
//                     queue->Free(index);
//                     count++;
//                 }
//             }
//             std::cout << getpid() << " recv: " << count << std::endl;
//             exit(0);
//         }
//         else if (pid > 0)
//         {
//             child_pids.push_back(pid);
//         }
//     }

//     for (pid_t pid : child_pids)
//     {
//         int status;
//         waitpid(pid, &status, 0);
//     }
// }

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