#include <atomic>
#include <iostream>
#include <mutex>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/threadpool.hpp"

TEST_CASE("threadpool_v1")
{
    auto fun = []() {};
    threadpool::v1::ThreadPool<FreeLockRingQueue<std::function<void()>>> tp1(1);
    tp1.EnqueueFun(fun);
    tp1.StopThreadPool();

    threadpool::v1::ThreadPool<FreeLockRingQueue<std::string>> tp3(1);
    tp3.EnqueueStr("aa");
    tp3.StopThreadPool();
}

TEST_CASE("threadpool_v2")
{
    auto fun  = []() {};
    auto fun2 = [](int a) {
        return a;
    };
    threadpool::v2::ThreadPool<FreeLockRingQueue<std::function<void()>>> tp1(1);
    tp1.EnqueueFun(fun);
    tp1.StopThreadPool();

    threadpool::v2::ThreadPool<FreeLockRingQueue<std::function<int()>>> tp2(1);
    auto future = tp2.EnqueueFun(std::function<int()>([]() -> int {
        return 1;
    }));
    CHECK_EQ(1, future.get());
    tp2.StopThreadPool();
}

TEST_CASE("threadpool_v3")
{
    threadpool::v3::ThreadPoll<int, 1> thread_pool;
    std::vector<int> result;
    thread_pool.Start([&result](const int& i) {
        result.push_back(i);
    });
    usleep(1000);
    for (int i = 0; i < 10; i++)
        thread_pool.Put(i, std::move(i));
    usleep(1000);
    CHECK_EQ(10, result.size());
    CHECK_EQ(result, std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}));
    thread_pool.Stop();
}

TEST_CASE("threadpool_v4")
{
    threadpool::v4::ThreadPoll<int, 1> thread_pool;
    std::vector<int> result;
    thread_pool.Start([&result](const int& i) {
        result.push_back(i);
    });
    usleep(1000);
    for (int i = 0; i < 10; i++)
        thread_pool.Put(i, std::move(i));
    usleep(1000);
    CHECK_EQ(10, result.size());
    CHECK_EQ(result, std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}));
    thread_pool.Stop();
}

TEST_CASE("threadpool_bench")
{
    SUBCASE("use threadpool_v3")
    {
        threadpool::v3::ThreadPoll<int> thread_pool;
        std::atomic<int> recv_count = 0;
        thread_pool.Start([&recv_count](const int& i) {
            ++recv_count;
        });
        usleep(1000);

        int send_count1 = 0;
        std::thread send1([&thread_pool, &send_count1]() {
            auto now = std::chrono::high_resolution_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::high_resolution_clock::now() - now)
                       .count()
                < 10)
            {
                thread_pool.Put(1, 1);
                ++send_count1;
            }
        });
        int send_count2 = 0;
        std::thread send2([&thread_pool, &send_count2]() {
            auto now = std::chrono::high_resolution_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::high_resolution_clock::now() - now)
                       .count()
                < 10)
            {
                thread_pool.Put(1, 1);
                ++send_count2;
            }
        });
        send1.join();
        send2.join();
        std::cout << "threadpool_v3 send_count1 num: " << send_count1
                  << " send_count2 num: " << send_count2
                  << " total send: " << send_count1 + send_count2
                  << " recv num: " << recv_count << std::endl;

        thread_pool.Stop();
    }
    SUBCASE("use threadpool_v4")
    {
        threadpool::v4::ThreadPoll<int> thread_pool;
        std::atomic<int> recv_count = 0;
        thread_pool.Start([&recv_count](const int& i) {
            ++recv_count;
        });
        usleep(1000);

        int send_count1 = 0;
        std::thread send1([&thread_pool, &send_count1]() {
            auto now = std::chrono::high_resolution_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::high_resolution_clock::now() - now)
                       .count()
                < 10)
            {
                thread_pool.Put(1, 1);
                ++send_count1;
            }
        });
        int send_count2 = 0;
        std::thread send2([&thread_pool, &send_count2]() {
            auto now = std::chrono::high_resolution_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::high_resolution_clock::now() - now)
                       .count()
                < 10)
            {
                thread_pool.Put(2, 1);
                ++send_count2;
            }
        });
        send1.join();
        send2.join();
        std::cout << "threadpool_v4 send_count1 num: " << send_count1
                  << " send_count2 num: " << send_count2
                  << " total send: " << send_count1 + send_count2
                  << " recv num: " << recv_count << std::endl;

        thread_pool.Stop();
    }
}