#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/threadpool.hpp"

TEST_CASE("testing threadpool v1")
{
    auto fun = []() {};
    threadpool::v1::ThreadPool<FreeLockRingQueue<std::function<void()>>> tp1(1);
    tp1.EnqueueFun(fun);
    tp1.StopThreadPool();

    threadpool::v1::ThreadPool<FreeLockRingQueue<std::string>> tp3(1);
    tp3.EnqueueStr("aa");
    tp3.StopThreadPool();
}

int test(int a)
{
    return a;
}

TEST_CASE("testing threadpool v2")
{
    auto fun  = []() {};
    auto fun2 = [](int a) {
        return a;
    };
    threadpool::v2::ThreadPool<FreeLockRingQueue<std::function<void()>>> tp1(1);
    tp1.EnqueueFun(fun);
    tp1.StopThreadPool();

    threadpool::v2::ThreadPool<FreeLockRingQueue<std::function<int()>>> tp2(1);
    tp2.EnqueueFun(std::function<int()>([]() -> int {
        return 1;
    }));
    tp2.StopThreadPool();
}