#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/threadpool.hpp"

void tp1test()
{
    std::cout << "tp1test" << std::endl;
    return;
}

TEST_CASE("testing threadpool")
{
    ThreadPool<FreeLockRingQueue<std::function<void()>>> tp1(1);
    tp1.EnqueueStr(tp1test);
    sleep(1);
    tp1.StopThreadPool();

    ThreadPool<FreeLockRingQueue<std::function<int()>>> tp2(1);
    tp2.EnqueueStr([]() {
        std::cout << "tp2test" << std::endl;
        return 1;
    });
    sleep(1);
    tp2.StopThreadPool();

    ThreadPool<FreeLockRingQueue<std::string>> tp3(1);
    tp3.EnqueueStr("aa");
    sleep(1);
    tp3.StopThreadPool();
}