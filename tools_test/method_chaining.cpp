#include <chrono>
#include <condition_variable>
#include <future>
#include <random>
#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/timermanager.hpp"
#include "tools/uuid.hpp"
#include "tools/method_chaining.hpp"

TEST_CASE("method_chaining_ThenOrder")
{
    auto timerManager_v4 = timermanager::v4::TimerManager<UUID>::GetInstance();
    timerManager_v4->StartTimerManager();
    auto promiseptr          = std::make_shared<std::promise<void>>();
    std::future<void> future = promiseptr->get_future();

    auto thenable = MakeThenOrder([]() {
        return 42;
    });

    thenable
        ->ThenOrder([](int result) {
            std::cout << "ThenOrder 1: " << result << std::endl;
            return result * 2;
        })
        ->ThenOrder([](int result) {
            std::cout << "ThenOrder 2: " << result << std::endl;
        })
        ->ThenOrder([]() {
            std::cout << "ThenOrder 3: " << std::endl;
        })
        ->ThenOrder([]() {
            std::cout << "ThenOrder 4: " << std::endl;
            return 2;
        })
        ->ThenOrder([promiseptr](int result) {
            std::cout << "ThenOrder 5: " << result << std::endl;
            promiseptr->set_value();
        });

    future.wait();
    timerManager_v4->StopTimerManager();
}

TEST_CASE("method_chaining_ThenOrderInWorkPool")
{
    auto timerManager_v4 = timermanager::v4::TimerManager<UUID>::GetInstance();
    timerManager_v4->StartTimerManager();
    auto promiseptr          = std::make_shared<std::promise<void>>();
    std::future<void> future = promiseptr->get_future();

    auto thenable = MakeThenOrder([]() {
        return 42;
    });

    thenable
        ->ThenOrderInWorkPool([](int result) {
            std::cout << "ThenOrderInWorkPool 1: " << result << std::endl;
            return result * 2;
        })
        ->ThenOrderInWorkPool([](int result) {
            std::cout << "ThenOrderInWorkPool 2: " << result << std::endl;
        })
        ->ThenOrderInWorkPool([]() {
            std::cout << "ThenOrderInWorkPool 3: " << std::endl;
        })
        ->ThenOrderInWorkPool([]() {
            std::cout << "ThenOrderInWorkPool 4: " << std::endl;
            return 2;
        })
        ->ThenOrderInWorkPool([promiseptr](int result) {
            std::cout << "ThenOrderInWorkPool 5: " << result << std::endl;
            promiseptr->set_value();
        });
    future.wait();
    timerManager_v4->StopTimerManager();
}