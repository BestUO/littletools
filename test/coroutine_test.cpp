#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/coroutine_tasks_pool/any_wrapper.hpp"
#include "tools/coroutine_tasks_pool/custom_awaiter.hpp"
#include "tools/coroutine_tasks_pool/lazy_task.hpp"
#include "tools/coroutine_tasks_pool/base_fun.hpp"

TEST_CASE("coroutine_task_not_void")
{
    auto fun = [](int i) -> coroutine_task::LazyTask<int> {
        co_return i;
    };
    fun(1).AsyncStart2([](auto result) {
        CHECK(result.GetValue() == 1);
    });
    sleep(1);
}

TEST_CASE("coroutine_task_void")
{
    auto fun = [](int i) -> coroutine_task::LazyTask<void> {
        CHECK(i == 1);
        co_return;
    };
    fun(1).AsyncStart2([](auto result) {});
    sleep(1);
}

TEST_CASE("coroutine_task_nested_int")
{
    auto fun = [](int i) -> coroutine_task::LazyTask<void> {
        auto fun2 = [](int i) -> coroutine_task::LazyTask<int> {
            co_return i;
        };
        auto result = co_await fun2(i);
        CHECK(result.GetValue() == 1);
        co_return;
    };
    fun(1).AsyncStart2([](auto result) {});
    sleep(1);
}

TEST_CASE("coroutine_task_nested_void")
{
    auto fun = [](int i) -> coroutine_task::LazyTask<int> {
        auto fun2 = [](int i) -> coroutine_task::LazyTask<void> {
            co_return;
        };
        co_await fun2(i);
        co_return i;
    };
    fun(1).AsyncStart2([](auto result) {
        CHECK(result.GetValue() == 1);
    });
    sleep(1);
}

TEST_CASE("coroutine_task_async")
{
    auto fun = [](int i) -> coroutine_task::LazyTask<void> {
        auto fun2 = [](int i) -> coroutine_task::LazyTask<int> {
            co_return i;
        };
        auto result = co_await fun2(i);
        CHECK(result.GetValue() == 1);
        co_return;
    };
    fun(1).AsyncStart2([](auto result) {});
    sleep(1);
}

TEST_CASE("coroutine_task_sleep")
{
    auto fun = []() -> coroutine_task::LazyTask<void> {
        co_await SleepAwaiter{std::chrono::milliseconds(50)};
        co_return;
    };
    fun().AsyncStart2([](auto result) {});
    sleep(1);
}

TEST_CASE("coroutine_task_collect_any_int_string")
{
    auto fun = []() -> coroutine_task::LazyTask<void> {
        auto fun1 = [](int i) -> coroutine_task::LazyTask<int> {
            co_await SleepAwaiter{std::chrono::milliseconds(50)};
            co_return i;
        };
        auto fun2 = [](std::string i) -> coroutine_task::LazyTask<std::string> {
            co_return i;
        };
        auto anyawaiter = co_await CollectAnyAwaiter(fun1(3), fun2("sss"));
        CHECK(
            std::get<AnyWrapper<std::string>>(anyawaiter).GetValue() == "sss");
        co_return;
    };
    fun().AsyncStart2([](auto result) {});
    sleep(1);
}

TEST_CASE("coroutine_task_collect_any_int_void")
{
    auto fun = []() -> coroutine_task::LazyTask<void> {
        auto fun1 = [](int i) -> coroutine_task::LazyTask<int> {
            co_await SleepAwaiter{std::chrono::milliseconds(10)};
            co_return i;
        };
        auto anyawaiter = co_await CollectAnyAwaiter(fun1(1),
            coroutine_task::CoroutineSleep(std::chrono::milliseconds(50)));
        CHECK(std::get<AnyWrapper<int>>(anyawaiter).GetValue() == 1);
        co_return;
    };
    fun().AsyncStart2([](auto result) {});
    sleep(1);
}

TEST_CASE("coroutine_task_collect_all_int_void_string")
{
    auto fun = []() -> coroutine_task::LazyTask<void> {
        auto fun1 = [](int i) -> coroutine_task::LazyTask<int> {
            co_await SleepAwaiter{std::chrono::milliseconds(10)};
            co_return i;
        };
        auto fun2 = [](std::string s) -> coroutine_task::LazyTask<std::string> {
            co_await SleepAwaiter{std::chrono::milliseconds(20)};
            co_return s;
        };
        auto results = co_await CollectAllAwaiter(fun1(1),
            fun2("sss"),
            coroutine_task::CoroutineSleep(std::chrono::milliseconds(30)));
        co_return;
    };
    fun().AsyncStart2([](auto result) {});
    sleep(1);
}

TEST_CASE("coroutine_task_collect_final")
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}