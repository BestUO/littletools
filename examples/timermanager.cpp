// #define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
// #define ANKERL_NANOBENCH_IMPLEMENT
#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/timermanager.hpp"

TEST_CASE("TimerManagerV1 base")
{
    auto timerManager
        = timermanager::V1::TimerManager<std::string>::GetInstance();
    class TestTimerManager
    {
    public:
        void TimeOutFun(int a, int b)
        {
            std::cout << "TestTimerManager: " << a + b << std::endl;
        }
    };
    TestTimerManager testTimerManager;
    timerManager->AddAlarm(
        std::chrono::system_clock::now() + std::chrono::seconds(5),
        "5",
        std::bind(&TestTimerManager::TimeOutFun, &testTimerManager, 2, 3));
    timerManager->AddAlarm(
        std::chrono::system_clock::now() + std::chrono::seconds(2),
        "2",
        std::bind(&TestTimerManager::TimeOutFun, &testTimerManager, 1, 1));
    timerManager->AddAlarm(
        std::chrono::system_clock::now() + std::chrono::seconds(4),
        "2",
        std::bind(&TestTimerManager::TimeOutFun, &testTimerManager, 2, 2));
    timerManager->AddAlarm(
        std::chrono::system_clock::now() + std::chrono::seconds(3),
        "3",
        std::bind(&TestTimerManager::TimeOutFun, &testTimerManager, 1, 2));
    timerManager->AddAlarm(
        std::chrono::system_clock::now() + std::chrono::seconds(3),
        "3",
        std::bind(&TestTimerManager::TimeOutFun, &testTimerManager, 1, 2));
    timerManager->DeleteAlarm(std::string("3"));
    sleep(6);
    timermanager::V1::TimerManager<std::string>::GetInstance()
        ->StopTimerManager();
}

TEST_CASE("testing TimerManagerV1 AddAlarm DeleteAlarm")
{
    struct Key
    {
        std::string name;
        int id;
    };

    auto fun2 = [](int a, int b) {
        std::cout << a << '\t' << b << std::endl;
    };
    auto tm = timermanager::V1::TimerManager<Key>::GetInstance();
    for (int i = 5; i < 10; i++)
        tm->AddAlarm(std::chrono::system_clock::now() + std::chrono::seconds(2),
            Key{"123", i},
            std::bind(fun2, i, i + 1));
    ankerl::nanobench::Bench().run("DeleteAlarm cost time", [&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    });
    CHECK(tm->DeleteAlarm(std::bind(
              [](const Key& t, int id) {
                  return t.id == id;
              },
              std::placeholders::_1,
              7))
        == true);
    sleep(3);
    tm->StopTimerManager();
}

TEST_CASE("testing TimerManagerV1 AddAlarmInterval DeleteAlarm")
{
    struct Key
    {
        std::string name;
        int id;
    };

    auto fun2 = [](int a, int b) {
        std::cout << a << '\t' << b << std::endl;
    };
    auto tm = timermanager::V1::TimerManager<Key>::GetInstance();

    tm->AddAlarm(std::chrono::system_clock::now() + std::chrono::seconds(2),
        Key{"run fun1interval", 3},
        std::bind(fun2, 7, 8),
        std::chrono::seconds(1));
    sleep(5);

    CHECK_MESSAGE(tm->DeleteAlarm(std::bind(
                      [](const Key& t, int id) {
                          return t.id == id;
                      },
                      std::placeholders::_1,
                      3))
            == true,
        "可能再执行删除操作的时候正好从定时器取出，因此删除失败");

    sleep(2);
    tm->StopTimerManager();
}

TEST_CASE("testing TimerManagerV1 two same key")
{
    struct Key
    {
        std::string name;
        int id;
    };

    auto fun2 = [](int a, int b) {
        std::cout << a << '\t' << b << std::endl;
    };
    auto tm = timermanager::V1::TimerManager<Key>::GetInstance();
    tm->AddAlarm(std::chrono::system_clock::now() + std::chrono::seconds(2),
        Key{"123", 6},
        std::bind(fun2, 8, 8));
    tm->AddAlarm(std::chrono::system_clock::now() + std::chrono::seconds(4),
        Key{"123", 6},
        std::bind(fun2, 8, 8));
    CHECK(tm->DeleteAlarm(std::bind(
              [](const Key& t, int id) {
                  return t.id == id;
              },
              std::placeholders::_1,
              6))
        == true);
    sleep(6);
    tm->StopTimerManager();
}
