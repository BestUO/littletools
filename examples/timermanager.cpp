// #define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
// #define ANKERL_NANOBENCH_IMPLEMENT
#include <chrono>
#include <random>
#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/timermanager.hpp"
#include "tools/uuid.hpp"

TEST_CASE("TimerManager_v1_base")
{
    auto tm = timermanager::v1::TimerManager<std::string>::GetInstance();
    class TestTimerManager
    {
    public:
        void TimeOutFun(int a, int b)
        {
            std::cout << "TestTimerManager: " << a + b << std::endl;
        }
    };
    TestTimerManager testTimerManager;
    tm->AddAlarm(std::chrono::seconds(5),
        "5",
        std::bind(&TestTimerManager::TimeOutFun, &testTimerManager, 2, 3));
    tm->AddAlarm(std::chrono::seconds(2),
        "2",
        std::bind(&TestTimerManager::TimeOutFun, &testTimerManager, 1, 1));
    tm->AddAlarm(std::chrono::seconds(4),
        "2",
        std::bind(&TestTimerManager::TimeOutFun, &testTimerManager, 2, 2));
    tm->AddAlarm(std::chrono::seconds(3),
        "3",
        std::bind(&TestTimerManager::TimeOutFun, &testTimerManager, 1, 2));
    tm->AddAlarm(std::chrono::seconds(3),
        "3",
        std::bind(&TestTimerManager::TimeOutFun, &testTimerManager, 1, 2));
    tm->DeleteAlarm(std::string("3"));
    sleep(6);
    timermanager::v1::TimerManager<std::string>::GetInstance()
        ->StopTimerManager();
}

TEST_CASE("TimerManager_v1_interval")
{
    auto tm = timermanager::v1::TimerManager<std::string>::GetInstance();
    tm->StartTimerManager();
    tm->AddAlarm(std::chrono::milliseconds(10),
        "10",
        std::bind(
            [](int a, int b) {
                std::cout << "TestTimerManager: " << a + b << std::endl;
            },
            5,
            5),
        std::chrono::milliseconds(10));
    usleep(30000);
    tm->DeleteAlarm(std::string("10"));
    timermanager::v1::TimerManager<std::string>::GetInstance()
        ->StopTimerManager();
}

TEST_CASE("TimerManager_v1_two_same_key")
{
    struct Key
    {
        std::string name;
        int id;
    };

    auto fun2 = [](int a, int b) {
        std::cout << a << '\t' << b << std::endl;
    };
    auto tm = timermanager::v1::TimerManager<Key>::GetInstance();
    tm->AddAlarm(
        std::chrono::milliseconds(2), Key{"123", 6}, std::bind(fun2, 8, 8));
    tm->AddAlarm(
        std::chrono::milliseconds(4), Key{"123", 6}, std::bind(fun2, 8, 8));
    sleep(1);
    tm->StopTimerManager();
}

TEST_CASE("TimerManager_v2")
{
    auto func = [](int a, int b) {
        std::cout << a << '\t' << b << std::endl;
    };
    auto tm = timermanager::v2::TimerManager<std::string>::GetInstance();
    tm->StartTimerManager();

    tm->AddAlarm(
        std::chrono::milliseconds(500), "0", "5", std::bind(func, 2, 3));
    tm->AddAlarm(
        std::chrono::milliseconds(200), "0", "2", std::bind(func, 1, 1));
    tm->AddAlarm(
        std::chrono::milliseconds(400), "0", "4", std::bind(func, 2, 2));
    tm->AddAlarm(
        std::chrono::milliseconds(300), "0", "3-1", std::bind(func, 1, 2));
    tm->AddAlarm(
        std::chrono::milliseconds(300), "0", "3-2", std::bind(func, 1, 20));
    tm->AddAlarm(
        std::chrono::milliseconds(200), "T", "", std::bind(func, 1, 100));
    tm->AddAlarm(
        std::chrono::milliseconds(200), "T", "", std::bind(func, 1, 100));
    CHECK_EQ(tm->DeleteAlarm("0", "3-2"), true);
    CHECK_EQ(tm->DeleteAlarm("T"), true);
    sleep(1);
    tm->StopTimerManager();
}

TEST_CASE("TimerManager_v2_interval")
{
    auto func = [](int a, int b) {
        std::cout << a << '\t' << b << std::endl;
    };
    auto tm = timermanager::v2::TimerManager<std::string>::GetInstance();
    tm->StartTimerManager();
    tm->AddAlarm(std::chrono::milliseconds(1000),
        "10",
        "",
        std::bind(func, 5, 5),
        std::chrono::milliseconds(1000));
    usleep(2500000);
    CHECK_MESSAGE(tm->DeleteAlarm("10", ""),
        true,
        "可能再执行删除操作的时候正好从定时器取出，因此删除失败");
    tm->DeleteAlarm("10", "");
    tm->StopTimerManager();
}

TEST_CASE("TimerManager_v2_10w")
{
    auto func = [](std::atomic<int>& count) {
        count++;
    };

    auto GetRandomNumber = [](int from, int to) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(from, to);
        return dis(gen);
    };
    auto tm = timermanager::v2::TimerManager<std::string>::GetInstance();
    tm->StartTimerManager();
    int addnum             = 100000;
    int popnum             = 100;
    int num                = addnum - popnum;
    std::atomic<int> count = {0};

    std::thread t([tm                 = tm,
                      popnum          = popnum,
                      GetRandomNumber = GetRandomNumber]() {
        for (int i = 0; i < popnum; i++)
        {
            while (!tm->DeleteAlarm(std::to_string(GetRandomNumber(0, 10)),
                std::to_string(GetRandomNumber(0, 5)),
                timermanager::v2::TimerManager<std::string>::DeleteModel::ANY))
            { }
        }
    });
    for (int i = 0; i < addnum; i++)
    {
        tm->AddAlarm(std::chrono::milliseconds(GetRandomNumber(0, 1)),
            std::to_string(GetRandomNumber(0, 10)),
            std::to_string(GetRandomNumber(0, 5)),
            std::bind(func, std::ref(count)));
    }
    t.join();
    while (tm->GetSize() > 0)
    {
        usleep(10000);
    }
    CHECK_EQ(count, num);
    tm->StopTimerManager();
}

TEST_CASE("TimerManager_v3")
{
    auto func = [](int a, int b) {
        std::cout << a << '\t' << b << std::endl;
    };
    auto tm = timermanager::v3::TimerManager<std::string>::GetInstance();
    tm->StartTimerManager();

    tm->AddAlarm(
        std::chrono::milliseconds(500), "0", "5", std::bind(func, 2, 3));
    tm->AddAlarm(
        std::chrono::milliseconds(200), "0", "2", std::bind(func, 1, 1));
    tm->AddAlarm(
        std::chrono::milliseconds(400), "0", "4", std::bind(func, 2, 2));
    tm->AddAlarm(
        std::chrono::milliseconds(300), "0", "3-1", std::bind(func, 1, 2));
    tm->AddAlarm(
        std::chrono::milliseconds(300), "0", "3-2", std::bind(func, 1, 20));
    tm->AddAlarm(
        std::chrono::milliseconds(200), "T", "", std::bind(func, 1, 100));
    tm->AddAlarm(
        std::chrono::milliseconds(200), "T", "1-1", std::bind(func, 1, 100));
    tm->DeleteAlarm("0", "3-2");
    tm->DeleteAlarm("T");
    sleep(1);
    tm->StopTimerManager();
}

TEST_CASE("TimerManager_v3_recursive")
{
    auto timerManager = timermanager::v3::TimerManager<int>::GetInstance();
    timerManager->StartTimerManager();
    struct A
    {
        ~A()
        {
            timermanager::v3::TimerManager<int>::GetInstance()->DeleteAlarm(0);
        }
        void callback()
        {
            std::cout << "callback" << std::endl;
        }
        void fun()
        {
            timermanager::v3::TimerManager<int>::GetInstance()->AddAlarm(
                std::chrono::milliseconds(200), 0, "1-1", [this]() {
                    callback();
                });
        }
    };
    {
        auto a = std::make_shared<A>();
        timermanager::v3::TimerManager<int>::GetInstance()->AddAlarm(
            std::chrono::milliseconds(0),
            0,
            "",
            [a]() {
                a->fun();
            },
            std::chrono::milliseconds(100));
        sleep(1);
    }
    timerManager->StopTimerManager();
}

TEST_CASE("TimerManager_v4_base")
{
    auto timerManager = timermanager::v4::TimerManager<int>::GetInstance();
    timerManager->StartTimerManager();

    int count = 0;
    auto func = [&count]() {
        count++;
    };

    uint32_t total = 1024 * 1024;
    for (int i = 0; i < total; i++)
        timerManager->AddAlarm(std::chrono::milliseconds(1), i, "", func);
    while (timerManager->GetSize())
        sleep(1);

    CHECK_EQ(count, total);
    timerManager->StopTimerManager();
}

TEST_CASE("TimerManager_v4_recursive")
{
    auto timerManager = timermanager::v4::TimerManager<int>::GetInstance();
    timerManager->StartTimerManager();
    struct A
    {
        ~A()
        {
            timermanager::v4::TimerManager<int>::GetInstance()->DeleteAlarm(
                0, "1-1");
        }
        void callback()
        {
            std::cout << "callback" << std::endl;
        }
        void fun()
        {
            timermanager::v4::TimerManager<int>::GetInstance()->AddAlarm(
                std::chrono::milliseconds(200), 1, "1-1", [this]() {
                    callback();
                });
        }
    };
    {
        auto a = std::make_shared<A>();
        timermanager::v4::TimerManager<int>::GetInstance()->AddAlarm(
            std::chrono::milliseconds(0),
            1,
            "",
            [a]() {
                a->fun();
            },
            std::chrono::milliseconds(100));
        sleep(1);
    }
    timerManager->StopTimerManager();
}

TEST_CASE("TimerManager_v4_benchmark")
{
    int totalnum      = 1024 * 1024;
    uint32_t epochnum = 1;

    auto timerManager_v3 = timermanager::v3::TimerManager<int>::GetInstance();
    timerManager_v3->StartTimerManager();

    ankerl::nanobench::Bench().epochs(epochnum).run(
        "timerManager_v3", [&totalnum, &timerManager_v3]() {
            int count = 0;
            for (int i = 0; i < totalnum; i++)
                timerManager_v3->AddAlarm(
                    std::chrono::milliseconds(1), i, "", [&count]() {
                        count++;
                    });
            while (count != totalnum)
                usleep(1000);
        });
    timerManager_v3->StopTimerManager();

    auto timerManager_v4 = timermanager::v4::TimerManager<int>::GetInstance();
    timerManager_v4->StartTimerManager();

    ankerl::nanobench::Bench().epochs(epochnum).run(
        "timerManager_v4", [&totalnum, &timerManager_v4]() {
            int count = 0;
            for (int i = 0; i < totalnum; i++)
                timerManager_v4->AddAlarm(
                    std::chrono::milliseconds(1), i, "", [&count]() {
                        count++;
                    });
            while (count != totalnum)
                usleep(1000);
        });
    timerManager_v4->StopTimerManager();
}

// #include "tools/concurrentqueue/concurrentqueue.h"
// #include "tools/concurrentqueue/blockingconcurrentqueue.h"
// TEST_CASE("TimerManager_ttt_v3")
// {
//     uint32_t epochnum = 1;
//     moodycamel::BlockingConcurrentQueue<UUID> q;
//     auto timerManager = timermanager::v3::TimerManager<int>::GetInstance();
//     // timerManager->StartTimerManager();

//     for (int i = 0; i < 1000000; i++)
//         timerManager->AddAlarm(
//             std::chrono::milliseconds(0), i, std::string(), [i]() {});

//     ankerl::nanobench::Bench().epochs(epochnum).run(
//         "v3", [&q, &timerManager]() {
//             for (int i = 0; i < 1000000; i++)
//                 timerManager->DeleteAlarm(i, std::string());
//             // for (int i = 0; i < 1000000; i++)
//             //     timerManager->AddAlarm(
//             //         std::chrono::milliseconds(0), i, std::string(), [i]()
//             //         {});
//         });
//     timerManager->StopTimerManager();
// }

// TEST_CASE("TimerManager_ttt_v4")
// {
//     uint32_t epochnum = 1;
//     moodycamel::BlockingConcurrentQueue<UUID> q;
//     auto timerManager = timermanager::v4::TimerManager<int>::GetInstance();
//     // timerManager->StartTimerManager();

//     for (int i = 0; i < 1000000; i++)
//         timerManager->AddAlarm(
//             std::chrono::milliseconds(0), i, std::string(), [i]() {});

//     ankerl::nanobench::Bench().epochs(epochnum).run(
//         "v4", [&q, &timerManager]() {
//             for (int i = 0; i < 1000000; i++)
//                 timerManager->DeleteAlarm(i, std::string());
//             // for (int i = 0; i < 1000000; i++)
//             //     timerManager->AddAlarm(
//             //         std::chrono::milliseconds(0), i, std::string(), [i]()
//             //         {});
//         });
//     timerManager->StopTimerManager();
// }