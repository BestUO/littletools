#include <chrono>
#include <random>
#include <iostream>
#include <future>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/timermanager.hpp"

TEST_CASE("TimerManager_v1_base" * doctest::skip())
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

TEST_CASE("TimerManager_v1_interval" * doctest::skip())
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

TEST_CASE("TimerManager_v1_two_same_key" * doctest::skip())
{
    struct Key
    {
        std::string name;
        int id;
    };

    auto fun2 = [](int a, int b) {
        std::cout << "TimerManager_v1_two_same_key a=" << a << '\t' << "b=" << b
                  << std::endl;
    };
    auto tm = timermanager::v1::TimerManager<Key>::GetInstance();
    tm->AddAlarm(
        std::chrono::milliseconds(2), Key{"123", 6}, std::bind(fun2, 8, 8));
    tm->AddAlarm(
        std::chrono::milliseconds(4), Key{"123", 6}, std::bind(fun2, 8, 8));
    sleep(1);
    tm->StopTimerManager();
}

TEST_CASE("TimerManager_v2" * doctest::skip())
{
    auto func = [](int a, int b) {
        std::cout << "TimerManager_v2 a=" << a << '\t' << "b=" << b
                  << std::endl;
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

TEST_CASE("TimerManager_v2_interval" * doctest::skip())
{
    auto func = [](int a, int b) {
        std::cout << "TimerManager_v2_interval a=" << a << '\t' << "b=" << b
                  << std::endl;
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

TEST_CASE("TimerManager_v2_10w" * doctest::skip())
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
            {
            }
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

TEST_CASE("TimerManager_v3" * doctest::skip())
{
    auto func = [](int a, int b) {
        std::cout << "TimerManager_v3 a=" << a << '\t' << "b=" << b
                  << std::endl;
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

TEST_CASE("TimerManager_v3_recursive" * doctest::skip())
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
            std::cout << "TimerManager_v3_recursive callback()" << std::endl;
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

TEST_CASE("TimerManager_v4_base" * doctest::skip())
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

TEST_CASE("TimerManager_v4_recursive" * doctest::skip())
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
            std::cout << "TimerManager_v4_recursive callback()" << std::endl;
        }
        void fun()
        {
            timermanager::v4::TimerManager<int>::GetInstance()->AddAlarm(
                std::chrono::milliseconds(200), 1, "1-2", [this]() {
                    callback();
                });
        }
    };
    {
        auto a = std::make_shared<A>();
        timermanager::v4::TimerManager<int>::GetInstance()->AddAlarm(
            std::chrono::milliseconds(0),
            2,
            "",
            [a]() {
                a->fun();
            },
            std::chrono::milliseconds(100));
        sleep(1);
    }
    timerManager->StopTimerManager();
}

TEST_CASE("TimerManager_v4_benchmark" * doctest::skip())
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

TEST_CASE("TimerManager_v6_functional")
{
    using namespace std::chrono_literals;

    auto& timer_manager = timermanager::v6::TimerManager::GetInstance();
    timer_manager.StartTimerManager();
    std::atomic<int> one_shot_count{0};
    std::atomic<int> cancelled_count{0};
    std::atomic<int> repeating_count{0};
    std::atomic<timermanager::v6::TimerManager::TimerId> repeating_id{0};

    timer_manager.AddTimer(2ms, [&] {
        ++one_shot_count;
    });
    const auto cancelled_id = timer_manager.AddTimer(20ms, [&] {
        ++cancelled_count;
    });
    CHECK(timer_manager.CancelTimer(cancelled_id));

    const auto timer_id = timer_manager.AddTimer(
        1ms,
        [&] {
            if (++repeating_count == 3)
                timer_manager.CancelTimer(repeating_id.load());
        },
        1ms);
    repeating_id.store(timer_id);

    std::this_thread::sleep_for(20ms);
    CHECK_EQ(one_shot_count.load(), 1);
    CHECK_EQ(cancelled_count.load(), 0);
    CHECK_EQ(repeating_count.load(), 3);
    timer_manager.StopTimerManager();
}

TEST_CASE("TimerManager_v6_benchmark")
{
    using namespace std::chrono_literals;

    constexpr int timer_count = 100000;
    auto& timer_manager       = timermanager::v6::TimerManager::GetInstance();
    timer_manager.StartTimerManager();
    std::atomic<int> fired_count{0};
    std::vector<timermanager::v6::TimerManager::TimerId> timer_ids;
    timer_ids.reserve(timer_count);

    ankerl::nanobench::Bench().epochs(1).run("TimerManager_v6_add", [&] {
        for (int i = 0; i < timer_count; ++i)
            timer_ids.push_back(timer_manager.AddTimer(1s, [&] {
                ++fired_count;
            }));
    });

    ankerl::nanobench::Bench().epochs(1).run("TimerManager_v6_cancel", [&] {
        for (const auto timer_id : timer_ids)
            timer_manager.CancelTimer(timer_id);
    });
    CHECK_EQ(fired_count.load(), 0);
    timer_manager.StopTimerManager();
}

TEST_CASE("TimerManager_v6_functional_matrix")
{
    using namespace std::chrono_literals;
    auto& manager = timermanager::v6::TimerManager::GetInstance();
    manager.StartTimerManager();

    std::atomic<int> fired{0};
    CHECK(manager.AddTimer(100ms, [&] {
        ++fired;
    }) != 0);
    std::this_thread::sleep_for(130ms);
    CHECK_EQ(fired.load(), 1);

    const auto one_shot = manager.AddTimer(1ms, [&] {
        ++fired;
    });
    std::this_thread::sleep_for(20ms);
    CHECK_FALSE(manager.CancelTimer(one_shot));
    CHECK_FALSE(manager.CancelTimer(999999));

    const auto cancelled = manager.AddTimer(100ms, [&] {
        ++fired;
    });
    CHECK(manager.CancelTimer(cancelled));
    CHECK_FALSE(manager.CancelTimer(cancelled));

    std::atomic<int> periodic{0};
    std::atomic<timermanager::v6::TimerManager::TimerId> periodic_id{0};
    const auto id = manager.AddTimer(
        0ms,
        [&] {
            if (++periodic == 3)
                CHECK(manager.CancelTimer(periodic_id.load()));
        },
        10ms);
    periodic_id.store(id);
    std::this_thread::sleep_for(80ms);
    CHECK_EQ(periodic.load(), 3);
    CHECK_FALSE(manager.CancelTimer(id));

    std::atomic<int> simultaneous{0};
    for (int i = 0; i < 16; ++i)
        manager.AddTimer(10ms, [&] {
            ++simultaneous;
        });
    std::this_thread::sleep_for(40ms);
    CHECK_EQ(simultaneous.load(), 16);
    manager.StopTimerManager();
}

TEST_CASE("TimerManager_v6_lifecycle_and_reentrancy")
{
    using namespace std::chrono_literals;
    auto& manager = timermanager::v6::TimerManager::GetInstance();
    manager.StopTimerManager();
    std::atomic<int> fired{0};
    CHECK_EQ(manager.AddTimer(1ms,
                 [&] {
                     ++fired;
                 }),
        0);
    CHECK_FALSE(manager.CancelTimer(1));
    manager.StartTimerManager();
    manager.StartTimerManager();

    std::promise<void> nested_done;
    auto nested_future = nested_done.get_future();
    manager.AddTimer(1ms, [&] {
        manager.AddTimer(1ms, [&] {
            ++fired;
            nested_done.set_value();
        });
    });
    CHECK_EQ(nested_future.wait_for(100ms), std::future_status::ready);
    manager.StopTimerManager();
    manager.StopTimerManager();
    CHECK_EQ(manager.AddTimer(1ms,
                 [&] {
                     ++fired;
                 }),
        0);
    manager.StartTimerManager();
    manager.StopTimerManager();
}

TEST_CASE("TimerManager_v6_singleton")
{
    constexpr int thread_count = 16;
    std::vector<const timermanager::v6::TimerManager*> instances(thread_count);
    std::vector<std::thread> threads;
    threads.reserve(thread_count);
    for (int i = 0; i < thread_count; ++i)
        threads.emplace_back([&, i] {
            instances[i] = &timermanager::v6::TimerManager::GetInstance();
        });
    for (auto& thread : threads)
        thread.join();
    for (const auto* instance : instances)
        CHECK_EQ(instance, instances.front());
}

TEST_CASE("TimerManager_v6_exception_inputs")
{
    auto& manager = timermanager::v6::TimerManager::GetInstance();
    manager.StartTimerManager();
    CHECK_EQ(manager.AddTimer(std::chrono::milliseconds(100), nullptr), 0);
    const auto huge = manager.AddTimer(std::chrono::hours(24 * 365), [] {});
    CHECK_NE(huge, 0);
    CHECK(manager.CancelTimer(huge));
    manager.StopTimerManager();
}

TEST_CASE("TimerManager_v6_cancel_race")
{
    using namespace std::chrono_literals;
    auto& manager = timermanager::v6::TimerManager::GetInstance();
    manager.StartTimerManager();
    std::atomic<int> fired{0};
    const auto id = manager.AddTimer(5ms, [&] {
        ++fired;
    });
    std::vector<std::thread> threads;
    std::atomic<int> successful_cancels{0};
    for (int i = 0; i < 8; ++i)
        threads.emplace_back([&] {
            if (manager.CancelTimer(id))
                ++successful_cancels;
        });
    for (auto& thread : threads)
        thread.join();
    std::this_thread::sleep_for(20ms);
    CHECK(successful_cancels.load() <= 1);
    CHECK(fired.load() <= 1);
    manager.StopTimerManager();
}

TEST_CASE("TimerManager_v6_concurrent_stop_add_cancel")
{
    using namespace std::chrono_literals;
    auto& manager = timermanager::v6::TimerManager::GetInstance();
    manager.StartTimerManager();
    std::atomic<bool> running{true};
    std::thread producer([&] {
        while (running.load())
        {
            const auto id = manager.AddTimer(1s, [] {});
            if (id != 0)
                manager.CancelTimer(id);
        }
    });
    std::this_thread::sleep_for(10ms);
    manager.StopTimerManager();
    running.store(false);
    producer.join();
    CHECK_EQ(manager.AddTimer(1ms, [] {}), 0);
}

TEST_CASE("TimerManager_v6_concurrency_performance")
{
    using namespace std::chrono_literals;
    constexpr int thread_count      = 8;
    constexpr int timers_per_thread = 12500;
    auto& manager = timermanager::v6::TimerManager::GetInstance();
    manager.StartTimerManager();
    std::vector<std::vector<timermanager::v6::TimerManager::TimerId>> ids(
        thread_count);
    std::vector<std::thread> threads;
    const auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < thread_count; ++i)
        threads.emplace_back([&, i] {
            ids[i].reserve(timers_per_thread);
            for (int j = 0; j < timers_per_thread; ++j)
                ids[i].push_back(manager.AddTimer(1min, [] {}));
        });
    for (auto& thread : threads)
        thread.join();
    const auto add_elapsed
        = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - begin);
    for (const auto& group : ids)
        for (const auto id : group)
            manager.CancelTimer(id);
    std::cout << "TimerManager_v6 concurrent add: " << add_elapsed.count()
              << " ms\n";
    manager.StopTimerManager();
}
