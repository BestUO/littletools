#include <iostream>
#include <thread>
#include <sys/types.h>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/objectpool.hpp"
#include "queue/threadsafecontainer.hpp"

TEST_CASE("ObjectPool_test")
{
    struct ObjectPoolTest
    {
        int a            = 2;
        std::string b    = "c";
        bool c           = true;
        ObjectPoolTest() = default;
        ObjectPoolTest(int a, std::string b, bool c)
            : a(a)
            , b(b)
            , c(c){};
    };

    auto op = v1::ObjectPool<ObjectPoolTest>::GetInstance();
    {
        ObjectPoolTest* ptr[5];
        for (int i = 0; i < 5; i++)
            ptr[i] = op->GetObject(1, "sss", false);
        for (int i = 0; i < 5; i++)
            op->PutObject(ptr[i]);
        for (int i = 0; i < 5; i++)
            ptr[i] = op->GetObject(1, "sss", false);
        ;
        for (int i = 0; i < 5; i++)
            op->PutObject(ptr[i]);
    }
    {
        auto obj = op->GetObject(1, "sss", false);
        CHECK(obj->a == 1);
        CHECK(obj->b == "sss");
        CHECK(obj->c == false);
        obj = op->GetObject(3, "tt", true);
        CHECK(obj->a == 3);
        CHECK(obj->b == "tt");
        CHECK(obj->c == true);
        op->PutObject(obj);
    }
    {
        int size = 1000000;
        std::thread t1([&op, &size]() {
            ObjectPoolTest* ptr[size];
            for (int i = 0; i < size; i++)
                ptr[i] = op->GetObject(1, "sss", false);
            for (int i = 0; i < size; i++)
                op->PutObject(ptr[i]);
        });

        std::thread t2([&op, &size]() {
            ObjectPoolTest* ptr[size];
            for (int i = 0; i < size; i++)
                ptr[i] = op->GetObject(1, "sss", false);
            for (int i = 0; i < size; i++)
                op->PutObject(ptr[i]);
        });

        LockQueue<ObjectPoolTest*> lq;
        std::thread t3([&op, &size, &lq]() {
            ObjectPoolTest* ptr[size];
            for (int i = 0; i < size; i++)
            {
                while (!lq.AddObj(op->GetObject(1, "sss", false)))
                    ;
            }
        });
        std::thread t4([&op, &size, &lq]() {
            for (int i = 0; i < size; i++)
            {
                auto opt = lq.GetObj();
                while (opt == std::nullopt)
                    opt = lq.GetObj();
                op->PutObject(opt.value());
            }
        });

        t1.join();
        t2.join();
        t3.join();
        t4.join();
    }
}

TEST_CASE("ObjectPool_one_thread_perf")
{
    struct ObjectPoolTest
    {
        int a            = 2;
        std::string b    = "c";
        bool c           = true;
        ObjectPoolTest() = default;
        ObjectPoolTest(int a, std::string b, bool c)
            : a(a)
            , b(b)
            , c(c){};
    };
    int totalnum      = 10000000;
    int runningnum    = 2000;
    uint32_t epochnum = 10;

    auto v1op = v1::ObjectPool<ObjectPoolTest>::GetInstance();
    v1op->GetObject(1, "sss", false);
    ankerl::nanobench::Bench().epochs(epochnum).run(
        "one thread ObjectPool perf v1", [&]() {
            for (int i = 0; i < totalnum; i++)
            {
                auto obj = v1op->GetObject(1, "sss", false);
                v1op->PutObject(obj);
            }
        });

    auto v2op = v2::ObjectPool<ObjectPoolTest>::GetInstance();
    v2op->GetObject(1, "sss", false);
    ankerl::nanobench::Bench().epochs(epochnum).run(
        "one thread ObjectPool perf v2", [&]() {
            for (int i = 0; i < totalnum; i++)
            {
                auto obj = v2op->GetObject(1, "sss", false);
                v2op->PutObject(obj);
            }
        });

    ankerl::nanobench::Bench().epochs(epochnum).run(
        "one thread new delete perf", [&]() {
            for (int i = 0; i < totalnum; i++)
            {
                auto obj = new ObjectPoolTest(1, "sss", false);
                delete obj;
            }
        });
}

TEST_CASE("ObjectPool_two_thread_perf")
{
    struct ObjectPoolTest
    {
        int a            = 2;
        std::string b    = "c";
        bool c           = true;
        ObjectPoolTest() = default;
        ObjectPoolTest(int a, std::string b, bool c)
            : a(a)
            , b(b)
            , c(c){};
    };
    int totalnum      = 10000000;
    int runningnum    = 2000;
    uint32_t epochnum = 10;

    auto v2op = v2::ObjectPool<ObjectPoolTest>::GetInstance();
    v2op->GetObject(1, "sss", false);
    ankerl::nanobench::Bench().epochs(1).run(
        "two thread 1000w objectpool v2", [&totalnum, &runningnum, v2op]() {
            std::thread t1([&totalnum, &runningnum, v2op]() {
                ObjectPoolTest* ptr[runningnum];
                int j = 0;
                for (int i = 0; i < totalnum; i++)
                {
                    ptr[j] = v2op->GetObject(1, "sss", false);
                    j++;
                    if (j == runningnum)
                    {
                        for (int k = 0; k < j; k++)
                        {
                            v2op->PutObject(ptr[k]);
                        }
                        j = 0;
                    }
                }
            });
            std::thread t2([&totalnum, &runningnum]() {
                auto v2op = v2::ObjectPool<ObjectPoolTest>::GetInstance();
                v2op->GetObject(1, "sss", false);
                ObjectPoolTest* ptr[runningnum];
                int j = 0;
                for (int i = 0; i < totalnum; i++)
                {
                    ptr[j] = v2op->GetObject(1, "sss", false);
                    j++;
                    if (j == runningnum)
                    {
                        for (int k = 0; k < j; k++)
                        {
                            v2op->PutObject(ptr[k]);
                        }
                        j = 0;
                    }
                }
            });
            t1.join();
            t2.join();
        });
    ankerl::nanobench::Bench().epochs(1).run(
        "two thread 1000w new delete", [&totalnum, &runningnum]() {
            std::thread t1([&totalnum, &runningnum]() {
                ObjectPoolTest* ptr[runningnum];
                int j = 0;
                for (int i = 0; i < totalnum; i++)
                {
                    ptr[j] = new ObjectPoolTest(1, "sss", false);
                    j++;
                    if (j == runningnum)
                    {
                        for (int k = 0; k < j; k++)
                        {
                            delete ptr[k];
                        }
                        j = 0;
                    }
                }
            });
            std::thread t2([&totalnum, &runningnum]() {
                ObjectPoolTest* ptr[runningnum];
                int j = 0;
                for (int i = 0; i < totalnum; i++)
                {
                    ptr[j] = new ObjectPoolTest(1, "sss", false);
                    j++;
                    if (j == runningnum)
                    {
                        for (int k = 0; k < j; k++)
                        {
                            delete ptr[k];
                        }
                        j = 0;
                    }
                }
            });
            t1.join();
            t2.join();
        });
}