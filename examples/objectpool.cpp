#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/objectpool.hpp"
#include "queue/threadsafecontainer.hpp"

TEST_CASE("ObjectPool test")
{
    struct ObjectPoolTest
    {
        int a         = 2;
        std::string b = "c";
        bool c        = true;
        // ObjectPoolTest()=default;
        ObjectPoolTest(int a, std::string b, bool c)
            : a(a)
            , b(b)
            , c(c){};
    };

    auto op = ObjectPool<ObjectPoolTest>::GetInstance();
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
