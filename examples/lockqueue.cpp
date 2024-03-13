#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#define ANKERL_NANOBENCH_IMPLEMENT
#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "queue/threadsafecontainer.hpp"

TEST_CASE("LockQueue")
{
    struct point
    {
        int* a        = nullptr;
        std::string s = "aaaa";
        point()
        {
            std::cout << "普通构造函数" << std::endl;
        }

        point(const point& other)
            : a(other.a)
            , s(other.s)
        {
            std::cout << "拷贝构造函数" << std::endl;
        }
        point(point&& other)
            : a(other.a)
            , s(std::move(other.s))
        {
            std::cout << "移动构造函数" << std::endl;
            other.a = nullptr;
        }
        point& operator=(const point& other)
        {
            std::cout << "赋值构造函数" << std::endl;
            a = other.a;
            s = other.s;
            return *this;
        }
        point& operator=(const point&& other)
        {
            std::cout << "移动赋值构造函数" << std::endl;
            a = other.a;
            s = std::move(other.s);
            return *this;
        }
    };
    LockQueue<point, 100> q;
    auto p1 = point();
    p1.s    = "aaaaa";
    auto p2 = point();
    p2.s    = "bbbbb";
    CHECK(q.AddObj(std::move(p1)) == true);
    CHECK(q.AddObj(std::move(p2)) == true);
    // CHECK(q.GetObj() != nullptr);

    p1.s = "aaaaa";
    p2.s = "bbbbb";
    std::vector<point> v;
    v.push_back(p1);
    v.push_back(p2);
    q.AddObjBulk(std::move(v));
    auto bulk = q.GetObjBulk();
    q.AddObj(std::move(p1));
}