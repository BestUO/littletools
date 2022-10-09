#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "../tools/timermanager.hpp"

TEST_CASE("testing TimerManager") 
{
    struct Key
    {
        std::string name;
        int id;
    };

    auto fun2 = [](int a, int b)
    {
        std::cout << a << '\t' << b << std::endl;
    };
    TimerManager<Key> *tm = TimerManager<Key>::GetInstance();
    for(int i=5;i<10;i++)
        tm->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(2), Key{"123",i}, std::bind(fun2,i,i+1));
    tm->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(2), Key{"123", 5}, std::bind(fun2,5,6));
    tm->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(2), Key{"123", 6}, std::bind(fun2,6,7));
    CHECK(tm->DeleteAlarm(std::bind([](const Key& t, int id){ return t.id == id; },std::placeholders::_1, 5)) == true );
    tm->AddAlarmInterval(std::chrono::system_clock::now()+std::chrono::seconds(2), Key{"run fun1interval",3}, std::bind(fun2,7,8),std::chrono::seconds(1));
    sleep(5);

    CHECK_MESSAGE(tm->DeleteAlarm(std::bind([](const Key& t, int id)
    {
        return t.id == id; 
    },std::placeholders::_1, 3)) == true,"可能再执行删除操作的时候正好从定时器取出，因此删除失败");

    sleep(2);
    tm->StopTimerManager();
}

// TEST_CASE("testLockQueue")
// {
//     struct point
//     {
//         int *a = nullptr;
//         std::string s = "aaaa";
//         point()
//         {
//             std::cout << "普通构造函数" << std::endl;
//         }

//         point(const point&other):a(other.a),s(other.s)
//         {
//             std::cout << "拷贝构造函数" << std::endl;
//         }
//         point(point &&other):a(other.a),s(std::move(other.s))
//         {
//             std::cout << "移动构造函数" << std::endl;
//             other.a = nullptr;
//         }
//         point& operator = (const point& other)
//         {
//             std::cout << "赋值构造函数" << std::endl;
//             a = other.a;
//             s = other.s;
//             return *this;
//         }
//         point& operator = (const point&& other)
//         {
//             std::cout << "移动赋值构造函数" << std::endl;
//             a = other.a;
//             s = std::move(other.s);
//             return *this;
//         }
//     };
//     LockQueue<point,100> q;
//     auto p1 = point();
//     p1.s = "aaaaa";
//     auto p2 = point();
//     p2.s = "bbbbb";
//     q.AddObj(std::move(p1));
//     q.AddObj(std::move(p2));
//     auto p = q.GetObj();

//     p1.s = "aaaaa";
//     p2.s = "bbbbb";
//     std::vector<point> v;
//     v.push_back(p1);
//     v.push_back(p2);
//     q.AddObjBulk(std::move(v));
//     auto bulk = q.GetObjBulk();
//     q.AddObj(std::move(p1));
//     std::cout << "end" << std::endl;
// }