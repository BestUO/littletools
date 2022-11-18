#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#define ANKERL_NANOBENCH_IMPLEMENT
#include "doctest/doctest.h"
#include "nanobench.h"

#include "tools/timermanager.hpp"
#include "tools/lrucache.hpp"
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
    tp2.EnqueueStr([](){std::cout << "tp2test" << std::endl;return 1;});
    sleep(1);
    tp2.StopThreadPool();

    ThreadPool<FreeLockRingQueue<std::string>> tp3(1);
    tp3.EnqueueStr("aa");
    sleep(1);
    tp3.StopThreadPool();
}

TEST_CASE("testing TimerManager AddAlarm DeleteAlarm") 
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
    ankerl::nanobench::Bench().run("DeleteAlarm cost time",[&]()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    });
    CHECK(tm->DeleteAlarm(std::bind([](const Key& t, int id){ return t.id == id; },std::placeholders::_1, 7)) == true );
    sleep(3);
    tm->StopTimerManager();
}

TEST_CASE("testing TimerManager AddAlarmInterval DeleteAlarm") 
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

    tm->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(2), Key{"run fun1interval",3}, std::bind(fun2,7,8),std::chrono::seconds(1));
    sleep(5);

    CHECK_MESSAGE(tm->DeleteAlarm(std::bind([](const Key& t, int id)
    {
        return t.id == id; 
    },std::placeholders::_1, 3)) == true,"可能再执行删除操作的时候正好从定时器取出，因此删除失败");

    sleep(2);
    tm->StopTimerManager();
}

TEST_CASE("testing TimerManager two same key") 
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
    tm->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(2), Key{"123", 6}, std::bind(fun2,8,8));
    tm->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(4), Key{"123", 6}, std::bind(fun2,8,8));
    CHECK(tm->DeleteAlarm(std::bind([](const Key& t, int id){ return t.id == id; },std::placeholders::_1, 6)) == true );
    sleep(6);
    tm->StopTimerManager();
}

TEST_CASE("testLockQueue")
{
    struct point
    {
        int *a = nullptr;
        std::string s = "aaaa";
        point()
        {
            std::cout << "普通构造函数" << std::endl;
        }

        point(const point&other):a(other.a),s(other.s)
        {
            std::cout << "拷贝构造函数" << std::endl;
        }
        point(point &&other):a(other.a),s(std::move(other.s))
        {
            std::cout << "移动构造函数" << std::endl;
            other.a = nullptr;
        }
        point& operator = (const point& other)
        {
            std::cout << "赋值构造函数" << std::endl;
            a = other.a;
            s = other.s;
            return *this;
        }
        point& operator = (const point&& other)
        {
            std::cout << "移动赋值构造函数" << std::endl;
            a = other.a;
            s = std::move(other.s);
            return *this;
        }
    };
    LockQueue<point,100> q;
    auto p1 = point();
    p1.s = "aaaaa";
    auto p2 = point();
    p2.s = "bbbbb";
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
    std::cout << "end" << std::endl;
}

TEST_CASE("LRUCache")
{
    LRUCache<int,std::string> cache(1,20);
    cache.StoreKeyValue(1,"1");
    CHECK(cache.GetValue(1).value() == "1");
    cache.StoreKeyValue(1,"2");
    cache.StoreKeyValue(3,"3");
    CHECK(cache.GetValue(1).value() == "2");
    CHECK(cache.GetValue(2) == std::nullopt);
}