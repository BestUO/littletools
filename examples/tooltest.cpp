#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#define ANKERL_NANOBENCH_IMPLEMENT
#include "doctest/doctest.h"
#include "nanobench.h"

#include "tools/timermanager.hpp"
#include "tools/TimerManager.hpp"
#include "tools/lrucache.hpp"
#include "tools/threadpool.hpp"
#include "tools/objectpool.hpp"

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
    tp2.EnqueueStr([]() {
        std::cout << "tp2test" << std::endl;
        return 1;
    });
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

    auto fun2 = [](int a, int b) {
        std::cout << a << '\t' << b << std::endl;
    };
    TimerManager<Key>* tm = TimerManager<Key>::GetInstance();
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

TEST_CASE("testing TimerManager AddAlarmInterval DeleteAlarm")
{
    struct Key
    {
        std::string name;
        int id;
    };

    auto fun2 = [](int a, int b) {
        std::cout << a << '\t' << b << std::endl;
    };
    TimerManager<Key>* tm = TimerManager<Key>::GetInstance();

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

TEST_CASE("testing TimerManager two same key")
{
    struct Key
    {
        std::string name;
        int id;
    };

    auto fun2 = [](int a, int b) {
        std::cout << a << '\t' << b << std::endl;
    };
    TimerManager<Key>* tm = TimerManager<Key>::GetInstance();
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

TEST_CASE("testLockQueue")
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
    std::cout << "end" << std::endl;
}

TEST_CASE("LRUCache")
{
    LRUCache<int, std::string> cache(1, 20);
    cache.StoreKeyValue(1, "1");
    CHECK(cache.GetValue(1).value() == "1");
    cache.StoreKeyValue(1, "2");
    cache.StoreKeyValue(3, "3");
    CHECK(cache.GetValue(1).value() == "2");
    CHECK(cache.GetValue(2) == std::nullopt);
}

TEST_CASE("RBTreeWrap")
{
    struct TestStruct
    {
        std::string key = "83";
        int age         = 10;
        int operator<(const TestStruct& t) const
        {
            return age < t.age;
        }
        bool operator==(const TestStruct& t) const
        {
            return key == t.key;
        }
    };
    RBTreeWrap<TestStruct> tree;
    TestStruct val1 = {"67", 1};
    TestStruct val2 = {"1338", 5};
    TestStruct val3 = {"1337", 5};
    TestStruct val4 = {"83", 10};
    TestStruct val5 = {"84", 11};

    tree.AddObj(val5);
    tree.AddObj(val1);
    tree.AddObj(val2);
    tree.AddObj(val3);
    tree.AddObj(val4);

    auto t = tree.GetTopObjPtr();
    REQUIRE_EQ(t->age, val1.age);

    tree.PopTopObj();
    t = tree.GetTopObjPtr();
    REQUIRE_EQ(t->age, val2.age);

    tree.DeleteObj(*t);
    t = tree.GetTopObjPtr();
    REQUIRE_EQ(t->age, val3.age);

    t = tree.SearchObj(val4);
    REQUIRE_EQ(t->age, val4.age);
    t = tree.SearchObj([key = "84"](const TestStruct& b) {
        return key == b.key;
    });
    REQUIRE_EQ(t->age, val5.age);
}

TEST_CASE("TimerManager")
{
    auto timerManager = Boray::TimerManager<std::string>::GetInstance();
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
    Boray::TimerManager<std::string>::GetInstance()->StopTimerManager();
}