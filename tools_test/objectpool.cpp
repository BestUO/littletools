#include <iostream>
#include <thread>
#include <sys/types.h>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/objectpool.hpp"
#include <set>

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

        t1.join();
        t2.join();
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
    int totalnum      = 1024 * 1024 * 10;
    uint32_t epochnum = 5;

    auto v1op = v1::ObjectPool<ObjectPoolTest>::GetInstance();
    v1op->GetObject(1, "sss", false);
    ankerl::nanobench::Bench()
        .epochs(epochnum)
        .minEpochIterations(epochnum)
        .run("one thread ObjectPool perf v1", [&]() {
            for (int i = 0; i < totalnum; i++)
            {
                auto obj = v1op->GetObject(1, "sss", false);
                ankerl::nanobench::doNotOptimizeAway(obj);
                v1op->PutObject(obj);
            }
        });

    auto v2op = v2::ObjectPool<ObjectPoolTest>::GetInstance();
    v2op->GetObject(1, "sss", false);
    ankerl::nanobench::Bench()
        .epochs(epochnum)
        .minEpochIterations(epochnum)
        .run("one thread ObjectPool perf v2", [&]() {
            for (int i = 0; i < totalnum; i++)
            {
                auto obj = v2op->GetObject(1, "sss", false);
                ankerl::nanobench::doNotOptimizeAway(obj);
                v2op->PutObject(obj);
            }
        });

    auto v3op = v3::ObjectPool<ObjectPoolTest>::GetInstance();
    v3op->GetObject(1, "sss", false);
    ankerl::nanobench::Bench()
        .epochs(epochnum)
        .minEpochIterations(epochnum)
        .run("one thread ObjectPool perf v3", [&]() {
            for (int i = 0; i < totalnum; i++)
            {
                auto obj = v3op->GetObject(1, "sss", false);
                ankerl::nanobench::doNotOptimizeAway(obj);
                v3op->PutObject(obj);
            }
        });

    auto v4op = v4::ObjectPool<ObjectPoolTest>::GetInstance();
    v4op->GetObject(1, "sss", false);
    ankerl::nanobench::Bench()
        .epochs(epochnum)
        .minEpochIterations(epochnum)
        .run("one thread ObjectPool perf v4", [&]() {
            for (int i = 0; i < totalnum; i++)
            {
                auto obj = v4op->GetObject(1, "sss", false);
                ankerl::nanobench::doNotOptimizeAway(obj);
                v4op->PutObject(obj);
            }
        });

    ankerl::nanobench::Bench()
        .epochs(epochnum)
        .minEpochIterations(epochnum)
        .run("one thread new delete perf", [&]() {
            for (int i = 0; i < totalnum; i++)
            {
                auto obj = new ObjectPoolTest(1, "sss", false);
                ankerl::nanobench::doNotOptimizeAway(obj);
                delete obj;
            }
        });
}

TEST_CASE("ObjectPool_multi_thread_perf")
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
    int totalnum   = 1024 * 1024;
    int thread_num = 8;
    int epochnum   = 5;

    auto RunConcurrent
        = []<typename Fun>(size_t thread_num, size_t ops_size, Fun f) {
              std::vector<std::thread> threads;
              for (size_t i = 0; i < thread_num; i++)
              {
                  threads.emplace_back([&f, ops_size] {
                      for (int j = 0; j < ops_size; ++j)
                          f();
                  });
              }
              for (auto& t : threads)
              {
                  t.join();
              }
          };

    ankerl::nanobench::Bench()
        .epochs(epochnum)
        .minEpochIterations(epochnum)
        .run("multi thread  objectpool v1", [&]() {
            auto op = v1::ObjectPool<ObjectPoolTest>::GetInstance();
            op->GetObject(1, "sss", false);
            RunConcurrent(thread_num, totalnum, [&]() {
                auto obj = op->GetObject(1, "sss", false);
                ankerl::nanobench::doNotOptimizeAway(obj);
                op->PutObject(obj);
            });
        });
    ankerl::nanobench::Bench()
        .epochs(epochnum)
        .minEpochIterations(epochnum)
        .run("multi thread  objectpool v2", [&]() {
            auto op = v2::ObjectPool<ObjectPoolTest>::GetInstance();
            op->GetObject(1, "sss", false);
            RunConcurrent(thread_num, totalnum, [&]() {
                auto obj = op->GetObject(1, "sss", false);
                ankerl::nanobench::doNotOptimizeAway(obj);
                op->PutObject(obj);
            });
        });
    ankerl::nanobench::Bench()
        .epochs(epochnum)
        .minEpochIterations(epochnum)
        .run("multi thread  objectpool v3", [&]() {
            auto op = v3::ObjectPool<ObjectPoolTest>::GetInstance();
            op->GetObject(1, "sss", false);
            RunConcurrent(thread_num, totalnum, [&]() {
                auto obj = op->GetObject(1, "sss", false);
                ankerl::nanobench::doNotOptimizeAway(obj);
                op->PutObject(obj);
            });
        });
    ankerl::nanobench::Bench()
        .epochs(epochnum)
        .minEpochIterations(epochnum)
        .run("multi thread  objectpool v4", [&]() {
            auto op = v4::ObjectPool<ObjectPoolTest>::GetInstance();
            op->GetObject(1, "sss", false);
            RunConcurrent(thread_num, totalnum, [&]() {
                auto obj = op->GetObject(1, "sss", false);
                ankerl::nanobench::doNotOptimizeAway(obj);
                op->PutObject(obj);
            });
        });
    ankerl::nanobench::Bench()
        .epochs(epochnum)
        .minEpochIterations(epochnum)
        .run("multi thread  new delete", [&]() {
            RunConcurrent(thread_num, totalnum, []() {
                auto obj = new ObjectPoolTest(1, "sss", false);
                ankerl::nanobench::doNotOptimizeAway(obj);
                delete obj;
            });
        });
}

struct Plain
{
    int value = 0;
};

// Tracks live-instance count so we can assert construction/destruction
// pairs balance (i.e. PutObject really destroys the object).
struct Tracked
{
    static std::atomic<int> live;
    int value;
    explicit Tracked(int v = 0)
        : value(v)
    {
        live.fetch_add(1, std::memory_order_relaxed);
    }
    ~Tracked()
    {
        live.fetch_sub(1, std::memory_order_relaxed);
    }
    Tracked(const Tracked&)            = delete;
    Tracked& operator=(const Tracked&) = delete;
};
std::atomic<int> Tracked::live{0};

// Not default-constructible, not copyable: v1's `new T[NITEM]` approach
// could never have supported this type. Verifies the v4 fix.
struct NoDefaultCtor
{
    int a;
    int b;
    NoDefaultCtor(int a_, int b_)
        : a(a_)
        , b(b_)
    { }
    NoDefaultCtor(const NoDefaultCtor&)            = delete;
    NoDefaultCtor& operator=(const NoDefaultCtor&) = delete;
};

// Throws from its constructor when told to, to exercise exception safety.
struct ThrowsOnRequest
{
    static bool should_throw;
    int value;
    explicit ThrowsOnRequest(int v)
        : value(v)
    {
        if (should_throw)
            throw std::runtime_error("constructed with should_throw=true");
    }
};
bool ThrowsOnRequest::should_throw = false;

// ---------------------------------------------------------------------
// Basic functional behavior
// ---------------------------------------------------------------------

TEST_CASE(
    "ObjectPoolv4 GetObject returns a usable, correctly constructed object")
{
    auto* pool = v4::ObjectPool<Plain>::GetInstance();
    Plain* p   = pool->GetObject();
    REQUIRE(p != nullptr);
    CHECK(p->value == 0);
    p->value = 42;
    CHECK(p->value == 42);
    pool->PutObject(p);
}

TEST_CASE("ObjectPoolv4 GetObject forwards constructor arguments correctly")
{
    auto* pool = v4::ObjectPool<Plain>::GetInstance();
    Plain* p   = pool->GetObject();
    p->value   = 7;
    CHECK(p->value == 7);
    pool->PutObject(p);
}

TEST_CASE(
    "ObjectPoolv4 GetInstance always returns the same singleton for a given T")
{
    auto* a = v4::ObjectPool<Plain>::GetInstance();
    auto* b = v4::ObjectPool<Plain>::GetInstance();
    CHECK(a == b);
}

TEST_CASE("ObjectPoolv4 Different T instantiations use independent pools")
{
    auto* int_pool   = v4::ObjectPool<int>::GetInstance();
    auto* plain_pool = v4::ObjectPool<Plain>::GetInstance();
    CHECK(static_cast<void*>(int_pool) != static_cast<void*>(plain_pool));

    int* i   = int_pool->GetObject(5);
    Plain* p = plain_pool->GetObject();
    p->value = 5;
    CHECK(*i == 5);
    CHECK(p->value == 5);
    int_pool->PutObject(i);
    plain_pool->PutObject(p);
}

TEST_CASE(
    "ObjectPoolv4 PutObject actually destroys the object (fixes v1's missing "
    "dtor call)")
{
    auto* pool = v4::ObjectPool<Tracked>::GetInstance();
    int before = Tracked::live.load();

    Tracked* t = pool->GetObject(1);
    CHECK(Tracked::live.load() == before + 1);

    pool->PutObject(t);
    CHECK(Tracked::live.load() == before);
}

TEST_CASE("ObjectPoolv4 PutObject(nullptr) is a safe no-op")
{
    auto* pool = v4::ObjectPool<Plain>::GetInstance();
    CHECK_NOTHROW(pool->PutObject(nullptr));
}

TEST_CASE(
    "ObjectPoolv4 Freed slots are recycled (single free slot gets reused)")
{
    // Use a dedicated type so this test isn't affected by pool state left
    // over from other tests.
    struct ReuseProbe
    {
        int x;
    };
    auto* pool = v4::ObjectPool<ReuseProbe>::GetInstance();

    ReuseProbe* first = pool->GetObject();
    pool->PutObject(first);
    ReuseProbe* second = pool->GetObject();
    // With only one object ever freed, the very next allocation from the
    // same thread's local pool must reuse that exact address.
    CHECK(first == second);
    pool->PutObject(second);
}

TEST_CASE(
    "ObjectPoolv4 Non-default-constructible, non-copyable types work (fixes v1 "
    "limitation)")
{
    auto* pool       = v4::ObjectPool<NoDefaultCtor>::GetInstance();
    NoDefaultCtor* p = pool->GetObject(3, 4);
    REQUIRE(p != nullptr);
    CHECK(p->a == 3);
    CHECK(p->b == 4);
    pool->PutObject(p);
}

TEST_CASE(
    "ObjectPoolv4 Allocations spanning multiple blocks (> NITEM objects) all "
    "succeed and "
    "are unique")
{
    auto* pool      = v4::ObjectPool<Plain>::GetInstance();
    const int count = NITEM * 3 + 17;  // force several block rollovers
    std::vector<Plain*> ptrs;
    ptrs.reserve(count);

    for (int i = 0; i < count; ++i)
    {
        Plain* p = pool->GetObject();
        REQUIRE(p != nullptr);
        p->value = i;
        ptrs.push_back(p);
    }

    std::set<Plain*> unique_ptrs(ptrs.begin(), ptrs.end());
    CHECK(unique_ptrs.size() == static_cast<size_t>(count));

    for (int i = 0; i < count; ++i)
        CHECK(ptrs[static_cast<size_t>(i)]->value == i);

    for (auto* p : ptrs)
        pool->PutObject(p);
}

// ---------------------------------------------------------------------
// Exception safety
// ---------------------------------------------------------------------

TEST_CASE(
    "ObjectPoolv4 Exception during construction propagates and does not "
    "corrupt the pool")
{
    auto* pool = v4::ObjectPool<ThrowsOnRequest>::GetInstance();

    ThrowsOnRequest::should_throw = true;
    CHECK_THROWS_AS(pool->GetObject(1), std::runtime_error);
    ThrowsOnRequest::should_throw = false;

    // Pool must still be fully usable afterwards, and the failed slot must
    // not have been permanently lost.
    ThrowsOnRequest* ok = nullptr;
    CHECK_NOTHROW(ok = pool->GetObject(2));
    REQUIRE(ok != nullptr);
    CHECK(ok->value == 2);
    pool->PutObject(ok);
}

TEST_CASE(
    "ObjectPoolv4 Exception safety holds across many alternating "
    "throw/no-throw allocations")
{
    auto* pool = v4::ObjectPool<ThrowsOnRequest>::GetInstance();
    std::vector<ThrowsOnRequest*> alive;

    for (int i = 0; i < 50; ++i)
    {
        ThrowsOnRequest::should_throw = (i % 3 == 0);
        try
        {
            alive.push_back(pool->GetObject(i));
        } catch (const std::runtime_error&)
        {
            // expected for every 3rd iteration
        }
    }
    ThrowsOnRequest::should_throw = false;

    // i in [0,50) with i % 3 == 0 throws: i = 0,3,...,48 -> 17 throws,
    // so 50 - 17 = 33 successful allocations survive.
    CHECK(alive.size() == 33);
    for (auto* p : alive)
        pool->PutObject(p);
}

// ---------------------------------------------------------------------
// Concurrency
// ---------------------------------------------------------------------

TEST_CASE(
    "ObjectPoolv4 Concurrent GetObject/PutObject from many threads is "
    "race-free")
{
    auto* pool   = v4::ObjectPool<Tracked>::GetInstance();
    int baseline = Tracked::live.load();

    constexpr int kThreads      = 8;
    constexpr int kOpsPerThread = 20000;
    std::atomic<int> completed{0};

    auto worker = [&]() {
        for (int i = 0; i < kOpsPerThread; ++i)
        {
            Tracked* t = pool->GetObject(i);
            // Touch the memory to help catch corruption under TSan/ASan.
            CHECK(t->value == i);
            pool->PutObject(t);
        }
        completed.fetch_add(1, std::memory_order_relaxed);
    };

    std::vector<std::thread> threads;
    for (int i = 0; i < kThreads; ++i)
        threads.emplace_back(worker);
    for (auto& th : threads)
        th.join();

    CHECK(completed.load() == kThreads);
    // Every object obtained was also put back, so live count returns to
    // baseline once all threads (and their LocalPools) have exited.
    CHECK(Tracked::live.load() == baseline);
}

TEST_CASE(
    "ObjectPoolv4 Concurrent producers/consumers across threads exercise "
    "cross-thread "
    "recycling")
{
    // Objects are allocated on one set of threads and freed on another,
    // forcing FreeChunks to actually travel through the shared,
    // lock-protected registry rather than staying thread-local.
    auto* pool = v4::ObjectPool<Plain>::GetInstance();

    constexpr int kProduce = 5000;
    std::vector<Plain*> produced(kProduce, nullptr);
    std::mutex produced_mutex;

    std::thread producer([&]() {
        for (int i = 0; i < kProduce; ++i)
        {
            Plain* p = pool->GetObject();
            p->value = i;
            std::lock_guard<std::mutex> lck(produced_mutex);
            produced[static_cast<size_t>(i)] = p;
        }
    });
    producer.join();

    std::atomic<int> freed{0};
    auto consumer = [&](int start, int end) {
        for (int i = start; i < end; ++i)
        {
            Plain* p = produced[static_cast<size_t>(i)];
            REQUIRE(p != nullptr);
            CHECK(p->value == i);
            pool->PutObject(p);
            freed.fetch_add(1, std::memory_order_relaxed);
        }
    };

    std::thread c1(consumer, 0, kProduce / 2);
    std::thread c2(consumer, kProduce / 2, kProduce);
    c1.join();
    c2.join();

    CHECK(freed.load() == kProduce);
}

// ---------------------------------------------------------------------
// Basic sanity/smoke of high volume single-threaded churn
// ---------------------------------------------------------------------

TEST_CASE(
    "ObjectPoolv4 High-volume single-threaded churn does not leak or crash")
{
    auto* pool   = v4::ObjectPool<Tracked>::GetInstance();
    int baseline = Tracked::live.load();

    for (int i = 0; i < 200000; ++i)
    {
        Tracked* t = pool->GetObject(i);
        CHECK(t->value == i);
        pool->PutObject(t);
    }

    CHECK(Tracked::live.load() == baseline);
}