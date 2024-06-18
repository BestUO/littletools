#include <cstdint>
#include <cstring>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <memory>
#include <mutex>
#include <thread>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/shm_component.hpp"

TEST_CASE("shm_cv")
{
    shm_unlink("cond_var");
    shm_unlink("mutex");
    std::thread t2([]() {
        SHMFactory<CondVar> cond_var("cond_var");
        SHMFactory<MutexLock> mutex("mutex");
        std::lock_guard<MutexLock> lck(*mutex);
        std::cout << "wait" << std::endl;
        cond_var->wait(*mutex);
        std::cout << "wait OK" << std::endl;
    });
    sleep(1);
    std::thread t1([]() {
        SHMFactory<CondVar> cond_var("cond_var");
        cond_var->notify_one();
        std::cout << "signal OK" << std::endl;
    });
    t2.join();
    t1.join();
}

TEST_CASE("shm_SHMSharedObj")
{
    struct A
    {
        int b;
    };
    SHMObj<A> a;
    SHMSharedObj<A> obj(&a, nullptr);
    {
        CHECK(1 == obj.GetCount());
        SHMSharedObj<A> obj2 = obj;
        CHECK(2 == obj.GetCount());
        CHECK(2 == obj2.GetCount());
        SHMSharedObj<A> obj3;
        obj3 = obj;
        CHECK(3 == obj.GetCount());
        CHECK(3 == obj2.GetCount());
        CHECK(3 == obj3.GetCount());
    }
    CHECK(1 == obj.GetCount());
}

TEST_CASE("shm_SHMMemoryPool_special_type")
{
    struct A
    {
        int b;
    };
    A a;
    uint16_t index = -1;
    SHMMemoryPool<A> pool;
    {
        auto obj                     = pool.Allocate();
        obj.GetSHMObj()->GetData().b = 10;
        index                        = obj.GetIndex();
        CHECK(1 == obj.GetCount());
        auto obj2 = pool.GetSharedObject(index);
        CHECK(10 == obj2.GetSHMObj()->GetData().b);
        CHECK(2 == obj.GetCount());
        CHECK(2 == obj2.GetCount());
    }
    auto obj3 = pool.GetObjectForce(index);
    CHECK(0 == obj3->GetCount());
}

TEST_CASE("shm_SHMMemoryPool_size_type")
{
    uint16_t index = -1;
    SHMMemoryPool<char, 10> pool;
    {
        auto obj = pool.Allocate();
        memcpy(obj.GetSHMObj()->GetData(), "123", 3);
        index = obj.GetIndex();
        CHECK(1 == obj.GetCount());
        auto obj2 = pool.GetSharedObject(index);
        CHECK(0 == memcmp(obj2.GetSHMObj()->GetData(), "123", 3));
        CHECK(2 == obj.GetCount());
        CHECK(2 == obj2.GetCount());
    }
    auto obj3 = pool.GetObjectForce(index);
    CHECK(0 == obj3->GetCount());
}

TEST_CASE("shm_SHMMessageQueue_base")
{
    shm_unlink("SHMMessageQueue");
    struct A
    {
        int b;
    };
    std::thread t2([]() {
        SHMFactory<SHMMessageQueue<A>> queue("SHMMessageQueue");
        queue->GetMutex().lock();
        queue->GetCondVar().wait(queue->GetMutex());
        auto [flag, a] = queue->PopFront();
        queue->GetMutex().unlock();
        CHECK(1 == a.b);
    });
    sleep(1);
    std::thread t1([]() {
        SHMFactory<SHMMessageQueue<A>> queue("SHMMessageQueue");
        queue->GetMutex().lock();
        queue->PushBack({1});
        queue->GetMutex().unlock();
        queue->GetCondVar().notify_one();
    });

    t2.join();
    t1.join();
}

TEST_CASE("shm_SHMMessageQueue_crazy")
{
    shm_unlink("SHMMessageQueue");
    struct A
    {
        int b;
    };
    int total = 1000000;
    std::thread t2([total = total]() {
        SHMFactory<SHMMessageQueue<A>> queue("SHMMessageQueue");
        while (true)
        {
            queue->GetMutex().lock();
            auto [flag, a] = queue->PopFront();
            while (!flag)
            {
                queue->GetCondVar().wait(queue->GetMutex());
                std::tie(flag, a) = queue->PopFront();
            }
            queue->GetMutex().unlock();
            queue->GetCondVar().notify_one();
            if (a.b == total)
                break;
        }
    });
    sleep(1);
    std::thread t1([total = total]() {
        SHMFactory<SHMMessageQueue<A>> queue("SHMMessageQueue");
        int num = 0;
        while (num++ < total)
        {
            queue->GetMutex().lock();
            auto flag = queue->PushBack({num});
            while (!flag)
            {
                queue->GetCondVar().wait(queue->GetMutex());
                flag = queue->PushBack({num});
            }
            queue->GetMutex().unlock();
            queue->GetCondVar().notify_one();
        }
    });

    t2.join();
    t1.join();
}

TEST_CASE("shm_SHMMemoryPool_SHMMessageQueue_special_type")
{
    shm_unlink("MemoryPool");
    shm_unlink("SHMMessageQueue");
    struct A
    {
        int b;
    };

    SHMFactory<SHMMemoryPool<A>> pool("MemoryPool");
    SHMFactory<SHMMessageQueue<SHMMemoryPool<A>::MPE, true>> queue(
        "SHMMessageQueue");
    {
        auto sharedobj                     = pool->Allocate();
        sharedobj.GetSHMObj()->GetData().b = 10;
        queue->PushBackSharedObj(sharedobj);
        CHECK(2 == sharedobj.GetCount());
    }
    uint16_t index;
    {
        auto [flag, a] = queue->PopFront(*pool);
        CHECK(10 == a.GetSHMObj()->GetData().b);
        index = a.GetIndex();
        CHECK(1 == a.GetCount());
    }

    auto obj3 = pool->GetObjectForce(index);
    CHECK(0 == obj3->GetCount());
}

TEST_CASE("shm_SHMMemoryPool_SHMMessageQueue_size_type_base")
{
    shm_unlink("MemoryPool");
    shm_unlink("SHMMessageQueue");
    SHMFactory<SHMMemoryPool<char, 10>> pool("MemoryPool");
    SHMFactory<SHMMessageQueue<SHMMemoryPool<char, 10>::MPE, true>> queue(
        "SHMMessageQueue");
    {
        auto sharedobj = pool->Allocate();
        memcpy(sharedobj.GetSHMObj()->GetData(), "123", 3);
        queue->PushBackSharedObj(sharedobj);
        CHECK(2 == sharedobj.GetCount());
    }
    uint16_t index;
    {
        auto [flag, sharedobj] = queue->PopFront(*pool);
        CHECK(0 == memcmp(sharedobj.GetSHMObj()->GetData(), "123", 3));
        index = sharedobj.GetIndex();
        CHECK(1 == sharedobj.GetCount());
    }

    auto obj3 = pool->GetObjectForce(index);
    CHECK(0 == obj3->GetCount());
}

TEST_CASE("shm_SHMMemoryPool_SHMMessageQueue_size_type_crazy")
{
    shm_unlink("MemoryPool");
    shm_unlink("SHMMessageQueue");
    int total = 1000000;
    std::thread t2([total = total]() {
        SHMFactory<SHMMemoryPool<char, 1000>> pool("MemoryPool");
        SHMFactory<SHMMessageQueue<SHMMemoryPool<char, 10>::MPE, true>> queue(
            "SHMMessageQueue");
        int total = 1000000;
        int num   = 0;
        auto data = std::to_string(total);
        while (num++ < total)
        {
            queue->GetMutex().lock();
            auto [flag, sharedobj] = queue->PopFront(*pool);
            while (!flag)
            {
                queue->GetCondVar().wait(queue->GetMutex());
                std::tie(flag, sharedobj) = queue->PopFront(*pool);
            }
            queue->GetMutex().unlock();
            queue->GetCondVar().notify_one();
            if (0
                == memcmp(sharedobj.GetSHMObj()->GetData(),
                    data.c_str(),
                    data.length()))
                break;
        }
    });
    sleep(1);
    std::thread t1([total = total]() {
        SHMFactory<SHMMemoryPool<char, 1000>> pool("MemoryPool");
        SHMFactory<SHMMessageQueue<SHMMemoryPool<char, 10>::MPE, true>> queue(
            "SHMMessageQueue");

        int num = 0;
        while (num++ < total)
        {
            auto sharedobj = pool->Allocate();
            while (!sharedobj.GetSHMObj())
            {
                pool->Wait();
                sharedobj = pool->Allocate();
            }
            auto data = std::to_string(num);
            memcpy(
                sharedobj.GetSHMObj()->GetData(), data.c_str(), data.length());
            queue->GetMutex().lock();
            auto flag = queue->PushBackSharedObj(sharedobj);
            while (!flag)
            {
                queue->GetCondVar().wait(queue->GetMutex());
                flag = queue->PushBackSharedObj(sharedobj);
            }
            queue->GetMutex().unlock();
            queue->GetCondVar().notify_one();
        }
    });
    t2.join();
    t1.join();
}