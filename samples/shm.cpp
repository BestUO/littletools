#include <cstdlib>
#include <iostream>
#include <thread>
#include "tools/shm_component.hpp"
struct TestStruct
{
    int b;
};
class Service1
{
public:
    void testMutexNormal()
    {
        SHMFactory<MutexLock> mutex("my_mutex");
        {
            std::lock_guard<MutexLock> lck(*mutex);
            printf("sleep 3s then free lock\n");
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
        printf("lock free\n");
        std::this_thread::sleep_for(std::chrono::seconds(10000));
    }

    void testMutexRobust()
    {
        SHMFactory<MutexLock> mutex("my_mutex");
        {
            std::lock_guard<MutexLock> lck(*mutex);
            printf("sleep 3s then free lock\n");
            std::this_thread::sleep_for(std::chrono::seconds(10));
            exit(0);
        }
        printf("should not print this\n");
        std::this_thread::sleep_for(std::chrono::seconds(10000));
    }

    void testcondVar()
    {
        SHMFactory<CondVar> cond_var("my_cond");
        SHMFactory<MutexLock> mutex("my_mutex");
        {
            std::lock_guard<MutexLock> lck(*mutex);
            printf("wait signal\n");
            cond_var->wait(*mutex);
            printf("get signal\n");
        }
        printf("lock free\n");
        std::this_thread::sleep_for(std::chrono::seconds(10000));
    }

    void testSHMMempool()
    {
        SHMFactory<SHMV2::SHMMemoryPool<TestStruct, 1>> pool("MemoryPool");
        int32_t times = 1024 * 1024;
        while (true)
        {
            if (times <= 0)
                break;

            auto [index, s_ptr] = pool->Allocate();
            if (index.__index != -1)
            {
                sleep(1);
                times--;
                s_ptr->b = times;
                pool->Free(index);
                printf("server1 %d\n", s_ptr->b);
            }
        }
    }

    void testSHMMsgQueue()
    {
        int32_t times = 1024 * 1024;
        SHMFactory<SHMV2::SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
        while (true)
        {
            if (times <= 0)
                break;

            auto [index, s_ptr] = queue->Allocate();
            if (index != -1)
            {
                times--;
                s_ptr->b = times;
                queue->SendMsg(index);
            }
        }
    }
};

class Service2
{
public:
    void testMutexNormal()
    {
        SHMFactory<MutexLock> mutex("my_mutex");
        {
            printf("wait service1 free lock\n");
            std::lock_guard<MutexLock> lck(*mutex);
            printf("get lock\n");
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        printf("lock free\n");
        std::this_thread::sleep_for(std::chrono::seconds(10000));
    }

    void testMutexRobust()
    {
        SHMFactory<MutexLock> mutex("my_mutex");
        {
            printf("wait service1 exit\n");
            std::lock_guard<MutexLock> lck(*mutex);
            printf("get lock\n");
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        printf("lock free\n");
        std::this_thread::sleep_for(std::chrono::seconds(10000));
    }

    void testcondVar()
    {
        SHMFactory<CondVar> cond_var("my_cond");
        SHMFactory<MutexLock> mutex("my_mutex");
        {
            std::lock_guard<MutexLock> lck(*mutex);
            printf("signal\n");
            cond_var->notify_all();
            printf("signal OK\n");
        }
        std::this_thread::sleep_for(std::chrono::seconds(10000));
    }

    void testcondVarRobust()
    {
        SHMFactory<CondVar> cond_var("my_cond");
        SHMFactory<MutexLock> mutex("my_mutex");
        {
            std::lock_guard<MutexLock> lck(*mutex);
            printf("signal\n");
            exit(0);
        }
        std::this_thread::sleep_for(std::chrono::seconds(10000));
    }

    void testSHMMempool()
    {
        SHMFactory<SHMV2::SHMMemoryPool<TestStruct, 1>> pool("MemoryPool");
        int32_t times = 1024 * 1024;
        while (true)
        {
            if (times <= 0)
                break;

            auto [index, s_ptr] = pool->Allocate();
            if (index.__index != -1)
            {
                times--;
                printf("%d\n", s_ptr->b);
                pool->Free(index);
            }
        }
    }

    void testSHMMsgQueue()
    {
        int32_t times = 1024 * 1024;
        SHMFactory<SHMV2::SHMMsgQueue<TestStruct, 1024>> queue("SHMMsgQueue");
        while (true)
        {
            if (times <= 0)
                break;

            auto [index, s_ptr] = queue->RecvMsg();
            if (index != -1)
            {
                times--;
                queue->Free(index);
            }
        }
    }
};

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <service_type>"
                  << " <fun_index>" << std::endl;
        std::cerr << "service_type=1 need start first" << std::endl;
        std::cerr << "<fun_index> 0: Lock normal" << std::endl;
        std::cerr << "<fun_index> 1: exit befor unlock" << std::endl;
        std::cerr << "<fun_index> 2: condVar normal" << std::endl;
        std::cerr << "<fun_index> 3: exit befor condVar notify" << std::endl;
        std::cerr << "<fun_index> 4: test SHMMempool" << std::endl;
        std::cerr << "<fun_index> 5: test SHMMsgQueue" << std::endl;
        return 1;
    }
    auto service_type = std::stoi(argv[1]);
    auto fun_index    = std::stoi(argv[2]);
    std::cout << "service_type:" << service_type << " fun_index:" << fun_index
              << std::endl;
    Service1 service1;
    Service2 service2;
    switch (fun_index)
    {
    case 0:
        if (service_type == 1)
            service1.testMutexNormal();
        else if (service_type == 2)
            service2.testMutexNormal();
        break;
    case 1:
        if (service_type == 1)
            service1.testMutexRobust();
        else if (service_type == 2)
            service2.testMutexRobust();
        break;
    case 2:
        if (service_type == 1)
            service1.testcondVar();
        else if (service_type == 2)
            service2.testcondVar();
        break;
    case 3:
        if (service_type == 1)
            service1.testcondVar();
        else if (service_type == 2)
            service2.testcondVarRobust();
        break;
    case 4:
        if (service_type == 1)
            service1.testSHMMempool();
        else if (service_type == 2)
            service2.testSHMMempool();
        break;
    case 5:
        if (service_type == 1)
            service1.testSHMMsgQueue();
        else if (service_type == 2)
            service2.testSHMMsgQueue();
        break;
    }
    return 0;
}