#include <cstdlib>
#include <iostream>
#include <thread>
#include "tools/shm_component.hpp"

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
};

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <service_type>"
                  << " <fun_index>" << std::endl;
        std::cerr << "service_type=1 need start first" << std::endl;
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
    }
    return 0;
}