#pragma once

#include <cstddef>
#include <ctime>
#include <string>
#include <fcntl.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include "tools/shm/shm_sem.hpp"

// template <typename T>
// class SHMFactory
// {
// public:
//     SHMFactory(std::string&& name)
//         : name(std::move(name))
//     {
//         bool is_create = true;
//         __fd = shm_open(name.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
//         if (__fd >= 0)
//         {
//             if (ftruncate(__fd, sizeof(T)) == -1)
//                 perror("ftruncate failed");
//         }
//         else if (errno == EEXIST)
//         {
//             __fd      = shm_open(name.c_str(), O_RDWR, 0666);
//             is_create = false;
//         }

//         auto ptr = (T*)mmap(
//             NULL, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, __fd, 0);
//         if (is_create)
//         {
//             __ptr = new (ptr) T();
//         }
//         else
//         {
//             __ptr = reinterpret_cast<T*>(ptr);
//         }
//     }
//     ~SHMFactory()
//     {
//         close(__fd);
//         munmap(&__ptr, sizeof(T));
//         shm_unlink(name.c_str());
//     }
//     T* operator->() const
//     {
//         return __ptr;
//     }

//     T& operator*() const
//     {
//         return *__ptr;
//     }

// private:
//     int __fd;
//     T* __ptr;
//     std::string name = "";
// };

template <size_t N>
class SHMFactoryMeta
{
public:
    void Init()
    {
        SetLocalPid();
        __shm_size = N;
    }

    bool AnyAlive()
    {
        bool any_alive = false;
        for (size_t i = 0; i < 64; i++)
        {
            if (__pid[i] == 0)
            {
                continue;
            }
            else
            {
                any_alive |= !IsPidDead(__pid[i]);
            }
        }
        return any_alive;
    }

    bool Check(size_t size)
    {
        if (__shm_size == size && AnyAlive())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void ClearLocalPid()
    {
        auto pid = getpid();
        for (size_t i = 0; i < 64; i++)
        {
            if (__pid[i] == pid)
            {
                __pid[i] = 0;
                break;
            }
        }
    }

    void SetLocalPid()
    {
        for (size_t i = 0; i < 64; i++)
        {
            if (__pid[i] == 0)
            {
                __pid[i] = getpid();
                break;
            }
        }
    }

private:
    pid_t __pid[64]   = {0};
    size_t __shm_size = 0;
    bool IsPidDead(pid_t tid)
    {
        return kill(tid, 0) != 0;
    }
};

template <typename T>
class SHMFactory
{
public:
    SHMFactory(const std::string& name)
        : __name(name)
    {
        if (init(name) == false)
        {
            init(name);
        }
    }

    bool init(const std::string& name)
    {
        __shm_sem.InitNamedSemaphore("/" + name + "_factory_sem", true);
        if (__shm_sem.Wait(50))
        {
            bool is_create = true;
            __fd = shm_open(name.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
            if (__fd >= 0)
            {
                if (ftruncate(
                        __fd, sizeof(T) + sizeof(SHMFactoryMeta<sizeof(T)>))
                    == -1)
                {
                    return false;
                }
            }
            else if (errno == EEXIST)
            {
                __fd      = shm_open(name.c_str(), O_RDWR, 0666);
                is_create = false;
            }
            else
            {
                return false;
            }

            bool result = false;
            auto ptr    = (T*)mmap(NULL,
                sizeof(T) + sizeof(SHMFactoryMeta<sizeof(T)>),
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                __fd,
                0);
            if (is_create)
            {
                __ptr      = new (ptr) T();
                __meta_ptr = reinterpret_cast<SHMFactoryMeta<sizeof(T)>*>(
                    reinterpret_cast<char*>(ptr) + sizeof(T));
                __meta_ptr->Init();
                result = true;
            }
            else
            {
                __ptr      = reinterpret_cast<T*>(ptr);
                __meta_ptr = reinterpret_cast<SHMFactoryMeta<sizeof(T)>*>(
                    reinterpret_cast<char*>(ptr) + sizeof(T));
                if (__meta_ptr->Check(sizeof(T)))
                {
                    __meta_ptr->SetLocalPid();
                    result = true;
                }
                else
                {
                    munmap(
                        __ptr, sizeof(T) + sizeof(SHMFactoryMeta<sizeof(T)>));
                    close(__fd);
                    shm_unlink(name.c_str());
                    result = false;
                }
            }
            __shm_sem.Post();
            return result;
        }
        else
        {
            return false;
        }
    }

    ~SHMFactory()
    {
        if (__shm_sem.Wait(50))
        {
            __meta_ptr->ClearLocalPid();
            if (!__meta_ptr->AnyAlive())
            {
                munmap(__ptr, sizeof(T) + sizeof(SHMFactoryMeta<sizeof(T)>));
                close(__fd);
                shm_unlink(__name.c_str());
                __shm_sem.Cleanup();
            }
            else
            {
                __shm_sem.Post();
            }
        }
    }
    T* operator->() const
    {
        return __ptr;
    }

    T& operator*() const
    {
        return *__ptr;
    }

private:
    int __fd;
    T* __ptr;
    SHMSemaphore __shm_sem;
    SHMFactoryMeta<sizeof(T)>* __meta_ptr = nullptr;
    std::string __name;
};