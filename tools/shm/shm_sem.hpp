#pragma once

#include <string>
#include <stdexcept>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>

class SHMSemaphore
{
public:
    SHMSemaphore() = default;
    SHMSemaphore(SHMSemaphore&& other) noexcept
        : __named(other.__named)
        , __name(std::move(other.__name))
    {
        if (__named)
        {
            __sem_ptr       = other.__sem_ptr;
            other.__sem_ptr = nullptr;
        }
        else
        {
            memcpy(&__sem, &other.__sem, sizeof(sem_t));
            sem_init(&other.__sem, 0, 0);
        }
    }

    SHMSemaphore& operator=(SHMSemaphore&& other) noexcept
    {
        if (this != &other)
        {
            Cleanup();

            __named = other.__named;
            __name  = std::move(other.__name);

            if (__named)
            {
                __sem_ptr       = other.__sem_ptr;
                other.__sem_ptr = nullptr;
            }
            else
            {
                memcpy(&__sem, &other.__sem, sizeof(sem_t));
                sem_init(&other.__sem, 0, 0);
            }
        }
        return *this;
    }

    SHMSemaphore(const SHMSemaphore&)            = delete;
    SHMSemaphore& operator=(const SHMSemaphore&) = delete;
    ~SHMSemaphore()                              = default;

    void InitAnonymousSemaphore(unsigned int initial_value = 1)
    {
        __named = false;
        __name  = "";
        if (sem_init(&__sem, 0, initial_value) != 0)
        {
            throw std::runtime_error(
                std::string("SHMSemaphore init fail: ") + strerror(errno));
        }
    }

    void InitNamedSemaphore(const std::string& name,
        bool create                = true,
        unsigned int initial_value = 1)
    {
        __named = true;
        __name  = name;
        if (name.empty() || name[0] != '/')
        {
            throw std::invalid_argument("SHMSemaphore  name start with '/'");
        }

        if (create)
        {
            sem_t* result
                = sem_open(name.c_str(), O_CREAT | O_EXCL, 0666, initial_value);
            if (result == SEM_FAILED)
            {
                if (errno == EEXIST)
                {
                    result = sem_open(name.c_str(), 0);
                    if (result == SEM_FAILED)
                    {
                        throw std::runtime_error(
                            std::string("open existing sem fail: ")
                            + strerror(errno));
                    }
                }
                else
                {
                    throw std::runtime_error(
                        std::string("create sem fail: ") + strerror(errno));
                }
            }
            __sem_ptr = result;
        }
        else
        {
            __sem_ptr = sem_open(name.c_str(), 0);
            if (__sem_ptr == SEM_FAILED)
            {
                throw std::runtime_error(
                    std::string("open existing sem fail: ") + strerror(errno));
            }
        }
    }

    bool Wait(int timeout_ms = -1)
    {
        if (timeout_ms < 0)
        {
            return SemOperation([this]() {
                return __named ? sem_wait(__sem_ptr) : sem_wait(&__sem);
            });
        }
        else
        {
            struct timespec ts;
            if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
            {
                throw std::runtime_error(
                    std::string("get time fail: ") + strerror(errno));
            }

            ts.tv_sec += timeout_ms / 1000;
            ts.tv_nsec += (timeout_ms % 1000) * 1000000;
            if (ts.tv_nsec >= 1000000000)
            {
                ts.tv_sec += 1;
                ts.tv_nsec -= 1000000000;
            }

            return SemOperation([this, &ts]() {
                return __named ? sem_timedwait(__sem_ptr, &ts)
                               : sem_timedwait(&__sem, &ts);
            });
        }
    }

    bool TryWait()
    {
        return SemOperation([this]() {
            return __named ? sem_trywait(__sem_ptr) : sem_trywait(&__sem);
        });
    }

    bool Post()
    {
        return SemOperation([this]() {
            return __named ? sem_post(__sem_ptr) : sem_post(&__sem);
        });
    }

    int GetValue() const
    {
        int value;
        if (__named)
        {
            if (sem_getvalue(__sem_ptr, &value) != 0)
            {
                throw std::runtime_error(
                    std::string("get sem value fail: ") + strerror(errno));
            }
        }
        else
        {
            if (sem_getvalue(const_cast<sem_t*>(&__sem), &value) != 0)
            {
                throw std::runtime_error(
                    std::string("get sem value fail: ") + strerror(errno));
            }
        }
        return value;
    }

    static void Unlink(const std::string& name)
    {
        if (sem_unlink(name.c_str()) != 0)
        {
            throw std::runtime_error(
                std::string("unlink fail: ") + strerror(errno));
        }
    }

    std::string GetName() const
    {
        return __name;
    }

    bool IsNamed() const
    {
        return __named;
    }

    void Cleanup()
    {
        if (__named)
        {
            if (__sem_ptr != nullptr)
            {
                sem_close(__sem_ptr);
                Unlink(__name);
                __sem_ptr = nullptr;
            }
        }
        else
        {
            sem_destroy(&__sem);
        }
    }

private:
    template <typename Func>
    bool SemOperation(Func operation)
    {
        int result = operation();
        if (result == 0)
        {
            return true;
        }
        else
        {
            if (errno == ETIMEDOUT || errno == EAGAIN)
            {
                return false;
            }
            throw std::runtime_error(
                std::string("SemOperation fail: ") + strerror(errno));
        }
    }

    bool __named;
    std::string __name;
    union
    {
        sem_t __sem;
        sem_t* __sem_ptr;
    };
};