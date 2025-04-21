#pragma once

#include <pthread.h>
#include <cerrno>

class SHMMutex
{
public:
    SHMMutex()
    {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_NORMAL);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
#ifdef __linux__
        pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
#endif
        pthread_mutex_init(&__mutex, &attr);
    }
    ~SHMMutex()
    {
        pthread_mutex_destroy(&__mutex);
    }
    void lock()
    {
        auto r = pthread_mutex_lock(&__mutex);
        if (r == EOWNERDEAD)
            r = pthread_mutex_consistent(&__mutex);
    }
    bool try_lock()
    {
        return pthread_mutex_trylock(&__mutex) == 0;
    }
    void unlock()
    {
        pthread_mutex_unlock(&__mutex);
    }
    pthread_mutex_t* GetMutexPtr()
    {
        return &__mutex;
    }

private:
    pthread_mutex_t __mutex;
};