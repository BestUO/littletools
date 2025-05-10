#pragma once

#include <pthread.h>
#include "tools/shm/shm_mutex.hpp"

class SHMCondVar
{
public:
    SHMCondVar()
    {
        pthread_condattr_t attr;
        pthread_condattr_init(&attr);
        pthread_condattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
        pthread_cond_init(&__cond, &attr);
    }
    ~SHMCondVar()
    {
        pthread_cond_destroy(&__cond);
    }
    void wait(SHMMutex& lock)
    {
        pthread_cond_wait(&__cond, lock.GetMutexPtr());
    }
    void wait_for(SHMMutex& lock, const timespec& time)
    {
        pthread_cond_timedwait(&__cond, lock.GetMutexPtr(), &time);
    }
    void notify_one()
    {
        pthread_cond_signal(&__cond);
    }
    void notify_all()
    {
        pthread_cond_broadcast(&__cond);
    }

private:
    pthread_cond_t __cond;
};