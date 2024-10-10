#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <unistd.h>
#include <sched.h>

class CPUBind
{
public:
    static int GetCpuCount()
    {
        return sysconf(_SC_NPROCESSORS_ONLN);
    }

    static void BindCPU()
    {
        num = (num + 1) % cpu_num;
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(num, &mask);
        if (sched_setaffinity(0, sizeof(mask), &mask) == -1)
        {
            printf("Could not set CPU affinity\n");
        }
        CheckBindCPU();
    }

    static void CheckBindCPU()
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        if (sched_getaffinity(0, sizeof(mask), &mask) == -1)
        {
            printf("Could not get CPU affinity\n");
        }
        for (int i = 0; i < cpu_num; i++)
        {
            if (CPU_ISSET(i, &mask))
            {
                printf("CPU %d is binded\n", i);
            }
        }
    }

    static void BindCPU(int cpu)
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(cpu, &mask);
        if (sched_setaffinity(0, sizeof(mask), &mask) == -1)
        {
            printf("Could not set CPU affinity\n");
        }
    }

    static void BindCPU(pid_t pid, int cpu)
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(cpu, &mask);
        if (sched_setaffinity(pid, sizeof(mask), &mask) == -1)
        {
            printf("Could not set CPU affinity\n");
        }
    }

private:
    static int num;
    static int cpu_num;
};

inline int CPUBind::num     = 0;
inline int CPUBind::cpu_num = GetCpuCount();