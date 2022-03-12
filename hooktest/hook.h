#pragma once
#include <unistd.h>
#include <type_traits>
#include <utility>

extern "C" 
{
    typedef unsigned int(*sleep_t)(unsigned int seconds);
    extern sleep_t sleep_f;
}
extern void initHook();