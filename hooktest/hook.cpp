#include "hook.h"
#include <dlfcn.h>
#include <stdio.h>

extern "C" 
{
    sleep_t sleep_f = NULL;
}

static int doInitHook()
{
    sleep_f = (sleep_t)dlsym(RTLD_NEXT, "sleep");
    return 0;
}

void initHook()
{
    static int isInit = doInitHook();
    (void)isInit;
}

unsigned int sleep(unsigned int seconds)
{
    if(!sleep_f)
        doInitHook();
    printf("oops!!! hack function invoked\n");
    return sleep_f(seconds);
}
