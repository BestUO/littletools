#include "hook.h"
#include "thirdso.h"
#include <iostream>

int main()
{
    std::cout << "-------------Hook sleep" << std::endl;
    thirdso test;
    initHook();
    std::cout << test.mystrcmp("test1") << std::endl;
    return 0;
}