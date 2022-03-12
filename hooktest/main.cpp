#include "hook.h"
#include "thirdso.h"
#include <iostream>

int main()
{
    thirdso test;
    std::cout << "-------------use normalsleep" << std::endl;
    std::cout << test.mystrcmp("test1") << std::endl;

    // initHook();
    // std::cout << "-------------use initHook" << std::endl;
    // std::cout << test.mystrcmp("test1") << std::endl;
    return 0;
}