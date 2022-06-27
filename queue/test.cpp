#include "test.h"      
#include <iostream>

void Test::p()
{
    std::cout << "test" << std::endl;
}

int main()
{
    Test a;
    a.p();
}