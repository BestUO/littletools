#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/fun_executor/include/fun_executor.hpp"

void test1()
{
    printf("test1\n");
}

int test2(int a, double b)
{
    printf("test2: a=%d, b=%f\n", a, b);
    return a + (int)b;
}

std::string test3(const std::string& s, int& x, double& y)
{
    printf("test3: s=%s, x=%d, y=%f\n", s.c_str(), x, y);
    x += 1;
    return s + " world";
}

TEST_CASE("fun_executor")
{
    class TestClass
    {
    public:
        int memberFunc(int a)
        {
            printf("TestClass::memberFunc: a=%d\n", a);
            return a;
        }
    };

    FunExecutor executor;
    executor.RegisterFunction("test1", test1);
    executor.RegisterFunction("test2", test2);
    executor.RegisterFunction("test3", test3);

    executor.InvokeFunction<void>("test1");
    CHECK(13 == *executor.InvokeFunction<int>("test2", 10, 3.14));
    CHECK("hello world"
        == *executor.InvokeFunction<std::string>(
            "test3", std::string("hello"), 10, 2.71));

    TestClass obj;
    executor.RegisterFunction("memberFunc", [&obj](int a) {
        return obj.memberFunc(a);
    });
    CHECK(99 == *executor.InvokeFunction<int>("memberFunc", 99));
}