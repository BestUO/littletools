#include <format>
#include <iostream>
#include <utility>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/fun_executor/include/fun_executor.hpp"

void test1()
{ }

int test2(int a, double b)
{
    return a + (int)b;
}

std::string test3(const std::string& s, int& x, double& y)
{
    x += 1;
    return s + " world";
}

void test4(int a)
{ }

TEST_CASE("fun_executor v1")
{
    class TestClass
    {
    public:
        int memberFunc(int a)
        {
            return a;
        }
    };

    funexecutor::v1::FunExecutor executor;
    executor.RegisterFunction("test1", test1);
    executor.InvokeFunction<void>("test1");

    executor.RegisterFunction("test2", test2);
    CHECK(13 == *executor.InvokeFunction<int>("test2", 10, 3.14));

    executor.RegisterFunction("test3", test3);
    CHECK("hello world"
        == *executor.InvokeFunction<std::string>(
            "test3", std::string("hello"), 10, 2.71));

    executor.RegisterFunction("test4", test4);
    executor.InvokeFunction<void>("test4", 42);

    TestClass obj;
    executor.RegisterFunction("memberFunc", [&obj](int a) {
        return obj.memberFunc(a);
    });
    CHECK(99 == *executor.InvokeFunction<int>("memberFunc", 99));
}

TEST_CASE("fun_executor v2")
{
    class TestClass
    {
    public:
        std::string memberFunc(const std::string& str)
        {
            return str;
        }
    };

    funexecutor::v2::FunExecutor executor;
    executor.RegisterFunction("test1", test1);
    executor.InvokeFunction<void>("test1");

    executor.RegisterFunction("test2", test2);
    CHECK(13 == *executor.InvokeFunction<int>("test2", 10, 3.14));

    executor.RegisterFunction("test3", test3);
    CHECK("hello world"
        == *executor.InvokeFunction<std::string>(
            "test3", std::string("hello"), 10, 2.71));

    executor.RegisterFunction("test4", test4);
    executor.InvokeFunction<void>("test4", 42);

    TestClass obj;
    executor.RegisterFunction("memberFunc", [&obj](const std::string& a) {
        return obj.memberFunc(std::forward<const std::string&>(a));
    });
    CHECK("hello"
        == *executor.InvokeFunction<std::string>("memberFunc", "hello"));
}