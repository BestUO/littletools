#include <format>
#include <iostream>
#include <utility>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/fun_executor/include/fun_executor.hpp"
#include "tools/serialize/serializer.hpp"

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

TEST_CASE("fun_executor v3")
{
    class TestClass
    {
    public:
        int MemberIntFunc(int a)
        {
            return a;
        }

        std::string MemberStringFunc(const std::string& str)
        {
            return str;
        }
    };

    funexecutor::v3::FunExecutor executor;
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
    executor.RegisterFunction("MemberIntFunc", [&obj](int a) {
        return obj.MemberIntFunc(a);
    });
    CHECK(99 == *executor.InvokeFunction<int>("MemberIntFunc", 99));

    executor.RegisterFunction("MemberStringFunc", [&obj](const std::string& a) {
        return obj.MemberStringFunc(std::forward<const std::string&>(a));
    });
    CHECK("hello"
        == *executor.InvokeFunction<std::string>("MemberStringFunc", "hello"));
}

TEST_CASE("fun_executor v3")
{
    struct MemberIntFuncParam
    {
        int param;
    };
    struct MemberIntFuncReturn
    {
        int res;
    };
    struct MemberStringFuncParam
    {
        std::string param;
    };
    struct MemberStringFuncReturn
    {
        std::string res;
    };
    class TestClass
    {
    public:
        MemberIntFuncReturn MemberIntFunc(MemberIntFuncParam a)
        {
            return MemberIntFuncReturn{a.param};
        }

        MemberStringFuncReturn MemberStringFunc(MemberStringFuncParam a)
        {
            return MemberStringFuncReturn{a.param};
        }
    };

    MemberIntFuncParam param_int{567};
    MemberStringFuncParam param_str{"hello world"};

    std::string int_serialize
        = serialize::SimpleSerializer::Serialize(param_int);
    std::string str_serialize
        = serialize::SimpleSerializer::Serialize(param_str);

    funexecutor::v3::FunExecutor executor;
    TestClass obj;

    executor.RegisterFunctionWithSerialize(
        "MemberIntFunc", [&obj](MemberIntFuncParam a) {
            return obj.MemberIntFunc(a);
        });
    CHECK(567
        == serialize::SimpleSerializer::Deserialize<MemberIntFuncReturn>(
            executor.InvokeFunctionWithSerialize(
                "MemberIntFunc", int_serialize))
               .res);

    executor.RegisterFunctionWithSerialize(
        "MemberStringFunc", [&obj](MemberStringFuncParam a) {
            return obj.MemberStringFunc(a);
        });
    CHECK("hello world"
        == serialize::SimpleSerializer::Deserialize<MemberStringFuncReturn>(
            executor.InvokeFunctionWithSerialize(
                "MemberStringFunc", str_serialize))
               .res);
}