#include <format>
#include <iostream>
#include <utility>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/fun_executor/include/fun_executor.hpp"
#include "tools/serialize/serializer.hpp"
#include "tools/simple_serialize.hpp"

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

TEST_CASE("fun_executor_simple_serialize v3")
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

TEST_CASE("fun_executor_macro_serialize v3")
{
    struct ParamBase
    {
        std::string function_name;
        GEN_SERIALIZE_WITH_NAME(ParamBase, function_name)
    };
    struct ParamReturnBase
    {
        int base;
        GEN_SERIALIZE_WITH_NAME(ParamReturnBase, base)
    };
    struct MemberFuncParam
    {
        ParamBase param_base;
        int param;
        GEN_SERIALIZE_WITH_NAME(MemberFuncParam, param_base, param)
    };
    struct MemberIntFuncReturn
    {
        ParamReturnBase param_return_base;
        int result;
        GEN_SERIALIZE_WITH_NAME(MemberIntFuncReturn, param_return_base, result)
    };
    struct MemberStringFuncReturn
    {
        ParamReturnBase param_return_base;
        std::string result;
        GEN_SERIALIZE_WITH_NAME(MemberStringFuncReturn,
            param_return_base,
            result)
    };

    class TestClass
    {
    public:
        MemberIntFuncReturn MemberIntFunc1(MemberFuncParam p)
        {
            return MemberIntFuncReturn{{12}, 1};
        }
        MemberIntFuncReturn MemberIntFunc2(MemberFuncParam p)
        {
            return MemberIntFuncReturn{{13}, 2};
        }

        MemberStringFuncReturn MemberStringFunc(MemberFuncParam p)
        {
            return MemberStringFuncReturn{{14}, "hello"};
        }
    };
    funexecutor::v3::FunExecutor executor;
    TestClass obj;
    executor.RegisterFunctionWithSerialize(
        "MemberIntFunc1", [&obj](MemberFuncParam p) {
            return obj.MemberIntFunc1(p);
        });
    executor.RegisterFunctionWithSerialize(
        "MemberIntFunc2", [&obj](MemberFuncParam p) {
            return obj.MemberIntFunc2(p);
        });
    executor.RegisterFunctionWithSerialize(
        "MemberStringFunc", [&obj](MemberFuncParam p) {
            return obj.MemberStringFunc(p);
        });

    MemberFuncParam param_int_fun1{{.function_name = "MemberIntFunc1"}, 42};
    std::string param_int_fun1_serialize = param_int_fun1.serialize();
    auto param_base = ParamBase::FromBuffer(param_int_fun1_serialize.c_str());
    CHECK("MemberIntFunc1" == param_base.function_name);
    auto member_int_func1_return_serialize
        = executor.InvokeFunctionWithSerialize(
            param_base.function_name, param_int_fun1_serialize);
    auto member_int_func1_return = MemberIntFuncReturn::FromBuffer(
        member_int_func1_return_serialize.c_str());
    CHECK(1 == member_int_func1_return.result);
    CHECK(12 == member_int_func1_return.param_return_base.base);

    MemberFuncParam param_int_fun2{{.function_name = "MemberIntFunc2"}, 43};
    std::string param_int_fun2_serialize = param_int_fun2.serialize();
    auto param_base2 = ParamBase::FromBuffer(param_int_fun2_serialize.c_str());
    CHECK("MemberIntFunc2" == param_base2.function_name);
    auto member_int_func2_return_serialize
        = executor.InvokeFunctionWithSerialize(
            param_base2.function_name, param_int_fun2_serialize);
    auto member_int_func2_return = MemberIntFuncReturn::FromBuffer(
        member_int_func2_return_serialize.c_str());
    CHECK(2 == member_int_func2_return.result);
    CHECK(13 == member_int_func2_return.param_return_base.base);

    MemberFuncParam param_string_fun{{.function_name = "MemberStringFunc"}, 44};
    std::string param_string_fun_serialize = param_string_fun.serialize();
    auto param_base3
        = ParamBase::FromBuffer(param_string_fun_serialize.c_str());
    CHECK("MemberStringFunc" == param_base3.function_name);
    auto member_string_func_return_serialize
        = executor.InvokeFunctionWithSerialize(
            param_base3.function_name, param_string_fun_serialize);
    auto member_string_func_return = MemberStringFuncReturn::FromBuffer(
        member_string_func_return_serialize.c_str());
    CHECK("hello" == member_string_func_return.result);
    CHECK(14 == member_string_func_return.param_return_base.base);
}