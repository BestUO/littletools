#include <format>
#include <iostream>
#include <tuple>
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

TEST_CASE("fun_executor_macro_serialize v3")
{
    struct ParamBase
    {
        std::string function_name;
        int base;
        GEN_SERIALIZE(function_name, base)
    };
    struct MemberFuncParam
    {
        ParamBase param_base;
        int param;
        GEN_SERIALIZE(param_base, param)
    };
    struct MemberIntFuncReturn
    {
        ParamBase param_base;
        int result;
        GEN_SERIALIZE(param_base, result)
    };

    class TestClass
    {
    public:
        MemberIntFuncReturn MemberIntFunc1(MemberFuncParam p)
        {
            return MemberIntFuncReturn{{p.param_base}, p.param};
        }
        int MemberIntFunc2(ParamBase param_base, int param)
        {
            return param;
        }
    };
    funexecutor::v3::FunExecutor executor;
    TestClass obj;
    executor.RegisterFunctionWithSerialize(
        "MemberIntFunc1", [&obj](MemberFuncParam p) {
            return obj.MemberIntFunc1(p);
        });
    executor.RegisterFunctionWithSerialize(
        "MemberIntFunc2", [&obj](ParamBase param_base, int param) {
            return obj.MemberIntFunc2(param_base, param);
        });

    MemberFuncParam fun1_request_user_data{{"aaa", 12}, 42};
    funexecutor::v3::FunExecutor::ResquestOrResponse<MemberFuncParam>
        fun1_true_request{"MemberIntFunc1", fun1_request_user_data};
    std::string fun1_true_request_str = fun1_true_request.serialize();
    auto func1_return_str
        = executor.InvokeFunctionWithSerialize(fun1_true_request_str);
    funexecutor::v3::FunExecutor::ResquestOrResponse<MemberIntFuncReturn>
        func1_true_return;
    func1_true_return.deserialize(func1_return_str.c_str());
    CHECK(fun1_true_request.function_id == func1_true_return.function_id);
    auto fun1_reponse_user_data = std::get<0>(func1_true_return.user_data);
    CHECK(fun1_request_user_data.param_base.base
        == fun1_reponse_user_data.param_base.base);
    CHECK(fun1_request_user_data.param_base.function_name
        == fun1_reponse_user_data.param_base.function_name);
    CHECK(fun1_request_user_data.param == fun1_reponse_user_data.result);

    funexecutor::v3::FunExecutor::ResquestOrResponse<ParamBase, int>
        fun2_true_request{"MemberIntFunc2", {ParamBase{"bbb", 13}, 43}};

    std::string fun2_true_request_str = fun2_true_request.serialize();
    auto func2_true_response_str
        = executor.InvokeFunctionWithSerialize(fun2_true_request_str);
    funexecutor::v3::FunExecutor::ResquestOrResponse<int> func2_true_response;
    func2_true_response.deserialize(func2_true_response_str.c_str());
    CHECK(fun2_true_request.function_id == func2_true_response.function_id);
    auto fun2_response_user_data = std::get<0>(func2_true_response.user_data);
    CHECK(std::get<1>(fun2_true_request.user_data) == fun2_response_user_data);
}