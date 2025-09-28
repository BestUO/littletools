#pragma once
#include <functional>
#include <tuple>

template <typename T>
struct function_traits;

template <typename ReturnType, typename... Args>
struct function_traits<ReturnType (*)(Args...)>
{
    using return_type = ReturnType;
    using arguments   = std::tuple<Args...>;
};

template <typename ReturnType, typename... Args>
struct function_traits<std::function<ReturnType(Args...)>>
{
    using return_type = ReturnType;
    using arguments   = std::tuple<Args...>;
};

template <typename Callable>
struct function_traits : function_traits<decltype(&Callable::operator())>
{ };

template <typename ReturnType, typename ClassType, typename... Args>
struct function_traits<ReturnType (ClassType::*)(Args...) const>
{
    using return_type = ReturnType;
    using arguments   = std::tuple<Args...>;
};