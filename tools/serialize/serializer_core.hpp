#pragma once
#include <sys/types.h>
#include <cstddef>
#include <iostream>
#include <type_traits>

#include "concept_template.hpp"
#include "serializer_type.hpp"

namespace serialize
{

struct UniversalType
{
    template <typename T>
    operator T();
};

template <typename... Ts>
struct make_void
{
    typedef void type;
};

template <typename... Ts>
using my_void_t = typename make_void<Ts...>::type;

template <typename T,
    typename construct_param_t,
    typename = void,
    typename... Args>
struct IsConstructableImpl : std::false_type
{ };
template <typename T, typename construct_param_t, typename... Args>
struct IsConstructableImpl<T,
    construct_param_t,
    std::void_t<decltype(T{{Args{}}..., {construct_param_t{}}})>,
    Args...> : std::true_type
{ };

template <typename T, typename construct_param_t, typename... Args>
constexpr bool IsConstructable
    = IsConstructableImpl<T, construct_param_t, void, Args...>::value;

// template <typename T,
//     typename... Args,
//     std::enable_if_t<!IsConstructable<T, UniversalType, Args...>, int> a =
//     1>
// constexpr std::size_t MembersCountImpl()
// {
//     return sizeof...(Args);
// }

// template <typename T,
//     typename... Args,
//     std::enable_if_t<IsConstructable<T, UniversalType, Args...>, int> a = 1>
// constexpr std::size_t MembersCountImpl()
// {
//     return MembersCountImpl<T, Args..., UniversalType>();
// }

template <typename T, typename... Args>
constexpr std::size_t MembersCountImpl()
{
    if constexpr (IsConstructable<T, UniversalType, Args...>)
    {
        return MembersCountImpl<T, Args..., UniversalType>();
    }
    else
    {
        return sizeof...(Args);
    }
}

template <typename T, typename F>
constexpr decltype(auto) VisitorStructMember(T&& t, F&& f)
{
    using type           = std::remove_cvref_t<decltype(t)>;
    constexpr size_t num = MembersCountImpl<type>();
    if constexpr (num == 0)
    {
        // auto&& [a, b] = t;
        return f();
    }
    else if constexpr (num == 1)
    {
        auto&& [a1] = t;
        return f(a1);
    }
    else if constexpr (num == 2)
    {
        auto&& [a1, a2] = t;
        return f(a1, a2);
    }
    else if constexpr (num == 3)
    {
        auto&& [a1, a2, a3] = t;
        return f(a1, a2, a3);
    }
    else if constexpr (num == 4)
    {
        auto&& [a1, a2, a3, a4] = t;
        return f(a1, a2, a3, a4);
    }
    else if constexpr (num == 5)
    {
        auto&& [a1, a2, a3, a4, a5] = t;
        return f(a1, a2, a3, a4, a5);
    }
    else if constexpr (num == 6)
    {
        auto&& [a1, a2, a3, a4, a5, a6] = t;
        return f(a1, a2, a3, a4, a5, a6);
    }
    // else if constexpr (num == 7)
    // {
    //     auto&& [a1, a2, a3, a4, a5, a6, a7] = t;
    // }
    // else if constexpr (num == 8)
    // {
    //     auto&& [a1, a2, a3, a4, a5, a6, a7, a8] = t;
    // }
    // else if constexpr (num == 9)
    // {
    //     auto&& [a1, a2, a3, a4, a5, a6, a7, a8, a9] = t;
    // }
    // else if constexpr (num == 0)
    // {
    //     auto&& [a1, a2, a3, a4, a5, a6, a7, a8, a9, a10] = t;
    // }
    else
    {
        return "";
    }
}
}  // namespace serialize