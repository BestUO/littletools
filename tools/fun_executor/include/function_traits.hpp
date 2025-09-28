#pragma once
#include <functional>
#include <tuple>

// 普通函数.
// 函数指针.
// function/lambda.
// 成员函数.
// 函数对象.

// 转换为 std::function 和函数指针.
namespace function_traits
{
namespace v1
{

template <typename T>
struct function_traits;

// 普通函数.
template <typename Ret, typename... Args>
struct function_traits<Ret(Args...)>
{
public:
    enum
    {
        arity = sizeof...(Args)
    };
    typedef Ret function_type(Args...);
    typedef Ret return_type;
    using stl_function_type = std::function<function_type>;
    typedef Ret (*pointer)(Args...);

    template <size_t I>
    struct args
    {
        static_assert(I < arity,
            "index is out of range, index must less than sizeof Args");
        using type = typename std::tuple_element<I, std::tuple<Args...>>::type;
    };

    typedef std::tuple<std::remove_cv_t<std::remove_reference_t<Args>>...>
        tuple_type;
    typedef std::tuple<std::remove_const_t<std::remove_reference_t<Args>>...>
        bare_tuple_type;
};

// 函数指针.
template <typename Ret, typename... Args>
struct function_traits<Ret (*)(Args...)> : function_traits<Ret(Args...)>
{ };

// std::function.
template <typename Ret, typename... Args>
struct function_traits<std::function<Ret(Args...)>>
    : function_traits<Ret(Args...)>
{ };

// member function.
#define FUNCTION_TRAITS(...)                                               \
    template <typename ReturnType, typename ClassType, typename... Args>   \
    struct function_traits<ReturnType (ClassType::*)(Args...) __VA_ARGS__> \
        : function_traits<ReturnType(Args...)>                             \
    { };

FUNCTION_TRAITS()
FUNCTION_TRAITS(const)
FUNCTION_TRAITS(volatile)
FUNCTION_TRAITS(const volatile)

// 函数对象.
template <typename Callable>
struct function_traits : function_traits<decltype(&Callable::operator())>
{ };

// 特化处理 const 修饰的 lambda 表达式
template <typename Callable>
struct function_traits<const Callable> : function_traits<Callable>
{ };

// 特化处理引用类型的 lambda 表达式
template <typename Callable>
struct function_traits<Callable&> : function_traits<Callable>
{ };

template <typename Function>
typename function_traits<Function>::stl_function_type to_function(
    const Function& lambda)
{
    return static_cast<typename function_traits<Function>::stl_function_type>(
        lambda);
}

template <typename Function>
typename function_traits<Function>::stl_function_type to_function(
    Function&& lambda)
{
    return static_cast<typename function_traits<Function>::stl_function_type>(
        std::forward<Function>(lambda));
}

template <typename Function>
typename function_traits<Function>::pointer to_function_pointer(
    const Function& lambda)
{
    return static_cast<typename function_traits<Function>::pointer>(lambda);
}
}  // namespace v1

namespace v2
{
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
/*
template <typename Function>
struct function_traits;

template <typename Return, typename... Arguments>
struct function_traits<Return (*)(Arguments...)>
{
    using parameters_type = std::tuple<std::remove_cvref_t<Arguments>...>;
    using return_type     = Return;
};

template <typename Return, typename... Arguments>
struct function_traits<Return (*)(Arguments...) noexcept>
{
    using parameters_type = std::tuple<std::remove_cvref_t<Arguments>...>;
    using return_type     = Return;
};

template <typename Return, typename... Arguments>
struct function_traits<Return(Arguments...)>
{
    using parameters_type = std::tuple<std::remove_cvref_t<Arguments>...>;
    using return_type     = Return;
};

template <typename Return, typename... Arguments>
struct function_traits<Return(Arguments...) noexcept>
{
    using parameters_type = std::tuple<std::remove_cvref_t<Arguments>...>;
    using return_type     = Return;
};

template <typename This, typename Return, typename... Arguments>
struct function_traits<Return (This::*)(Arguments...)>
{
    using parameters_type = std::tuple<std::remove_cvref_t<Arguments>...>;
    using return_type     = Return;
    using class_type      = This;
};

template <typename This, typename Return, typename... Arguments>
struct function_traits<Return (This::*)(Arguments...) noexcept>
{
    using parameters_type = std::tuple<std::remove_cvref_t<Arguments>...>;
    using return_type     = Return;
    using class_type      = This;
};

template <typename This, typename Return, typename... Arguments>
struct function_traits<Return (This::*)(Arguments...) const>
{
    using parameters_type = std::tuple<std::remove_cvref_t<Arguments>...>;
    using return_type     = Return;
    using class_type      = This;
};

template <typename This, typename Return, typename... Arguments>
struct function_traits<Return (This::*)(Arguments...) const noexcept>
{
    using parameters_type = std::tuple<std::remove_cvref_t<Arguments>...>;
    using return_type     = Return;
    using class_type      = This;
};

template <typename Return>
struct function_traits<Return (*)()>
{
    using parameters_type = void;
    using return_type     = Return;
};

template <typename Return>
struct function_traits<Return (*)() noexcept>
{
    using parameters_type = void;
    using return_type     = Return;
};

template <typename Return>
struct function_traits<Return (&)()>
{
    using parameters_type = void;
    using return_type     = Return;
};

template <typename Return>
struct function_traits<Return (&)() noexcept>
{
    using parameters_type = void;
    using return_type     = Return;
};

template <typename Return>
struct function_traits<Return()>
{
    using parameters_type = void;
    using return_type     = Return;
};

template <typename Return>
struct function_traits<Return() noexcept>
{
    using parameters_type = void;
    using return_type     = Return;
};

template <typename This, typename Return>
struct function_traits<Return (This::*)()>
{
    using parameters_type = void;
    using return_type     = Return;
    using class_type      = This;
};

template <typename This, typename Return>
struct function_traits<Return (This::*)() noexcept>
{
    using parameters_type = void;
    using return_type     = Return;
    using class_type      = This;
};

template <typename This, typename Return>
struct function_traits<Return (This::*)() const>
{
    using parameters_type = void;
    using return_type     = Return;
    using class_type      = This;
};

template <typename This, typename Return>
struct function_traits<Return (This::*)() const noexcept>
{
    using parameters_type = void;
    using return_type     = Return;
    using class_type      = This;
};

// Support function object and lambda expression
template <class Function>
struct function_traits : function_traits<decltype(&Function::operator())>
{ };

template <typename Function>
using function_parameters_t =
    typename function_traits<std::remove_cvref_t<Function>>::parameters_type;

template <typename Function>
using last_parameters_type_t
    = std::tuple_element_t<std::tuple_size_v<function_parameters_t<Function>>
            - 1,
        function_parameters_t<Function>>;

template <typename Function>
using function_return_type_t =
    typename function_traits<std::remove_cvref_t<Function>>::return_type;

template <typename Function>
using class_type_t =
    typename function_traits<std::remove_cvref_t<Function>>::class_type;

template <typename F, typename... Args>
struct is_invocable
    : std::is_constructible<
          std::function<void(std::remove_reference_t<Args>...)>,
          std::reference_wrapper<typename std::remove_reference<F>::type>>
{ };

template <typename F, typename... Args>
inline constexpr bool is_invocable_v = is_invocable<F, Args...>::value;

template <typename T>
struct remove_first
{
    using type = T;
};

template <class First, class... Second>
struct remove_first<std::tuple<First, Second...>>
{
    using type = std::tuple<Second...>;
};

template <typename T>
using remove_first_t = typename remove_first<T>::type;

template <bool has_conn, typename T>
inline auto get_args()
{
    if constexpr (has_conn)
    {
        using args_type = remove_first_t<T>;
        return args_type{};
    }
    else
    {
        return T{};
    }
}

template <typename Test, template <typename...> class Ref>
struct is_specialization : std::false_type
{ };

template <template <typename...> class Ref, typename... Args>
struct is_specialization<Ref<Args...>, Ref> : std::true_type
{ };

template <typename Test, template <typename...> class Ref>
inline constexpr bool is_specialization_v = is_specialization<Test, Ref>::value;
*/
}  // namespace v2
}  // namespace function_traits