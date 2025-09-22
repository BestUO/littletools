#pragma once
#include "function_traits.hpp"
#include <any>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

template <typename T>
concept IsTuple = requires { requires std::tuple_size<T>::value >= 0; };

class FunExecutor
{
public:
    template <typename Function>
    void RegisterFunction(const std::string& name, Function&& func)
    {
        using traits = function_traits<std::decay_t<Function>>;

        auto wrapper = [func = std::forward<Function>(func)](
                           std::vector<std::any> args) mutable {
            if (args.size() != traits::arity)
            {
                throw std::invalid_argument("Argument count mismatch");
            }
            invoke_from_any<traits>(
                func, args, std::make_index_sequence<traits::arity>{});
        };
        functions_[name] = {typeid(Function).name(), wrapper};
    }

    template <typename... Args>
    bool InvokeFunction(const std::string& name, Args&&... args)
    {
        auto it = functions_.find(name);
        if (it != functions_.end())
        {
            it->second.second(ArgsToAnyVector(std::forward<Args>(args)...));
            return true;
        }
        else
        {
            return false;
        }
    }

private:
    // Use owning key type to avoid dangling string_view
    std::map<std::string,
        std::pair<std::string, std::function<void(std::vector<std::any>)>>>
        functions_;

    template <typename... Args>
    std::vector<std::any> ArgsToAnyVector(Args&&... args)
    {
        return {std::forward<Args>(args)...};
    }

    // Extract a single argument from std::any honoring reference qualifiers of
    // parameter P
    template <typename P>
    static decltype(auto) extract_arg(std::any& a)
    {
        using U = std::remove_cv_t<std::remove_reference_t<P>>;

        if constexpr (std::is_lvalue_reference_v<P>)
        {
            // P is U& or const U&
            return static_cast<P>(std::any_cast<U&>(a));
        }
        else if constexpr (std::is_rvalue_reference_v<P>)
        {
            // P is U&&
            return std::move(std::any_cast<U&>(a));
        }
        else
        {
            // P is by value
            return std::any_cast<U>(a);
        }
    }

    // Invoke function with arguments extracted from std::any vector, validating
    // types
    template <typename Traits, typename F, std::size_t... Indices>
    static void invoke_from_any(F& func,
        std::vector<std::any>& args,
        std::index_sequence<Indices...>)
    {
        // Validate types first for clearer error messages
        (void)std::initializer_list<int>{
            (validate_arg<typename Traits::template args<Indices>::type>(
                 args[Indices]),
                0)...};
        // Now invoke with properly formed arguments
        std::invoke(func,
            extract_arg<typename Traits::template args<Indices>::type>(
                args[Indices])...);
    }

    template <typename P>
    static void validate_arg(const std::any& a)
    {
        using U = std::remove_cv_t<std::remove_reference_t<P>>;
        if (!a.has_value() || a.type() != typeid(U))
        {
            throw std::runtime_error("Type mismatch for argument; expected: "
                + std::string(typeid(U).name()));
        }
    }
};