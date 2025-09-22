#pragma once
#include "function_traits.hpp"
#include <any>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

template <typename T>
concept IsTuple = requires { requires std::tuple_size<T>::value >= 0; };

class TupleWrapper
{
public:
    template <typename... Args>
    TupleWrapper(std::tuple<Args...> tuple)
        : data_(std::move(tuple))
        , size_(sizeof...(Args))
    {
        StoreTypes<Args...>(std::index_sequence_for<Args...>{});
    }

    template <typename ExpectedTuple>
    const ExpectedTuple& GetTuple() const
    {
        if (!IsMatched<ExpectedTuple>())
        {
            throw std::runtime_error("Type mismatch when accessing tuple");
        }
        return std::any_cast<const ExpectedTuple&>(data_);
    }

private:
    std::any data_;
    size_t size_;
    std::vector<std::type_info const*> element_types_;

    template <typename ExpectedTuple>
    bool IsMatched() const
    {
        constexpr std::size_t expected_size = std::tuple_size_v<ExpectedTuple>;
        if (size_ != expected_size)
        {
            return false;
        }
        return CheckTypes<ExpectedTuple>(
            std::make_index_sequence<expected_size>{});
    }

    template <typename... Args, std::size_t... I>
    void StoreTypes(std::index_sequence<I...>)
    {
        element_types_ = {&typeid(Args)...};
    }

    template <typename ExpectedTuple, std::size_t... I>
    bool CheckTypes(std::index_sequence<I...>) const
    {
        return ((element_types_[I]
                    == &typeid(std::tuple_element_t<I, ExpectedTuple>))
            && ...);
    }
};

class FunExecutor
{
public:
    template <typename Function>
    void RegisterFunction(const std::string& name, Function&& func)
    {
        functions_[name] = [func = std::forward<Function>(func)](
                               TupleWrapper params, void* result) mutable {
            using traits     = function_traits<std::decay_t<Function>>;
            using args_tuple = typename traits::tuple_type;
            auto args        = params.GetTuple<args_tuple>();
            if constexpr (std::is_void_v<typename traits::return_type>)
            {
                std::apply(
                    [&func](auto&&... args) {
                        std::invoke(
                            func, std::forward<decltype(args)>(args)...);
                    },
                    args);
            }
            else
            {
                if (result == nullptr)
                {
                    throw std::invalid_argument(
                        "Result pointer is null for non-void function");
                }
                using return_type = typename traits::return_type;
                *static_cast<return_type*>(result) = std::apply(func, args);
            }
        };
    }

    template <typename R, typename... Args>
    auto InvokeFunction(const std::string& name, Args&&... args)
    {
        using ReturnType
            = std::conditional_t<std::is_void_v<R>, std::monostate, R>;
        using OptionalReturnType = std::optional<ReturnType>;
        auto it                  = functions_.find(name);
        if (it != functions_.end())
        {
            if constexpr (std::is_void_v<R>)
            {
                it->second(
                    TupleWrapper(std::make_tuple(std::forward<Args>(args)...)),
                    nullptr);
                return OptionalReturnType{std::monostate{}};
            }
            else
            {
                R result;
                it->second(
                    TupleWrapper(std::make_tuple(std::forward<Args>(args)...)),
                    &result);
                return OptionalReturnType{std::move(result)};
            }
        }
        else
        {
            return OptionalReturnType{std::nullopt};
        }
    }

private:
    std::map<std::string,
        std::function<void(TupleWrapper params, void* result)>>
        functions_;
};