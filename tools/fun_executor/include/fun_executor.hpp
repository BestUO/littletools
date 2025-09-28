#pragma once
#include "function_traits.hpp"
#include <any>
#include <functional>
#include <nlohmann/json.hpp>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace funexecutor
{
namespace v1
{
template <typename T>
concept IsTuple = requires { requires std::tuple_size<T>::value >= 0; };

class TupleWrapper
{
public:
    template <typename... Args>
    TupleWrapper(Args&&... args)
        : data_(std::make_any<std::tuple<std::decay_t<Args>...>>(
              std::forward<Args>(args)...))
        // data_(std::forward_as_tuple(args...))
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
            using traits
                = function_traits::v1::function_traits<std::decay_t<Function>>;
            using args_tuple  = typename traits::tuple_type;
            using return_type = typename traits::return_type;
            auto args         = params.GetTuple<args_tuple>();
            if constexpr (std::is_void_v<return_type>)
            {
                std::apply(func, args);
            }
            else
            {
                if (result == nullptr)
                {
                    std::apply(func, args);
                }
                else
                {
                    *static_cast<return_type*>(result) = std::apply(func, args);
                }
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
                it->second(TupleWrapper(std::forward<Args>(args)...), nullptr);
                return OptionalReturnType{std::monostate{}};
            }
            else
            {
                R result;
                it->second(TupleWrapper(std::forward<Args>(args)...), &result);
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
}  // namespace v1

namespace v2
{
template <typename T>
concept IsTuple = requires { requires std::tuple_size<T>::value >= 0; };

class TupleWrapper
{
public:
    template <typename... Args>
    TupleWrapper(Args&&... args)
    {
        params_.reserve(sizeof...(Args));
        (params_.emplace_back(MakeAny(std::forward<Args>(args))), ...);
    }

    template <typename T>
    static std::any MakeAny(T&& v)
    {
        using Decayed = std::decay_t<T>;
        if constexpr (std::is_array_v<Decayed>
            && std::is_same_v<std::remove_extent_t<Decayed>, const char>)
        {
            return std::string(v);
        }
        else
        {
            return std::any(std::forward<T>(v));
        }
    }

    std::vector<std::any>& GetParams()
    {
        return params_;
    }

private:
    std::vector<std::any> params_;
};

class FunExecutor
{
public:
    template <typename Function>
    void RegisterFunction(const std::string& name, Function&& func)
    {
        functions_[name] = [func = std::forward<Function>(func)](
                               TupleWrapper params, void* result) mutable {
            using args_tuple = typename function_traits::v2::function_traits<
                std::decay_t<Function>>::arguments;
            using return_type = typename function_traits::v2::function_traits<
                std::decay_t<Function>>::return_type;
            if constexpr (std::is_void_v<return_type>)
            {
                // std::apply(func, args);
                InvokeFromAny<args_tuple>(func,
                    std::move(params.GetParams()),
                    std::make_index_sequence<std::tuple_size_v<args_tuple>>{});
            }
            else
            {
                if (result == nullptr)
                {
                    InvokeFromAny<args_tuple>(func,
                        std::move(params.GetParams()),
                        std::make_index_sequence<
                            std::tuple_size_v<args_tuple>>{});
                }
                else
                {
                    *static_cast<return_type*>(result)
                        = InvokeFromAny<args_tuple>(func,
                            std::move(params.GetParams()),
                            std::make_index_sequence<
                                std::tuple_size_v<args_tuple>>{});
                }
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
                it->second(TupleWrapper(std::forward<Args>(args)...), nullptr);
                return OptionalReturnType{std::monostate{}};
            }
            else
            {
                R result;
                it->second(TupleWrapper(std::forward<Args>(args)...), &result);
                return OptionalReturnType{std::move(result)};
            }
        }
        else
        {
            return OptionalReturnType{std::nullopt};
        }
    }

    void Merge(FunExecutor&& other)
    {
        functions_.merge(other.functions_);
    }

private:
    std::map<std::string,
        std::function<void(TupleWrapper params, void* result)>>
        functions_;

    template <typename Target>
    static decltype(auto) AnyToDestType(std::any& a)
    {
        using Clean = std::remove_cv_t<std::remove_reference_t<Target>>;
        if (a.type() == typeid(Clean))
        {
            if constexpr (std::is_lvalue_reference_v<Target>)
            {
                if constexpr (std::is_const_v<std::remove_reference_t<Target>>)
                {
                    return static_cast<const Clean&>(std::any_cast<Clean&>(a));
                }
                else
                {
                    return static_cast<Clean&>(std::any_cast<Clean&>(a));
                }
            }
            else if constexpr (std::is_rvalue_reference_v<Target>)
            {
                return std::move(std::any_cast<Clean&>(a));
            }
            else
            {
                // Value requested – copy from stored object.
                return static_cast<Clean&>(std::any_cast<Clean&>(a));
            }
        }
        else if constexpr (std::is_same_v<Clean, std::string>)
        {
            // Allow implicit conversion from C-string pointers.
            if (a.type() == typeid(const char*) || a.type() == typeid(char*))
            {
                static thread_local std::string tmp_str;
                const char* c = (a.type() == typeid(const char*))
                    ? std::any_cast<const char*>(a)
                    : static_cast<const char*>(std::any_cast<char*>(a));
                tmp_str       = c;
                if constexpr (std::is_lvalue_reference_v<Target>)
                {
                    return static_cast<const std::string&>(tmp_str);
                }
                else if constexpr (std::is_rvalue_reference_v<Target>)
                {
                    return std::move(tmp_str);  // will copy/move to parameter
                }
                else
                {
                    return static_cast<const std::string&>(tmp_str);
                }
            }
        }
        else if (a.type() == typeid(nlohmann::json))
        {
            // Future: support conversion from json to arbitrary type.
        }
        throw std::runtime_error(
            std::string("bad any conversion for parameter type: ")
            + a.type().name());
    }

    template <IsTuple Args_tuple, typename F, std::size_t... Indices>
    static auto InvokeFromAny(F&& func,
        std::vector<std::any>&& args,
        std::index_sequence<Indices...>)
    {
        if (args.size() != sizeof...(Indices))
        {
            throw std::runtime_error("argument count mismatch");
        }
        return std::invoke(std::forward<F>(func),
            AnyToDestType<std::tuple_element_t<Indices, Args_tuple>>(
                args[Indices])...);
    }
};
}  // namespace v2
}  // namespace funexecutor