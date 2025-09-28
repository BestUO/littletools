#pragma once
#include "function_traits.hpp"
#include <any>
#include <functional>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

template<typename T>
concept IsTuple = requires { requires std::tuple_size<T>::value >= 0; };

class TupleWrapper {
public:
    template<typename... Args>
    TupleWrapper(Args &&...args) : params_({std::forward<Args>(args)...}) {}

    std::vector<std::any> &GetParams() { return params_; }

private:
    std::vector<std::any> params_;
};

class FunExecutor {
public:
    template<typename Function>
    void RegisterFunction(const std::string &name, Function &&func) {
        functions_[name] = [func = std::forward<Function>(func)](TupleWrapper params, void *result) mutable {
            using args_tuple  = typename function_traits<std::decay_t<Function>>::arguments;
            using return_type = typename function_traits<std::decay_t<Function>>::return_type;
            if constexpr (std::is_void_v<return_type>) {
                InvokeFromAny<args_tuple>(func, std::move(params.GetParams()),
                                          std::make_index_sequence<std::tuple_size_v<args_tuple>>{});
            } else {
                if (result == nullptr) {
                    InvokeFromAny<args_tuple>(func, std::move(params.GetParams()),
                                              std::make_index_sequence<std::tuple_size_v<args_tuple>>{});
                } else {
                    *static_cast<return_type *>(result) =
                            InvokeFromAny<args_tuple>(func, std::move(params.GetParams()),
                                                      std::make_index_sequence<std::tuple_size_v<args_tuple>>{});
                }
            }
        };
    }

    template<typename R, typename... Args>
    auto InvokeFunction(const std::string &name, Args &&...args) {
        using ReturnType         = std::conditional_t<std::is_void_v<R>, std::monostate, R>;
        using OptionalReturnType = std::optional<ReturnType>;
        auto it                  = functions_.find(name);
        if (it != functions_.end()) {
            if constexpr (std::is_void_v<R>) {
                it->second(TupleWrapper(std::forward<Args>(args)...), nullptr);
                return OptionalReturnType{std::monostate{}};
            } else {
                R result;
                it->second(TupleWrapper(std::forward<Args>(args)...), &result);
                return OptionalReturnType{std::move(result)};
            }
        } else {
            return OptionalReturnType{std::nullopt};
        }
    }

    void Merge(FunExecutor &&other) { functions_.merge(other.functions_); }

private:
    std::map<std::string, std::function<void(TupleWrapper params, void *result)>> functions_;

    template<typename Target>
    static std::decay_t<Target> AnyToDestType(const std::any &a) {
        using Clean = std::decay_t<Target>;
        if (a.type() == typeid(Clean)) { return std::any_cast<Clean>(a); }
        if (a.type() == typeid(nlohmann::json)) {
            const auto &j = std::any_cast<const nlohmann::json &>(a);
            return j.get<Clean>();
        }
        throw std::runtime_error(std::string("bad any conversion for parameter type: ") + a.type().name());
    }

    template<IsTuple Args_tuple, typename F, std::size_t... Indices>
    static auto InvokeFromAny(F &&func, std::vector<std::any> &&args, std::index_sequence<Indices...>) {
        return std::invoke(std::forward<F>(func),
                           AnyToDestType<std::tuple_element_t<Indices, Args_tuple>>(args[Indices])...);
    }
};