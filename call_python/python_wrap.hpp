#pragma once

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>

class PythonWrapper {
public:
    PythonWrapper() {}

    std::string CallFunction(pybind11::module_ &test, const std::string &func_name) {
        try {
            pybind11::object result = test.attr(func_name.c_str())();
            return result.cast<std::string>();
        } catch (const pybind11::error_already_set &e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }
    }

    template<typename ReturnType, typename... Args>
    ReturnType CallFunction(pybind11::module_ &test, const std::string &func_name, Args &&...args) {
        try {
            pybind11::object result = test.attr(func_name.c_str())(std::forward<Args>(args)...);
            return result.cast<ReturnType>();
        } catch (const pybind11::error_already_set &e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }
    }


    void ExecScript(const std::string &code) {
        try {
            pybind11::exec(code);
        } catch (const pybind11::error_already_set &e) {
            throw std::runtime_error(std::string("Python exec error: ") + e.what());
        }
    }

    template<typename... Args>
    pybind11::object CreateInstance(pybind11::module_ &test, const std::string &class_name, Args &&...args) {
        try {
            pybind11::object cls = test.attr(class_name.c_str());
            return cls(std::forward<Args>(args)...);
        } catch (const pybind11::error_already_set &e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }
    }

    template<typename ReturnType>
    ReturnType CallMethod(pybind11::object &instance, const std::string &method_name) {
        try {
            return instance.attr(method_name.c_str())().cast<ReturnType>();
        } catch (const pybind11::error_already_set &e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }
    }

    template<typename ReturnType, typename... Args>
    ReturnType CallMethod(pybind11::object &instance, const std::string &method_name, Args &&...args) {
        try {
            return instance.attr(method_name.c_str())(std::forward<Args>(args)...).template cast<ReturnType>();
        } catch (const pybind11::error_already_set &e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }
    }


    template<typename... Args>
    void CallMethodVoid(pybind11::object &instance, const std::string &method_name, Args &&...args) {
        try {
            instance.attr(method_name.c_str())(std::forward<Args>(args)...);
        } catch (const pybind11::error_already_set &e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }
    }

    template<typename ReturnType>
    ReturnType GetAttr(pybind11::object &instance, const std::string &attr_name) {
        try {
            return instance.attr(attr_name.c_str()).cast<ReturnType>();
        } catch (const pybind11::error_already_set &e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }
    }

    template<typename T>
    void SetAttr(pybind11::object &instance, const std::string &attr_name, T value) {
        try {
            instance.attr(attr_name.c_str()) = value;
        } catch (const pybind11::error_already_set &e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }
    }

    template<typename... MethodNames>
    bool HasCallable(pybind11::module_ &module, const std::string &class_name, MethodNames &&...method_names) {
        static_assert((std::is_convertible_v<MethodNames, std::string_view> && ...),
                      "All method names must be string-like");
        try {
            pybind11::object cls      = module.attr(class_name.c_str());
            pybind11::object callable = pybind11::module_::import("builtins").attr("callable");

            return (... && [&](std::string_view method_name) {
                if (!pybind11::hasattr(cls, method_name.data())) { return false; }
                return callable(cls.attr(method_name.data())).cast<bool>();
            }(std::string_view(method_names)));
        } catch (...) { return false; }
    }

    bool IsBaseOf(const std::string &base_module_name, const std::string &base_class_name, pybind11::module_ &module,
                  const std::string &class_name) {
        try {
            pybind11::object base_cls =
                    pybind11::module_::import(base_module_name.c_str()).attr(base_class_name.c_str());
            pybind11::object cls        = module.attr(class_name.c_str());
            pybind11::object issubclass = pybind11::module_::import("builtins").attr("issubclass");

            return issubclass(cls, base_cls).cast<bool>();
        } catch (const pybind11::error_already_set &e) {
            printf("[IsBaseOf] Python error: %s\n", e.what());
            return false;
        }
    }
};