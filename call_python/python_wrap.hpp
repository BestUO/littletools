#pragma once

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>

class PythonWrapper {
public:
    PythonWrapper() : interpreter_() {}

    std::string CallFunction(const pybind11::module_ &test, const std::string &func_name) {
        try {
            pybind11::object result = test.attr(func_name.c_str())();
            return result.cast<std::string>();
        } catch (const pybind11::error_already_set &e) {
            throw std::runtime_error(std::string("Python error: ") + e.what());
        }
    }

    template<typename ReturnType, typename... Args>
    ReturnType CallFunction(const pybind11::module_ &test, const std::string &func_name, Args &&...args) {
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
    pybind11::object CreateInstance(const pybind11::module_ &test, const std::string &class_name, Args &&...args) {
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

    bool HasCallable(const pybind11::module_ &module, const std::string &class_name, const std::string &method_name) {
        try {
            pybind11::object cls = module.attr(class_name.c_str());

            if (!pybind11::hasattr(cls, method_name.c_str())) { return false; }

            pybind11::object callable = pybind11::module_::import("builtins").attr("callable");
            return callable(cls.attr(method_name.c_str())).cast<bool>();
        } catch (...) { return false; }
    }

private:
    pybind11::scoped_interpreter interpreter_;
};