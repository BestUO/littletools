#pragma once

#include "python_wrap.hpp"
#include <filesystem>
#include <iostream>
#include <map>

class PythonMonitor : public PythonWrapper {
public:
    PythonMonitor() : PythonWrapper() {
        std::filesystem::path file_dir   = std::filesystem::path(__FILE__).parent_path();
        printf("PythonMonitor initializing, python_dir: %s\n", std::filesystem::read_symlink("/proc/self/exe").c_str());
        std::filesystem::path python_dir = file_dir / "python";
        pybind11::module_ sys            = pybind11::module_::import("sys");
        sys.attr("path").attr("insert")(0, python_dir.string());
    }

private:
    std::map<std::string, pybind11::module_> modules_;

    void LoadAllModules(const std::filesystem::path &python_dir) {
        if (!std::filesystem::exists(python_dir) || !std::filesystem::is_directory(python_dir)) {
            std::cerr << "[PythonWrapper] Directory not found: " << python_dir << std::endl;
            return;
        }

        for (const auto &entry : std::filesystem::directory_iterator(python_dir)) {
            if (!entry.is_regular_file()) continue;

            std::filesystem::path file = entry.path();
            if (file.extension() != ".py") continue;

            std::string module_name = file.stem().string();
            std::cout << "[PythonWrapper] Found: " << file.filename() << "  →  import " << module_name << std::endl;

            try {
                modules_[module_name] = pybind11::module_::import(module_name.c_str());
                std::cout << "[PythonWrapper] OK: " << module_name << std::endl;
            } catch (const pybind11::error_already_set &e) {
                std::cerr << "[PythonWrapper] Failed to import '" << module_name << "': " << e.what() << std::endl;
            }
        }
    }
};