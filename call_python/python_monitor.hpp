#pragma once

#include "python_wrap.hpp"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <map>
#include <thread>

class PythonMonitor : public PythonWrapper
{
public:
    PythonMonitor(const std::filesystem::path package_dir)
    {
        std::filesystem::path python_dir = package_dir / "python";
        printf("Initializing with package_dir: %s\n", python_dir.c_str());
        pybind11::module_ sys = pybind11::module_::import("sys");
        sys.attr("path").attr("insert")(0, python_dir.string());
        LoadAllModules(python_dir, package_dir);
        StartInstances();

        thread_ = std::thread([this]() {
            is_running_ = true;
            while (is_running_)
            {
                {
                    pybind11::gil_scoped_acquire acquire;
                    for (auto& [name, instance] : instances_)
                    {
                        if (!is_running_)
                            break;
                        try
                        {
                            auto msg
                                = CallMethod<std::string>(instance, "CallOnce");
                            if (!msg.empty())
                                printf("%s\n", msg.c_str());
                        } catch (const std::exception& e)
                        {
                            printf(
                                "[INFO] CallOnce interrupted: %s\n", e.what());
                            is_running_ = false;
                            break;
                        }
                    }
                }

                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            printf("PythonMonitor thread exiting...\n");
        });
    }

    ~PythonMonitor()
    {
        printf("Stopping PythonMonitor...\n");
        is_running_ = false;

        {
            pybind11::gil_scoped_release release;
            thread_.join();
        }
        StopInstances();
        printf("PythonMonitor stopped.\n");
    }

private:
    std::map<std::string, pybind11::object> instances_;
    std::atomic<bool> is_running_{false};
    std::thread thread_;

    void StartInstances()
    {
        for (auto& [name, instance] : instances_)
            CallMethodVoid(instance, "Start");
    }

    void StopInstances()
    {
        for (auto& [name, instance] : instances_)
        {
            try
            {
                CallMethodVoid(instance, "Stop");
            } catch (const std::exception& e)
            {
                printf("[WARN] Stop failed: %s\n", e.what());
            }
        }
    }

    void LoadAllModules(const std::filesystem::path& python_dir,
        const std::filesystem::path& package_dir)
    {
        if (!std::filesystem::exists(python_dir)
            || !std::filesystem::is_directory(python_dir))
        {
            std::cerr << "[PythonWrapper] Directory not found: " << python_dir
                      << std::endl;
            return;
        }

        for (const auto& entry :
            std::filesystem::directory_iterator(python_dir))
        {
            if (!entry.is_regular_file())
                continue;

            std::filesystem::path file = entry.path();
            if (file.extension() != ".py")
                continue;

            std::string module_name = file.stem().string();
            printf("Attempting to import module: %s\n", module_name.c_str());

            try
            {
                auto module = pybind11::module_::import(module_name.c_str());
                if (IsBaseOf(
                        "fun_base", "BaseInterface", module, "MonitorImpl"))
                {
                    instances_[module_name] = CreateInstance(
                        module, "MonitorImpl", package_dir.string());
                    printf("Created instance of MonitorImpl from module: %s\n",
                        module_name.c_str());
                }
            } catch (const pybind11::error_already_set& e)
            {
                printf("Failed to import module: %s, error: %s\n",
                    module_name.c_str(),
                    e.what());
            }
        }
    }
};