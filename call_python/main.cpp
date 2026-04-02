#include "python_monitor.hpp"

int main() {
    PythonMonitor python_monitor;
    pybind11::module_ test = pybind11::module_::import("test");
    printf("%s\n", python_monitor.CallFunction(test, "hello").c_str());
    printf("%d\n", python_monitor.CallFunction<int>(test, "add", 3, 5));
    auto instance = python_monitor.CreateInstance(test, "Robot", "Robo", 1.5);
    printf("%s\n", python_monitor.CallMethod<std::string>(instance, "move", 10.0).c_str());

    return 0;
}