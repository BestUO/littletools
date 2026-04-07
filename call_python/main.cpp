#include "python_monitor.hpp"
#include <atomic>
#include <csignal>
#include <condition_variable>
#include <mutex>

static std::atomic<bool> g_shutdown{false};
static std::condition_variable g_cv;
static std::mutex g_mutex;

static void SignalHandler(int /*sig*/)
{
    g_shutdown.store(true);
    g_cv.notify_all();
}

int main()
{
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    pybind11::scoped_interpreter interpreter;

    PythonMonitor python_monitor("./");

    {
        // 释放 GIL，等待退出信号
        pybind11::gil_scoped_release release;

        std::unique_lock<std::mutex> lock(g_mutex);
        g_cv.wait(lock, [] {
            return g_shutdown.load();
        });
    }
    // 离开 release 作用域后 GIL 已重新获取
    // python_monitor 在此处析构，GIL 已持有，析构安全

    printf("Shutting down...\n");
    return 0;
}