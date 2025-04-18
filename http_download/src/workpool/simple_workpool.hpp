#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>

class SimpleWorkPool
{
public:
    SimpleWorkPool()  = default;
    ~SimpleWorkPool() = default;

    template <typename Func, typename... Args>
    void AddTask(Func&& func, Args&&... args)
    {
        __tasks.emplace_back(
            std::bind(std::forward<Func>(func), std::forward<Args>(args)...));
    }

    void StartTask(size_t thread_num)
    {
        for (size_t i = 0; i < thread_num; ++i)
        {
            __threads.emplace_back([this]() {
                while (true)
                {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(__mutex);
                        if (__tasks.empty())
                        {
                            break;
                        }
                        task = __tasks.back();
                        __tasks.pop_back();
                    }
                    task();
                }
            });
        }
        for (auto& thread : __threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

private:
    std::vector<std::thread> __threads;
    std::mutex __mutex;
    std::vector<std::function<void()>> __tasks;
};