#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <string.h>
#include <tuple>
#include <vector>
#include <map>
#include <iostream>
#include <atomic>
#include <set>
#include <iomanip>

class SimpleLog
{
public:
    bool m_enableLog = false;

    SimpleLog()
    {
        m_enableLog = getLogFlag();
    };

    bool getLogFlag()
    {
        auto env = getenv("ACFW_CM_CONSOLE_LOG");
        if (env && std::string(env) == "true")
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void logOut2()
    {
        std::cout << std::endl;
    }

    template <typename First, typename... Rest>
    void logOut2(First&& first, Rest&&... rest)
    {
        std::cout << std::forward<First>(first);
        logOut2(std::forward<Rest>(rest)...);
    }

    template <typename... Args>
    void logOut(Args... args)
    {
        if (m_enableLog)
        {
            printBaseInfo();
            logOut2(std::forward<Args>(args)...);
        }
    }

    void printBaseInfo()
    {
        std::chrono::time_point<std::chrono::system_clock> currentTime
            = std::chrono::system_clock::now();

        auto duration = currentTime.time_since_epoch();
        auto microseconds
            = std::chrono::duration_cast<std::chrono::microseconds>(duration)
            % std::chrono::seconds(1);

        std::time_t currentTime_t
            = std::chrono::system_clock::to_time_t(currentTime);
        std::tm utc_tm = *std::gmtime(&currentTime_t);

        std::cout << std::put_time(&utc_tm, "%Y-%m-%d %H:%M:%S") << '.'
                  << std::setw(6) << std::setfill('0') << microseconds.count()
                  << " " << std::this_thread::get_id() << " ";
    }
};