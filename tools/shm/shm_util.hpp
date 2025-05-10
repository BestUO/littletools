#pragma once

#include <signal.h>
#include <string_view>

template <typename T>
class TypeInfo
{
public:
    static constexpr std::string_view GetTypeName()
    {
#ifdef __clang__
        constexpr std::string_view prefix   = "[T = ";
        constexpr std::string_view suffix   = "]";
        constexpr std::string_view function = __PRETTY_FUNCTION__;
#elif defined(__GNUC__)
        constexpr std::string_view prefix   = "T = ";
        constexpr std::string_view suffix   = "]";
        constexpr std::string_view function = __PRETTY_FUNCTION__;
#else
        static_assert(false, "Unsupported compiler");
#endif

        auto start = function.find(prefix) + prefix.size();
        auto end   = function.rfind(suffix);
        return function.substr(start, end - start);
    }
};

class SHMUtil
{
public:
    static bool IsPidDead(pid_t tid)
    {
        return kill(tid, 0) != 0;
    }
};