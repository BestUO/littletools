#pragma once

#include <format>
#include <iostream>
#include <string_view>

namespace common_utils {
    constexpr std::string_view GetFilename(std::string_view path) {
        size_t pos = path.find_last_of("/");
        return (pos == std::string_view::npos) ? path : path.substr(pos + 1);
    }


    template<typename... Args>
    inline void CustomLogPrint(std::string_view file, int line, std::string_view format, Args... args) {
        std::cout << std::format("[{}:{}] {}", GetFilename(file), line,
                                 std::vformat(format, std::make_format_args(args...)))
                  << std::endl;
    }

#define J36_LOGGER_INFO(...) common_utils::CustomLogPrint(__FILE__, __LINE__, __VA_ARGS__);
}// namespace common_utils