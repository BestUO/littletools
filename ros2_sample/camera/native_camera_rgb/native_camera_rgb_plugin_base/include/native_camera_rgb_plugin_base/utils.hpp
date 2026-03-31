#pragma once

#include "iostream"
#include <chrono>
#include <filesystem>
#include <string>
#include <tuple>

namespace controller_native {
    class CameraUtils {
    public:
        static std::tuple<bool, std::string> CreateDirectoriesAndGenFileName(const std::string &file_dir,
                                                                             const std::string &format) {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto local_time = std::localtime(&time_t);

            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
            char date_dir[32];
            std::strftime(date_dir, sizeof(date_dir), "%Y%m%d", local_time);
            char time_base[32];
            std::strftime(time_base, sizeof(time_base), "%H%M%S", local_time);

            auto millisec_str = std::format("{:03d}", static_cast<int>(milliseconds.count()));
            std::string time_filename = std::string(time_base) + millisec_str;

            std::filesystem::path full_path = file_dir;
            full_path.append(date_dir);

            std::error_code ec;
            std::filesystem::create_directories(full_path, ec);
            if (!ec) {
                full_path.append(time_filename + format);
                return {true, full_path.string()};
            } else {
                std::cout << "Failed to create directory: " << full_path.string() << "error: " << ec.message()
                          << std::endl;
                return {false, "Failed to create directory: " + full_path.string()};
            }
        }
    };
}// namespace controller_native