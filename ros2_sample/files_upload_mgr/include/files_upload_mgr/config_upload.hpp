#pragma once

#include "app_interfaces/msg/cloud_upload_type.hpp"
#include "common_utils/file_utils/file_utils.hpp"
#include "common_utils/yjh_authentication/yjh_authentication.hpp"
#include "node_to_iot/node_to_iot.hpp"
#include "type_define.hpp"
#include "yjhlog/logger.h"
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>

namespace app_nodes::files_upload_mgr {
    class ConfigUpload {
    public:
        ConfigUpload(std::shared_ptr<app_nodes::files_upload_mgr::NodeBaseInfo> node_base_info) {
            if (std::filesystem::exists(files_upload_path) || std::filesystem::create_directories(files_upload_path)) {
                config_                 = node_base_info->config;
                node_to_iot_client_ptr_ = node_base_info->node_to_iot_client_ptr;
            } else {
                throw std::runtime_error("Create files upload dir failed: " + files_upload_path);
            }
            Run();
        }

        ~ConfigUpload() { Stop(); }

    private:
        std::mutex mutex_;
        std::condition_variable cv_;
        std::thread upload_thread_;
        std::atomic<bool> stop_                                        = true;
        std::shared_ptr<cloud_platform::ConnectInfo> connect_info_ptr_ = nullptr;
        FileUploadMgrConfig config_;
        std::shared_ptr<iot::NodeToIotClient> node_to_iot_client_ptr_;

        void Run() {
            stop_          = false;
            upload_thread_ = std::thread([this]() {
                while (!stop_) {
                    std::unique_lock<std::mutex> lock(mutex_);
                    cv_.wait_for(lock, std::chrono::seconds(60));
                    RunOnce();
                }
            });
        }

        void Stop() {
            stop_ = true;
            cv_.notify_one();
            if (upload_thread_.joinable()) { upload_thread_.join(); }
        }

        std::optional<cloud_platform::ConnectInfo> GetConnectInfo() {
            auto response = node_to_iot_client_ptr_->GetCloudPlatformConnectInfo();
            LOG_INFO("Get connect info response: {}", response);
            if (!response.empty()) {
                try {
                    cloud_platform::ConnectInfo connect_info = nlohmann::json::parse(response);
                    return connect_info;
                } catch (const std::exception &e) { LOG_ERROR("Failed to get connect info: {}", e.what()); }
            } else {
                LOG_WARN("Get connect info response is empty");
            }
            return std::nullopt;
        }

        DirsInfo GetDirsInfo() {
            nlohmann::json files_info_json = nlohmann::json::object();
            if (std::filesystem::exists(files_info_path)) {
                std::ifstream ifs(files_info_path);
                ifs >> files_info_json;
                return files_info_json;
            }
            return {};
        }

        bool SaveDirsInfo(const DirsInfo &dirs_info) {
            std::ofstream file(files_info_path);
            if (!file.is_open()) {
                LOG_ERROR("Failed to open file: {}", files_info_path);
                return false;
            }
            nlohmann::json files_info_json = dirs_info;
            file << files_info_json.dump(4);
            file.close();
            return true;
        }

        bool UploadConfigDirs() {
            std::vector<std::string> upload_paths;
            bool ret = false;
            for (const auto &config_dir : config_.config_dirs.dirs) { upload_paths.push_back(config_dir.dir_path); }
            if (!upload_paths.empty()) {
                if (common_utils::file_utils::GenZip(upload_paths, {files_info_path}, config_dirs_zip_path)) {
                    if (auto connect_info_opt = GetConnectInfo(); !connect_info_opt) {
                        LOG_ERROR("Failed to get cloud platform connect info");
                        ret = false;
                    } else {
                        ret = cloud_platform::YJHAuthentication::UploadFileToCloud(
                                      connect_info_opt, config_dirs_zip_path,
                                      app_interfaces::msg::CloudUploadType::CONFIG_ZIP)
                                      .has_value();
                    }
                } else {
                    LOG_ERROR("Generate zip for config dirs failed");
                    ret = false;
                }
                common_utils::file_utils::remove_file(config_dirs_zip_path);
            }
            return ret;
        }

        void RunOnce() {
            bool need_upload = false;
            auto dirs_info   = GetDirsInfo();
            for (const auto &config_dir : config_.config_dirs.dirs) {
                if (std::filesystem::exists(config_dir.dir_path)) {
                    auto sys_time = std::chrono::clock_cast<std::chrono::system_clock>(
                            std::filesystem::last_write_time(config_dir.dir_path));
                    auto modify_time =
                            std::chrono::duration_cast<std::chrono::seconds>(sys_time.time_since_epoch()).count();
                    if (dirs_info.dirs.find(config_dir.dir_path) == dirs_info.dirs.end() ||
                        dirs_info.dirs[config_dir.dir_path].modify_time != modify_time) {
                        need_upload                                     = true;
                        dirs_info.dirs[config_dir.dir_path].modify_time = modify_time;
                    }
                } else {
                    LOG_WARN("Config dir does not exist: {}", config_dir.dir_path);
                }
            }
            if (need_upload) {
                SaveDirsInfo(dirs_info);
                if (!UploadConfigDirs()) { std::filesystem::remove(files_info_path); }
            }
        }
    };
}// namespace app_nodes::files_upload_mgr