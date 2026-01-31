#pragma once

#include "app_interfaces/msg/cloud_upload_type.hpp"
#include "app_interfaces/srv/iot_server_info.hpp"
#include "class_base.hpp"
#include "common_utils/file_utils/file_utils.hpp"
#include "common_utils/func/func.h"
#include "common_utils/yjh_authentication/http_client.hpp"
#include "common_utils/yjh_authentication/yjh_authentication.hpp"
#include "type_define.hpp"
#include "yjhlog/logger.h"
#include <filesystem>

namespace app_nodes::files_upload_mgr {
    struct OtherModueInfo {
        rclcpp::Client<app_interfaces::srv::IotServerInfo>::SharedPtr client;
        std::string module_name;
        std::string command;
        nlohmann::json body;
    };

    class IotSimpleHandle : public ClassBase<IotSimpleHandle> {
    public:
        using ClassBase<IotSimpleHandle>::CallImpl;
        IotSimpleHandle([[maybe_unused]] std::shared_ptr<NodeBaseInfo> node_base_info) {
            node_to_iot_client_ptr_ = node_base_info->node_to_iot_client_ptr;
            for (const auto &dir_element : node_base_info->config.config_dirs.dirs) {
                if (!dir_element.module_name.empty()) {
                    other_module_info_list_.push_back(
                            {node_base_info->node->create_client<app_interfaces::srv::IotServerInfo>(
                                     "iot_to_" + dir_element.module_name, rclcpp::ServicesQoS(),
                                     node_base_info->service_cb_group),
                             dir_element.module_name, dir_element.command, nlohmann::json::object()});
                }
            }
            for (const auto &dir_element : node_base_info->config.log_dirs.dirs) {
                auto file_info_list = GenFileInfoList(dir_element.dir_path);
                if (file_info_map_.find(dir_element.command) == file_info_map_.end()) {
                    file_info_map_[dir_element.command] = std::move(file_info_list);
                } else {
                    file_info_map_[dir_element.command].insert(file_info_map_[dir_element.command].end(),
                                                               file_info_list.begin(), file_info_list.end());
                }
            }
        }

        CommonRet CallImpl(const std::shared_ptr<typename app_interfaces::srv::IotServerInfo::Request> request,
                           std::shared_ptr<typename app_interfaces::srv::IotServerInfo::Response> response) {
            CommonRet ret{CommonRet::CommonStatus::NOT_SUPPORTED_YET, nlohmann::json::object()};
            if (request->command == "get_log_list") {
                ret = GetLogList();
            } else if (request->command == "get_bag_list") {
                ret = GetBagList();
            } else if (request->command == "sync_config") {
                //used for sync config from cloud platform
                ret = SyncConfig(request->body);
            } else if (request->command == "notify_dev_upload_file") {
                //used for upload log file to cloud platform
                ret = NotifyDevUploadFile(request->body);
            } else {
                ret = {CommonRet::CommonStatus::NOT_SUPPORTED_YET, nlohmann::json::object()};
            }
            response->result = nlohmann::json(ret).dump();
            return ret;
        }

    private:
        std::shared_ptr<iot::NodeToIotClient> node_to_iot_client_ptr_;
        std::vector<OtherModueInfo> other_module_info_list_;
        std::map<std::string, std::vector<FileInfo>> file_info_map_;

        CommonRet GetLogList() {
            LOG_INFO("GetLogList");
            if (auto it = file_info_map_.find("get_log_list"); it != file_info_map_.end()) {
                return {CommonRet::CommonStatus::SUCCESS, nlohmann::json{{"fileList", it->second}}};
            }
            return {CommonRet::CommonStatus::NOT_SUPPORTED_YET, nlohmann::json::object()};
        }

        CommonRet GetBagList() {
            LOG_INFO("GetBagList");
            if (auto it = file_info_map_.find("get_bag_list"); it != file_info_map_.end()) {
                return {CommonRet::CommonStatus::SUCCESS, nlohmann::json{{"fileList", it->second}}};
            }
            return {CommonRet::CommonStatus::NOT_SUPPORTED_YET, nlohmann::json::object()};
        }

        bool SyncConfigModule(const OtherModueInfo &client_info) {
            if (!client_info.client->wait_for_service(std::chrono::seconds(3))) {
                LOG_ERROR("service {} not available", client_info.client->get_service_name());
                return false;
            } else {
                auto srv_request  = std::make_shared<typename app_interfaces::srv::IotServerInfo::Request>();
                srv_request->head = client_info.module_name;//head：标识归属于哪个业务节点处理
                srv_request->command = client_info.command;//command：具体的处理指令，如设备控制，读取状态等
                srv_request->body = client_info.body.dump();//body：具体的指令参数

                auto result_future = client_info.client->async_send_request(srv_request);
                auto status        = result_future.wait_for(std::chrono::seconds(3));
                if (status != std::future_status::ready) {
                    client_info.client->remove_pending_request(result_future);
                    LOG_ERROR("call service {} timeout", client_info.client->get_service_name());
                    return false;
                } else {
                    auto result = nlohmann::json::parse(result_future.get()->result);
                    LOG_INFO("SyncConfigModule {} result: {}", client_info.module_name, result.dump());
                    return result.value("code", 0) == 0;
                }
            }
        }

        bool SyncModules() {
            return std::all_of(other_module_info_list_.begin(), other_module_info_list_.end(),
                               [this](const OtherModueInfo &client_info) { return SyncConfigModule(client_info); });
        }

        CommonRet SyncConfig(const std::string &body) {
            LOG_INFO("SyncConfig");
            std::thread([this, body]() {
                try {
                    auto body_json  = nlohmann::json::parse(body);
                    std::string url = body_json.at("configUrl");

                    uint8_t try_count = 0;
                    while (try_count++ < 3) {
                        if (http_client::HttpClientMgr::Instance().DownloadFile(url, config_dirs_zip_path)) {
                            if (common_utils::file_utils::UnZip(config_dirs_zip_path, "/")) {
                                nlohmann::json j = nlohmann::json::object();
                                if (SyncModules()) {
                                    j["messageId"] = common_utils::func::getUUID();
                                    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                             std::chrono::system_clock::now().time_since_epoch())
                                                             .count();
                                    j["properties"]["syncConfigFinishedNotice"] = nlohmann::json::array();
                                    node_to_iot_client_ptr_->PubToIot(
                                            "/properties/report", app_interfaces::msg::NodeToIot::MQTT_PUB_FAIL_RETRY,
                                            j);
                                    LOG_INFO("Published syncConfigFinishedNotice message to IoT: {}", j.dump());
                                    break;
                                }
                            }
                        } else {
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        }
                    }
                } catch (const std::exception &e) { LOG_ERROR("syncConfigFinishedNotice exception: {}", e.what()); }
            }).detach();

            return {CommonRet::CommonStatus::SUCCESS, nlohmann::json::object()};
        }

        std::vector<FileInfo> GenFileInfoList(const std::string &dir_path) {
            std::vector<FileInfo> file_list;
            if (!std::filesystem::exists(dir_path) || std::filesystem::is_regular_file(dir_path)) { return file_list; }

            std::filesystem::path root_path = std::filesystem::absolute(dir_path);
            if (root_path.filename().empty()) { root_path = root_path.parent_path(); }

            FileInfo root_info;
            root_info.name = std::filesystem::absolute(root_path).filename().string();
            root_info.path = std::filesystem::absolute(root_path).string();
            auto ftime     = std::filesystem::last_write_time(root_path);
            auto sctp      = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                    ftime - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
            root_info.created = std::chrono::duration_cast<std::chrono::milliseconds>(sctp.time_since_epoch()).count();
            root_info.type    = std::filesystem::is_directory(root_path) ? 1 : 0;
            root_info.size    = root_info.type == 1 ? 4096 : std::filesystem::file_size(root_path);
            root_info.level   = 1;
            root_info.parent  = std::filesystem::absolute(root_path).parent_path().string();

            file_list.push_back(root_info);

            std::function<void(const std::filesystem::path &, int)> traverse_directory =
                    [&](const std::filesystem::path &current_path, int current_level) {
                        try {
                            for (const auto &entry : std::filesystem::directory_iterator(current_path)) {
                                FileInfo info;
                                info.name  = entry.path().filename().string();
                                info.path  = std::filesystem::absolute(entry.path()).string();
                                auto ftime = std::filesystem::last_write_time(entry);
                                auto sctp  = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                                        ftime - std::filesystem::file_time_type::clock::now() +
                                        std::chrono::system_clock::now());
                                info.created =
                                        std::chrono::duration_cast<std::chrono::milliseconds>(sctp.time_since_epoch())
                                                .count();

                                if (std::filesystem::is_directory(entry)) {
                                    info.type = 1;
                                    info.size = 4096;
                                } else {
                                    info.type = 0;
                                    info.size = std::filesystem::file_size(entry);
                                }

                                info.level  = current_level;
                                info.parent = std::filesystem::absolute(current_path).string();
                                file_list.push_back(info);
                                if (info.type == 1) { traverse_directory(entry.path(), current_level + 1); }
                            }
                        } catch (const std::filesystem::filesystem_error &e) {
                            LOG_ERROR("Error accessing: {} - {}", current_path.string(), e.what());
                        }
                    };

            traverse_directory(dir_path, 2);

            return file_list;
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

        CommonRet NotifyDevUploadFile(const std::string &body) {
            LOG_INFO("NotifyDevUploadFile");

            std::thread([this, body]() {
                try {
                    std::string fileName = nlohmann::json::parse(body).at("fileName");
                    std::string filePath = nlohmann::json::parse(body).at("filePath");

                    // get GetConnectInfo every loop, for upload when connect with wifi. todo
                    if (auto connect_info_opt = GetConnectInfo(); !connect_info_opt) {
                        LOG_ERROR("Failed to get cloud platform connect info");
                    } else {
                        uint8_t try_count = 0;
                        while (try_count++ < 3) {
                            auto response = cloud_platform::YJHAuthentication::UploadFileToCloud(
                                    connect_info_opt, filePath,
                                    app_interfaces::msg::CloudUploadType::UP_LOAD_LOG_OR_BAG_FILE);

                            if (response.has_value()) {
                                auto result        = nlohmann::json::parse(response.value());
                                nlohmann::json res = nlohmann::json::object();
                                res["timestamp"]   = std::chrono::duration_cast<std::chrono::seconds>(
                                                           std::chrono::system_clock::now().time_since_epoch())
                                                           .count();
                                res["messageId"]                                        = common_utils::func::getUUID();
                                res["properties"]["syncFileFinishedNotice"]["fileName"] = fileName;
                                res["properties"]["syncFileFinishedNotice"]["url"] =
                                        result.at("url").get<std::string>();
                                res["properties"]["syncFileFinishedNotice"]["fileId"] =
                                        result.at("fileId").get<std::string>();

                                node_to_iot_client_ptr_->PubToIot(
                                        "/properties/report", app_interfaces::msg::NodeToIot::MQTT_PUB_FAIL_RETRY, res);
                                break;
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        }
                    }


                } catch (const std::exception &e) { LOG_ERROR("syncConfigFinishedNotice exception: {}", e.what()); }
            }).detach();

            return {CommonRet::CommonStatus::SUCCESS, nlohmann::json::object()};
        }
    };
};// namespace app_nodes::files_upload_mgr