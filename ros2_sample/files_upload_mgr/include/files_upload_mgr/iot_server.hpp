#pragma once

#include "app_interfaces/srv/iot_server_info.hpp"
#include "config_upload.hpp"
#include "type_define.hpp"
#include "yjhlog/logger.h"
#include <rclcpp/rclcpp.hpp>

namespace app_nodes::files_upload_mgr {
    template<typename... Args>
    class IOTServer {
    public:
        IOTServer(std::shared_ptr<NodeBaseInfo> node_base_info)
            : node_(node_base_info->node), clients_(std::make_unique<Args>(node_base_info)...),
              config_upload_(node_base_info) {
            iot_to_file_upload_mgr_service_ = node_->create_service<app_interfaces::srv::IotServerInfo>(
                    "iot_to_files_upload_mgr",
                    std::bind(&IOTServer::HandleCommonRequest<app_interfaces::srv::IotServerInfo>, this,
                              std::placeholders::_1, std::placeholders::_2),
                    rclcpp::ServicesQoS(), node_base_info->service_cb_group);
            LOG_INFO("IOTServer::IOTServer(std::shared_ptr<NodeBaseInfo> node_base_info)");
        };
        ~IOTServer() { LOG_INFO("IOTServer::~IOTServer()"); }

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::tuple<std::unique_ptr<Args>...> clients_;
        rclcpp::Service<app_interfaces::srv::IotServerInfo>::SharedPtr iot_to_file_upload_mgr_service_;
        ConfigUpload config_upload_;

        template<typename T>
        void HandleCommonRequest(const std::shared_ptr<typename T::Request> request,
                                 std::shared_ptr<typename T::Response> response) {
            CallClients(request, response);
        }

        template<typename... Fun_Args>
        CommonRet CallClients(Fun_Args &&...args) {
            CommonRet final_result(CommonRet::CommonStatus::NOT_SUPPORTED_YET);
            std::apply(
                    [&](auto &...client) {
                        (([&] {
                             auto r = client->Call(std::forward<Fun_Args>(args)...);
                             if (r.code != CommonRet::CommonStatus::NOT_SUPPORTED_YET) { final_result = r; }
                         }()),
                         ...);
                    },
                    clients_);
            return final_result;
        }
    };
}// namespace app_nodes::files_upload_mgr