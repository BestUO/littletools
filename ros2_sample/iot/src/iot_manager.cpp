#include "iot/iot_manager.h"
#include <filesystem>
#include <fstream>

namespace iot {

    void IotManager::Initialize(std::shared_ptr<rclcpp::Node> node) {
        node_ = node;

        // 注册默认指令回调函数
        RegisterCommand("startup_status", std::bind_front(&IotManager::GetStartupStatus, this));
        RegisterCommand("active_confirmation", std::bind_front(&IotManager::GetActiveStatus, this));

        RCLCPP_INFO(node_->get_logger(), "IotManager initialized with %zu commands", command_callbacks_.size());
    }

    void IotManager::RegisterCommand(std::string_view command, CommandCallback callback) {
        std::scoped_lock<std::mutex> lock(command_mutex_);
        command_callbacks_[std::string(command)] = std::move(callback);
    }

    nlohmann::json IotManager::HandleCommand(std::string_view command, const nlohmann::json &params) {
        std::scoped_lock<std::mutex> lock(command_mutex_);
        auto it = command_callbacks_.find(std::string(command));
        if (it == command_callbacks_.end()) {
            RCLCPP_WARN(node_->get_logger(), "Unknown command: %s", std::string(command).c_str());
            return {{"success", false}, {"error", "Unknown command: " + std::string(command)}};
        }

        RCLCPP_INFO(node_->get_logger(), "Executing command: %s with params: %s", std::string(command).c_str(),
                    params.dump().c_str());

        try {
            auto result = it->second(params);
            RCLCPP_INFO(node_->get_logger(), "Command %s executed successfully", std::string(command).c_str());
            return result;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "Command execution error: %s, command: %s", e.what(),
                         std::string(command).c_str());
            return {{"success", false}, {"error", std::string(e.what())}};
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown error in command: %s", std::string(command).c_str());
            return {{"success", false}, {"error", "Unknown error occurred"}};
        }
    }

    nlohmann::json IotManager::GetStartupStatus(const nlohmann::json &params) {
        try {
            if (params.contains("hmi") && params["hmi"].is_string()) {
                std::string new_version = params["hmi"].get<std::string>();
                if (hmi_version_ != new_version) {
                    RCLCPP_INFO(node_->get_logger(), "HMI version changed from '%s' to '%s'", hmi_version_.c_str(),
                                new_version.c_str());
                    hmi_version_ = std::move(new_version);
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(node_->get_logger(), "Error processing HMI version: %s", e.what());
        }

        nlohmann::json result;
        result["success"] = true;

        nlohmann::json data;
        data["isFreeze"]    = true;
        data["isActivated"] = true;

        result["data"] = data;
        return result;
    }

    nlohmann::json IotManager::GetActiveStatus(const nlohmann::json &params) {
        [[maybe_unused]] const auto &p = params;

        nlohmann::json result;
        result["success"] = true;

        nlohmann::json data;

        result["data"] = data;
        return result;
    }

}// namespace iot