#include "iot/http/request_handler/internal_handler.h"
#include "common_utils/string_utils/string_utils.h"
#include "iot/http/request_dispatcher.h"
#include "iot/iot_manager.h"
#include "nlohmann/json.hpp"
#include "lim/transport/http/http_request_session.h"

namespace iot::http {

    InternalHandler::InternalHandler(std::shared_ptr<rclcpp::Node> node) : RequestHandler(node) {
        // 设置支持处理的url集合
        std::unordered_set<std::string> allowed_methods = {
                "/cmd/*",
        };
        SetUrls(allowed_methods);
        
        // 初始化命令映射
        command_mapping_ = {
            {"ping", "startup_status"},
            {"delayed_sleep", "active_confirmation"}
        };
    }

    InternalHandler::~InternalHandler() = default;

    bool InternalHandler::HandleRequest(BytesMessage &request, RequestDispatcher *dispatcher) {
        auto &full_request = dynamic_cast<HttpFullRequest &>(request);

        // 读取http请求参数
        std::string request_param = GetRequestParam(full_request);

        // 切割url，判断具体的处理命令
        const std::string url_path          = full_request.request_line().GetUriPath();
        std::vector<std::string> tokenizers = common_utils::SplitString(url_path, '/');
        if (tokenizers.size() < 2) {
            // URL格式不符合规范，返回404错误
            RCLCPP_ERROR(node_->get_logger(), "url[%s] is invalid, return 404", url_path.c_str());
            dispatcher->SendBadRequestResponse();
            return true;
        }

        // 获取请求类型
        const std::string &req_type = tokenizers[1];

        if (req_type == "cmd") {
            if (tokenizers.size() < 3) {
                dispatcher->SendBadRequestResponse();
                return true;
            }

            const std::string &command = tokenizers[2];
            return HandleIotManagerRequest(command, request_param, dispatcher);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Unsupported request type: %s", req_type.c_str());
            dispatcher->SendResponse(-1, "Unsupported request type");
            return true;
        }
    }

    bool InternalHandler::HandleIotManagerRequest(std::string_view command, std::string_view request_param,
                                                  RequestDispatcher *dispatcher) {
        // 解析请求参数
        nlohmann::json params;
        if (!request_param.empty()) {
            try {
                params = nlohmann::json::parse(request_param);
            } catch (const nlohmann::json::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to parse JSON request: %s, error: %s", 
                             std::string(request_param).c_str(), e.what());
                dispatcher->SendResponse(-1, "Invalid JSON format");
                return true;
            }
        }

        // 查找命令映射
        auto it = command_mapping_.find(std::string(command));
        if (it == command_mapping_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown command: %s", std::string(command).c_str());
            dispatcher->SendResponse(-1, "Unknown command");
            return true;
        }

        // 调用IotManager处理指令
        nlohmann::json result = iot::IotManager::Instance().HandleCommand(it->second, params);
        
        // 使用响应构造
        nlohmann::json response = {
            {"code", result.contains("success") && result["success"].get<bool>() ? 0 : -1},
            {"msg", result.contains("success") && result["success"].get<bool>() ? 
                    "success" : result.value("error", "Unknown error")},
        };
        
        if (result.contains("success") && result["success"].get<bool>() && result.contains("data")) {
            response["data"] = result["data"];
        }

        dispatcher->SendResponse(response.dump());
        return true;
    }
}// namespace iot::http
