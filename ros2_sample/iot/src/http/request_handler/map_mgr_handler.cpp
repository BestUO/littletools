#include "iot/http/request_handler/map_mgr_handler.h"
#include "app_interfaces/srv/iot_to_map_mgr.hpp"
#include "common_utils/string_utils/string_utils.h"
#include "iot/http/request_dispatcher.h"
#include "lim/transport/http/http_request_session.h"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>


namespace iot::http {
    MapMgrHandler::MapMgrHandler(std::shared_ptr<rclcpp::Node> node) : RequestHandler(node) {
        //设置支持处理的url集合
        std::unordered_set<std::string> allowed_methods = {
                "/map_mgr/*",
        };
        SetUrls(allowed_methods);
    }

    MapMgrHandler::~MapMgrHandler() = default;

    bool MapMgrHandler::HandleRequest(BytesMessage &request, RequestDispatcher *dispatcher) {
        auto &full_request = dynamic_cast<HttpFullRequest &>(request);

        //读取http请求参数,
        std::string request_param = GetRequestParam(full_request);

        //切割url，构造service调用所需的参数
        const std::string url_path          = full_request.request_line().GetUriPath();
        std::vector<std::string> tokenizers = common_utils::SplitString(url_path, '/');
        if (tokenizers.size() < 3) {
            //小于3说明url异常，不符合与hmi的约定,返回404
            RCLCPP_ERROR(node_->get_logger(), "url[%s] is invalid, return 404 ", url_path.c_str());
            dispatcher->SendBadRequestResponse();
            return true;
        }

        //构造service请求消息
        auto srv_request     = std::make_shared<app_interfaces::srv::IotToMapMgr::Request>();
        srv_request->head    = tokenizers[1];//head：标识归属于哪个业务节点处理
        srv_request->command = tokenizers[2];//command：具体的处理指令，如设备控制，读取状态等
        srv_request->body    = request_param;//body：具体的指令参数

        //定义client的局部变量，并根据url中的head字段拼接service服务名称，用完即释放[性能是个问题]
        const std::string service_name = "iot_to_" + srv_request->head;
        auto common_client             = node_->create_client<app_interfaces::srv::IotToMapMgr>(service_name);

        //调用service
        if (!common_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "service not available for url: %s", url_path.c_str());
            dispatcher->SendResponse(SERVICE_CALL_ERROR_CODE, SERVICE_CALL_ERROR_MSG);//ros服务不可用
            return true;
        }

        auto future_result = common_client->async_send_request(srv_request);
        auto status        = future_result.wait_for(std::chrono::seconds(GetServiceCallTimeout()));
        if (status != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "service call failed for url: %s, status: %d", url_path.c_str(),
                         (int) status);
            dispatcher->SendResponse(SERVICE_CALL_ERROR_CODE, SERVICE_CALL_ERROR_MSG);//ros服务不可用
            return true;
        }


        //发送响应结果
        auto srv_response = future_result.get();

        //png图像请求，有数据则返回png图像数据，否则返回错误
        if (!srv_response->img.data.empty()) {
            auto cv_ptr   = cv_bridge::toCvCopy(srv_response->img, "bgr8");
            cv::Mat image = cv_ptr->image;

            // 2. 编码为 PNG 格式（内存中的二进制数据）
            std::vector<uint8_t> png_buffer;
            cv::imencode(".png", image, png_buffer);
            dispatcher->SendPngData(png_buffer);
            return true;
        }
        dispatcher->SendResponse(srv_response->result);
        return true;
    }
}// namespace iot::http
