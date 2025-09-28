#include "iot/http/request_dispatcher.h"
#include "jsoncpp/json/json.h"

namespace iot::http {
    RequestDispatcher::RequestDispatcher(SocketChannel socket_channel, SocketChannelEventLoop &event_loop,
                                         ThreadExecutor &executor)
        : HttpFullRequestSession(std::move(socket_channel), event_loop, executor) {

        //方案1：默认所有的请求都交由HandleRequest统一分发处理，按url匹配度优先级，查找最优匹配的handler处理请求
        MessageHandle call_back = [this](BytesMessage &request) -> bool { return HandleRequest(request); };
        RegisterHandle("POST", "/*", call_back);
        RegisterHandle("GET", "*", call_back);
    }

    RequestDispatcher::~RequestDispatcher() = default;

    void RequestDispatcher::Initialize(const std::vector<std::shared_ptr<RequestHandler>> &request_handlers) {
        request_handlers_ = request_handlers;

        //方案2:注册http请求的处理器，http库只允许所有url中存在一个url包含通配符*，超过1个时，多个带通配符的url选择的优先级不固定
        // for (auto &handler: request_handlers_) {
        //     //请求处理回调lambda
        //     MessageHandle call_back = [this, handler](BytesMessage &request)-> bool {
        //         return handler->HandleRequest(request, this);
        //     };
        //
        //     auto urls = handler->GetUrls();
        //     for (const auto &url: urls) {
        //         RegisterHandle("GET", url, call_back);
        //         RegisterHandle("POST", url, call_back);
        //     }
        // }
    }

    void RequestDispatcher::SendResponse(int code, const std::string &msg) {
        Json::Value res_body;
        res_body["code"] = code;
        res_body["msg"]  = msg;
        res_body["data"] = Json::Value();

        std::string content = res_body.toStyledString();

        HttpFullResponse response(200, "OK", "HTTP/1.1");
        auto content_length =
                response.content().content().WriteBytesFrom((uint8_t *) content.c_str(), content.length());
        response.headers().SetHeaderValue("Connection", "close");
        response.headers().SetHeaderValue("Content-Type", "application/json");
        response.headers().SetHeaderValue("Content-Length", std::to_string(content_length));

        std::string message = response.ToString();
        WriteHttpResponse(response, [this] { this->PostKillEvent(); });
    }

    void RequestDispatcher::SendResponse(const std::string res_data) {

        HttpFullResponse response(200, "OK", "HTTP/1.1");
        auto content_length =
                response.content().content().WriteBytesFrom((uint8_t *) res_data.c_str(), res_data.length());
        response.headers().SetHeaderValue("Connection", "close");
        response.headers().SetHeaderValue("Content-Type", "application/json");
        response.headers().SetHeaderValue("Content-Length", std::to_string(content_length));

        std::string message = response.ToString();
        WriteHttpResponse(response, [this] { this->PostKillEvent(); });
    }

    void RequestDispatcher::SendPngData(const std::vector<uint8_t> &res_data) {

        HttpFullResponse response(200, "OK", "HTTP/1.1");
        auto content_length = response.content().content().WriteBytesFrom(res_data.data(), res_data.size());
        response.headers().SetHeaderValue("Connection", "close");
        response.headers().SetHeaderValue("Content-Type", "image/png");
        response.headers().SetHeaderValue("Content-Length", std::to_string(content_length));

        std::string message = response.ToString();
        WriteHttpResponse(response, [this] { this->PostKillEvent(); });
    }


    void RequestDispatcher::SendBadRequestResponse() {
        HttpFullResponse response(404, "Bad Request", "HTTP/1.1");
        response.headers().SetHeaderValue("Connection", "close");

        std::string message = response.ToString();
        WriteHttpResponse(response, [&, message] {
            printf("HTTP RESPONSE: %s", message.c_str());
            this->PostKillEvent();
        });
    }

    bool RequestDispatcher::HandleRequest(BytesMessage &request) {
        auto &full_request = dynamic_cast<HttpFullRequest &>(request);

        //查找url匹配度最高的handler
        auto url     = full_request.request_line().GetUriPath();
        auto handler = FindBestMatchingHandler(url);

        //找不到匹配的handler，返回404
        if (nullptr == handler) {
            std::cout << "Request handler not found for url: " << url << std::endl;
            SendBadRequestResponse();
            return true;
        }
        return handler->HandleRequest(request, this);
    }

    std::shared_ptr<RequestHandler> RequestDispatcher::FindBestMatchingHandler(const std::string &url) const {
        std::string matched_url;
        size_t max_length                            = 0;
        std::shared_ptr<RequestHandler> best_handler = nullptr;
        //查找匹配度最高的url，完全匹配时直接返回对应的handler
        for (const auto &handler : request_handlers_) {
            for (const auto &pattern : handler->GetUrls()) {
                if (pattern == url) { return handler; }//完全匹配
                if (pattern.back() == '*') {           //通配符匹配
                    size_t len = pattern.size() - 1;
                    if (len > url.size()) { continue; }
                    if (memcmp(pattern.data(), url.data(), len) == 0 && len > max_length) {
                        max_length   = len;
                        best_handler = handler;
                    }
                }
            }
        }
        return best_handler;
    }
}// namespace iot::http
