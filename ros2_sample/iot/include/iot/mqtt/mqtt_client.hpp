#pragma once

#include <nlohmann/json.hpp>
#include <tuple>

#include "fun_executor.hpp"
#include "mqtt_callback_base.hpp"
#include "yjh_mqtt/yjh_mqtt.h"

namespace iot::mqtt
{
template <CALLBACKBASE... Args>
class MqttClient
{
   public:
    MqttClient(const char *id) : client_(app_nodes::mqtt_wrap::MqttWrapImpl::CreateMosquittoClient(id)), classes_callback_(Args()...)
    {
        std::apply([this](auto... class_callback) { (this->fun_executor_.Merge(std::move(class_callback.GetFunExecutor())), ...); },
                   classes_callback_);
        client_->SetMessageCallback(std::bind(&MqttClient::Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    ~MqttClient()
    {
    }

    void Connect(std::string &&host, int port, int keepalive)
    {
        client_->Connect(std::move(host), port, keepalive);
        std::apply(
            [this](auto &&...callbacks)
            {
                (
                    [&]
                    {
                        for (const auto &topic : callbacks.GetSubscribedTopics())
                        {
                            client_->Subscribe(topic.c_str());
                        }
                    }(),
                    ...);
            },
            classes_callback_);
    }

    void Disconnect()
    {
        client_->Disconnect();
    }

   private:
    FunExecutor fun_executor_;
    std::unique_ptr<app_nodes::mqtt_wrap::MosquittoClient> client_;
    std::tuple<Args...> classes_callback_;

    void Callback(const char *topic, int payloadlen, uint8_t *payload)
    {
        printf("topic: %s Received len:%d payload: %s\n", topic, payloadlen, payload);
        auto j = nlohmann::json::parse(payload);
        fun_executor_.InvokeFunction<void>(topic, j);
    }
};
}  // namespace iot::mqtt