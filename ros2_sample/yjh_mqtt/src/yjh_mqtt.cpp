#include "yjh_mqtt/yjh_mqtt.h"
#include <cstdio>
#include <rclcpp/rclcpp.hpp>

namespace app_nodes {
    namespace mqtt_wrap {
        MosquittoClient::MosquittoClient(const char *id) : mosqpp::mosquittopp(id) {}

        bool MosquittoClient::Connect(std::string &&host, int port, int keepalive) {
            if (connect(host.c_str(), port, keepalive) != MOSQ_ERR_SUCCESS) {
                RCLCPP_INFO(rclcpp::get_logger("MqttWrapImpl"), "MosquittoClient connect failed");
                return false;
            }
            loop_start();
            return true;
        }

        bool MosquittoClient::Disconnect() {
            disconnect();
            loop_stop();
            return true;
        }

        bool MosquittoClient::Publish(const char *topic, int payloadlen, const void *payload, bool retain) {
            return publish(nullptr, topic, payloadlen, payload, 1, retain) == MOSQ_ERR_SUCCESS;
        }

        bool MosquittoClient::Subscribe(const char *sub) { return subscribe(nullptr, sub, 1) == MOSQ_ERR_SUCCESS; }

        void MosquittoClient::on_connect(int rc) {
            if (rc) {
                RCLCPP_INFO(rclcpp::get_logger("MqttWrapImpl"), "Mosquitto connect failed, rc=%s",
                            mosquitto_connack_string(rc));
            } else {
                RCLCPP_INFO(rclcpp::get_logger("MqttWrapImpl"), "Mosquitto connect success");
            }
        }

        void MosquittoClient::on_disconnect(int rc) {
            RCLCPP_INFO(rclcpp::get_logger("MqttWrapImpl"), "Mosquitto disconnected, rc=%s",
                        mosquitto_connack_string(rc));
        }

        void MosquittoClient::on_log(int level, const char *str) {
            if (level != MOSQ_LOG_DEBUG) {
                RCLCPP_INFO(rclcpp::get_logger("MqttWrapImpl"), "Mosquitto log [level=%d]: %s", level, str);
            }
        };

        void MosquittoClient::on_message(const struct mosquitto_message *msg) {
            RCLCPP_INFO(rclcpp::get_logger("MqttWrapImpl"), "Mosquitto message [mid=%d, qos=%d]", msg->mid, msg->qos);
            if (message_callback_) { message_callback_(msg->topic, msg->payloadlen, (uint8_t *) msg->payload); }
        }

        MqttWrapImpl::MqttWrapImpl() { mosqpp::lib_init(); }

        MqttWrapImpl::~MqttWrapImpl() { mosqpp::lib_cleanup(); }

        bool MqttWrapImpl::MqttMgrInit() { return true; }

        std::unique_ptr<MosquittoClient> MqttWrapImpl::CreateMosquittoClient(const char *id) {
            return std::unique_ptr<MosquittoClient>(new MosquittoClient(id));
        }
    }// namespace mqtt_wrap
}// namespace app_nodes