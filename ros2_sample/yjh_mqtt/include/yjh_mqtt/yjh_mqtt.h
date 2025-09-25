#pragma once

#include <functional>
#include <memory>
#include <mosquitto.h>
#include <mosquittopp.h>
#include <string>

namespace app_nodes {
    namespace mqtt_mgr {

        class MosquittoClient : public mosqpp::mosquittopp {
        public:
            MosquittoClient(const MosquittoClient &)            = delete;
            MosquittoClient &operator=(const MosquittoClient &) = delete;
            MosquittoClient(MosquittoClient &&)                 = delete;
            MosquittoClient &operator=(MosquittoClient &&)      = delete;

            ~MosquittoClient() = default;

            bool Connect(std::string &&host, int port, int keepalive);
            bool Disconnect();

            void SetMessageCallback(
                    std::function<void(const char *topic, int payloadlen, uint8_t *payload)> message_callback) {
                message_callback_ = message_callback;
            }

            bool Publish(const char *topic, int payloadlen, const void *payload, bool retain = false);
            bool Subscribe(const char *sub);

        private:
            std::function<void(const char *topic, int payloadlen, uint8_t *payload)> message_callback_;

            explicit MosquittoClient(const char *id);
            void on_connect(int rc);
            void on_disconnect(int rc);
            void on_log(int level, const char *str);
            void on_message(const struct mosquitto_message *msg);
            friend class MqttMgr;
        };

        class MqttMgr {
        public:
            static MqttMgr *Instance() {
                static MqttMgr instance;
                return &instance;
            }

            static std::unique_ptr<MosquittoClient> CreateMosquittoClient(const char *id);
            bool MqttMgrInit();

        private:
            MqttMgr();
            MqttMgr(const MqttMgr &)            = delete;
            MqttMgr &operator=(const MqttMgr &) = delete;
            MqttMgr(MqttMgr &&)                 = delete;
            MqttMgr &operator=(MqttMgr &&)      = delete;
            ~MqttMgr();
        };
    }// namespace mqtt_mgr
}// namespace app_nodes