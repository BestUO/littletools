
#include "iot/mqtt/map_mgr/mqtt_callback.hpp"
#include "iot/mqtt/mqtt_client.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("iot");
    RCLCPP_INFO(node->get_logger(), "begin init");

    // 启动mqtt客户端
    iot::mqtt::MqttClient<iot::mqtt::map_mgr::MqttCallback> client("test_client");
    client.Connect("localhost", 1883, 60);

    // auto a = app_nodes::mqtt_wrap::MqttWrapImpl::CreateMosquittoClient("test_client");
    // a->SetMessageCallback([](const char *topic, int payloadlen, uint8_t *payload) {
    //     printf("topic: %s Received len:%d\n", topic, payloadlen);
    // });
    // a->Connect("127.0.0.1", 1883, 60);
    // a->Subscribe("test");

    RCLCPP_INFO(node->get_logger(), "init success");
    rclcpp::spin(node);
    rclcpp::shutdown();
    client.Disconnect();
    return 0;
}
