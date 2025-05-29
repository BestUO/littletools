#include "interface_pkg/msg/sample_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sample_server/SampleServer.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("server");
    node->declare_parameter("message_count", 10);
    node->declare_parameter("server_id", 0);
    int message_count = node->get_parameter("message_count").as_int();
    int server_id     = node->get_parameter("server_id").as_int();

    RCLCPP_INFO(node->get_logger(), "Starting Server %d with message count: %d", server_id, message_count);

    auto subscription = node->create_subscription<interface_pkg::msg::SampleMsg>(
        "sample_topic", 10,
        [server_id, node](const interface_pkg::msg::SampleMsg::SharedPtr msg)
        {
            RCLCPP_INFO(node->get_logger(), "[Server %d] Received message: %s, ID: %d, Frame: %s", server_id, msg->message.c_str(), msg->id,
                        msg->header.frame_id.c_str());
        });

    SampleServer sample_server(node);
    sample_server.Start();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}