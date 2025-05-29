#include <iostream>

#include "interface_pkg/msg/sample_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sample_client/SampleClient.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("client");
    node->declare_parameter("message_count", 10);
    node->declare_parameter("client_id", 0);
    int message_count = node->get_parameter("message_count").as_int();
    int client_id     = node->get_parameter("client_id").as_int();
    RCLCPP_INFO(node->get_logger(), "Starting client %d with message count: %d", client_id, message_count);
    SampleClient sample_client(node);
    sample_client.start();
    rclcpp::spin(node);
    // send_cancel_thread.join();
    rclcpp::shutdown();
    return 0;
}