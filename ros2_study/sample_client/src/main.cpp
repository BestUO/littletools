#include <iostream>

#include "interface_pkg/msg/sample_msg.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/simple_node"});
    auto node    = rclcpp::Node::make_shared("client", options);

    auto subscription = node->create_subscription<interface_pkg::msg::SampleMsg>(
        "sample_topic", 10, [](const interface_pkg::msg::SampleMsg::SharedPtr msg)
        { std::cout << "Received message: " << msg->message << ", ID: " << msg->id << ", Frame: " << msg->header.frame_id << std::endl; });

    std::cout << "Finished publishing messages" << std::endl;
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}