#include <iostream>

#include "interface_pkg/msg/sample_msg.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/simple_node"});
    auto node    = rclcpp::Node::make_shared("server", options);

    auto publisher    = node->create_publisher<interface_pkg::msg::SampleMsg>("sample_topic", 10);
    auto subscription = node->create_subscription<interface_pkg::msg::SampleMsg>(
        "sample_topic", 10, [](const interface_pkg::msg::SampleMsg::SharedPtr msg)
        { std::cout << "Received message: " << msg->message << ", ID: " << msg->id << ", Frame: " << msg->header.frame_id << std::endl; });

    int i = 10000;
    while (--i)
    {
        auto message            = interface_pkg::msg::SampleMsg();
        message.message         = "Hello from main! Count: " + std::to_string(i);
        message.id              = i;
        message.header.stamp    = node->get_clock()->now();
        message.header.frame_id = "main_frame";

        publisher->publish(message);
        std::cout << "Published: " << message.message << std::endl;

        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "Finished publishing messages" << std::endl;

    // 清理
    rclcpp::shutdown();
    return 0;
}