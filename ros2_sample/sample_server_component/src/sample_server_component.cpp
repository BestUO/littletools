#include "sample_server_component/sample_server_component.h"

#include <rclcpp_components/register_node_macro.hpp>

namespace sample_server_component
{

SampleServerComponent::SampleServerComponent(const rclcpp::NodeOptions& options) : Node("sample_server_component", options), message_count_(0)
{
    this->declare_parameter("publish_frequency", 1.0);
    this->declare_parameter("topic_name", "sample_topic");

    double frequency       = this->get_parameter("publish_frequency").as_double();
    std::string topic_name = this->get_parameter("topic_name").as_string();

    publisher_ = this->create_publisher<interface_pkg::msg::SampleMsg>(topic_name, 10);

    auto timer_period = std::chrono::duration<double>(1.0 / frequency);
    timer_            = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
                                                std::bind(&SampleServerComponent::publish_message, this));

    RCLCPP_INFO(this->get_logger(), "SampleServerComponent initialized with frequency: %.2f Hz", frequency);
}

void SampleServerComponent::publish_message()
{
    auto message            = interface_pkg::msg::SampleMsg();
    message.message         = "Hello from SampleServerComponent!";
    message.id              = message_count_++;
    message.header.stamp    = this->get_clock()->now();
    message.header.frame_id = "server_component_frame";

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published message #%d: %s", message.id, message.message.c_str());
}

}  // namespace sample_server_component

RCLCPP_COMPONENTS_REGISTER_NODE(sample_server_component::SampleServerComponent)