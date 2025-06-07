#include "sample_client_component/sample_client_component.h"

#include <rclcpp_components/register_node_macro.hpp>

using std::placeholders::_1;

namespace sample_client_component
{

SampleClientComponent::SampleClientComponent(const rclcpp::NodeOptions& options) : Node("sample_client_component", options)
{
    this->declare_parameter("topic_name", "sample_topic");
    this->declare_parameter("client_id", 0);

    std::string topic_name = this->get_parameter("topic_name").as_string();
    int client_id          = this->get_parameter("client_id").as_int();

    subscription_ =
        this->create_subscription<interface_pkg::msg::SampleMsg>(topic_name, 10, std::bind(&SampleClientComponent::message_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "SampleClientComponent %d initialized, listening to topic: %s", client_id, topic_name.c_str());
}

void SampleClientComponent::message_callback(const interface_pkg::msg::SampleMsg::SharedPtr msg)
{
    int client_id = this->get_parameter("client_id").as_int();
    RCLCPP_INFO(this->get_logger(), "[Client %d] Received message: %s, ID: %d, Frame: %s", client_id, msg->message.c_str(), msg->id,
                msg->header.frame_id.c_str());
}

}  // namespace sample_client_component

RCLCPP_COMPONENTS_REGISTER_NODE(sample_client_component::SampleClientComponent)