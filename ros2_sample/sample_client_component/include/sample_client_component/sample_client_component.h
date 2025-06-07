#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "interface_pkg/msg/sample_msg.hpp"

namespace sample_client_component
{

class SampleClientComponent : public rclcpp::Node
{
   public:
    explicit SampleClientComponent(const rclcpp::NodeOptions& options);

   private:
    void message_callback(const interface_pkg::msg::SampleMsg::SharedPtr msg);

    rclcpp::Subscription<interface_pkg::msg::SampleMsg>::SharedPtr subscription_;
};

}  // namespace sample_client_component