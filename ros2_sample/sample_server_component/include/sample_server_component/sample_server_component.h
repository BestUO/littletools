#pragma once

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "interface_pkg/msg/sample_msg.hpp"

namespace sample_server_component
{

class SampleServerComponent : public rclcpp::Node
{
   public:
    explicit SampleServerComponent(const rclcpp::NodeOptions& options);

   private:
    void publish_message();

    rclcpp::Publisher<interface_pkg::msg::SampleMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int32_t message_count_;
};

}  // namespace sample_server_component