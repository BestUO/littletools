#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <thread>

#include "interface_pkg/action/sample_action.hpp"
#include "interface_pkg/msg/sample_msg.hpp"
#include "interface_pkg/srv/sample_srv.hpp"
#include "nlohmann_json_ros/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "utils_ros/test_util.h"

class SampleServer
{
   public:
    SampleServer(rclcpp::Node::SharedPtr node) : __node(node)
    {
        TestUtil a;
        std::cout << "TestUtil add:" << a.Add(1, 2) << std::endl;
        nlohmann::json example = {{"name", "SampleServer"}, {"version", 1.0}, {"active", true}, {"features", {"publisher", "service", "action"}}};
        std::cout << "JSON example: " << example.dump(2) << std::endl;
    }
    ~SampleServer() = default;

    void Start()
    {
        __node->declare_parameter("action_name", "sample_action");
        std::string action_name = __node->get_parameter("action_name").as_string();

        __publisher  = __node->create_publisher<interface_pkg::msg::SampleMsg>("sample_topic", 10);
        __srv_server = __node->create_service<interface_pkg::srv::SampleSrv>(
            "sample_service", std::bind(&SampleServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));
        __action_server = rclcpp_action::create_server<interface_pkg::action::SampleAction>(
            __node, action_name, std::bind(&SampleServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SampleServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&SampleServer::handle_accepted, this, std::placeholders::_1));
        _timer = __node->create_wall_timer(std::chrono::seconds(1), std::bind(&SampleServer::PublishMessage, this));
        RCLCPP_INFO(__node->get_logger(), "SampleServer initialized with action server: %s", action_name.c_str());
    }

   private:
    rclcpp::Node::SharedPtr __node;
    rclcpp::Service<interface_pkg::srv::SampleSrv>::SharedPtr __srv_server;
    rclcpp::Publisher<interface_pkg::msg::SampleMsg>::SharedPtr __publisher;
    rclcpp_action::Server<interface_pkg::action::SampleAction>::SharedPtr __action_server;
    rclcpp::TimerBase::SharedPtr _timer;
    int32_t __index = 0;

    void PublishMessage()
    {
        auto message            = interface_pkg::msg::SampleMsg();
        message.message         = "Hello from SampleServer!";
        message.id              = __index++;  // Example ID
        message.header.stamp    = __node->get_clock()->now();
        message.header.frame_id = "sample_frame";

        __publisher->publish(message);
        RCLCPP_INFO(__node->get_logger(), "Published message: %s", message.message.c_str());
    }

    void handle_service(const std::shared_ptr<interface_pkg::srv::SampleSrv::Request> request,
                        std::shared_ptr<interface_pkg::srv::SampleSrv::Response> response)
    {
        RCLCPP_INFO(__node->get_logger(), "Received srv request: %d %d", request->a, request->b);

        response->sum = request->a + request->b;

        RCLCPP_INFO(__node->get_logger(), "Sending srv response: %d", response->sum);
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const interface_pkg::action::SampleAction::Goal> goal)
    {
        RCLCPP_INFO(__node->get_logger(), "Received action goal request with target: %s %d", goal->task_name.c_str(), goal->max_steps);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<interface_pkg::action::SampleAction>> goal_handle)
    {
        RCLCPP_INFO(__node->get_logger(), "Received request to cancel action goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interface_pkg::action::SampleAction>> goal_handle)
    {
        // todo
        std::thread as_work_pool_thread(std::bind(&SampleServer::execute_action, this, goal_handle));
        as_work_pool_thread.detach();
    }

    void execute_action(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interface_pkg::action::SampleAction>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback   = std::make_shared<interface_pkg::action::SampleAction::Feedback>();
        auto result     = std::make_shared<interface_pkg::action::SampleAction::Result>();

        RCLCPP_INFO(__node->get_logger(), "Executing action goal with target: %s %d", goal->task_name.c_str(), goal->max_steps);

        for (int i = 0; i <= goal->max_steps && rclcpp::ok(); ++i)
        {
            if (goal_handle->is_canceling())
            {
                result->final_step = i;
                goal_handle->canceled(result);
                RCLCPP_INFO(__node->get_logger(), "Action goal canceled");
                return;
            }

            feedback->current_step = i;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(__node->get_logger(), "Feedback: current count = %d", i);

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        if (rclcpp::ok())
        {
            result->final_step = goal->max_steps;
            goal_handle->succeed(result);
            RCLCPP_INFO(__node->get_logger(), "Action goal succeeded with final count: %d", result->final_step);
        }
    }
};