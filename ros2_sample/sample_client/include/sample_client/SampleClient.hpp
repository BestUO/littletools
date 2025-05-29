#pragma once
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <rclcpp/subscription.hpp>

#include "action_msgs/srv/cancel_goal.hpp"
#include "interface_pkg/action/sample_action.hpp"
#include "interface_pkg/msg/sample_msg.hpp"
#include "interface_pkg/srv/sample_srv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "test_util.h"

class SampleClient
{
   public:
    SampleClient(rclcpp::Node::SharedPtr node) : __node(node)
    {
        TestUtil a;
        std::cout << "TestUtil Subtract:" << a.Subtract(1, 2) << std::endl;
    }

    ~SampleClient() = default;

    void start()
    {
        __node->declare_parameter("action_name", "sample_action");
        std::string action_name = __node->get_parameter("action_name").as_string();

        __subscribe     = __node->create_subscription<interface_pkg::msg::SampleMsg>("sample_topic", 10,
                                                                                     std::bind(&SampleClient::SubCB, this, std::placeholders::_1));
        __srv_client    = __node->create_client<interface_pkg::srv::SampleSrv>("sample_service");
        __action_client = rclcpp_action::create_client<interface_pkg::action::SampleAction>(__node, action_name);
        while (!__srv_client->wait_for_service(std::chrono::seconds(1)) || !__action_client->wait_for_action_server(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(__node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(__node->get_logger(), "Service not available, waiting again...");
        }

        RCLCPP_INFO(__node->get_logger(), "SampleClient initialized with srv and action clients");
    }

   private:
    rclcpp::Node::SharedPtr __node;
    rclcpp::Client<interface_pkg::srv::SampleSrv>::SharedPtr __srv_client;
    rclcpp::Subscription<interface_pkg::msg::SampleMsg>::SharedPtr __subscribe;
    rclcpp_action::Client<interface_pkg::action::SampleAction>::SharedPtr __action_client;
    std::map<rclcpp_action::GoalUUID,
             std::pair<rclcpp_action::ClientGoalHandle<interface_pkg::action::SampleAction>::SharedPtr, rclcpp::TimerBase::SharedPtr>>
        active_goals_;

    void SubCB(const interface_pkg::msg::SampleMsg::SharedPtr msg)
    {
        RCLCPP_INFO(__node->get_logger(), "Received message: %s, ID: %d, Frame: %s", msg->message.c_str(), msg->id, msg->header.frame_id.c_str());
        if (msg->id % 2 == 0)
        {
            async_call_service(msg->id, msg->id + 1);
        }
        else if (msg->id % 3 == 0)
        {
            send_action_goal("SampleTask", msg->id);
        }
    }

    void async_call_service(int a, int b)
    {
        auto request = std::make_shared<interface_pkg::srv::SampleSrv::Request>();
        request->a   = a;
        request->b   = b;

        RCLCPP_INFO(__node->get_logger(), "Sending srv request: %d + %d", a, b);
        __srv_client->async_send_request(request, std::bind(&SampleClient::ResponseCB, this, std::placeholders::_1));
    }

    void ResponseCB(rclcpp::Client<interface_pkg::srv::SampleSrv>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(__node->get_logger(), "Received srv response: %d", response->sum);
    }

    bool send_action_goal(const std::string& task_name, int max_steps)
    {
        if (!__action_client->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(__node->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg      = interface_pkg::action::SampleAction::Goal();
        goal_msg.task_name = task_name;
        goal_msg.max_steps = max_steps;

        RCLCPP_INFO(__node->get_logger(), "Sending action goal: task=%s, max_steps=%d", task_name.c_str(), max_steps);

        auto send_goal_options = rclcpp_action::Client<interface_pkg::action::SampleAction>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&SampleClient::goal_response_callback, this, std::placeholders::_1, std::chrono::milliseconds(3000));
        send_goal_options.feedback_callback = std::bind(&SampleClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback   = std::bind(&SampleClient::result_callback, this, std::placeholders::_1);

        __action_client->async_send_goal(goal_msg, send_goal_options);
        return true;
    }

    void goal_response_callback(typename rclcpp_action::ClientGoalHandle<interface_pkg::action::SampleAction>::SharedPtr goal_handle,
                                std::chrono::milliseconds timeout)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(__node->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(__node->get_logger(), "Goal accepted by server, waiting for result");
            auto goal_id           = goal_handle->get_goal_id();
            auto timer             = __node->create_wall_timer(timeout,
                                                               [this, goal_id]()
                                                               {
                                                       auto it = active_goals_.find(goal_id);
                                                       if (it != active_goals_.end())
                                                       {
                                                           RCLCPP_WARN(__node->get_logger(), "Goal timeout so canceling");
                                                           this->cancel_specific_goal(it->second.first);
                                                           it->second.second->cancel();
                                                           active_goals_.erase(it);
                                                       }
                                                   });
            active_goals_[goal_id] = std::make_pair(goal_handle, timer);
        }
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<interface_pkg::action::SampleAction>::WrappedResult& result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(__node->get_logger(), "Action succeeded! Final step: %d", result.result->final_step);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(__node->get_logger(), "Action was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(__node->get_logger(), "Action was canceled");
                break;
            default:
                RCLCPP_ERROR(__node->get_logger(), "Unknown result code");
                break;
        }
        if (auto it = active_goals_.find(result.goal_id); it != active_goals_.end())
        {
            it->second.second->cancel();
            active_goals_.erase(it);
        }
    }

    void feedback_callback(rclcpp_action::ClientGoalHandle<interface_pkg::action::SampleAction>::SharedPtr,
                           const std::shared_ptr<const interface_pkg::action::SampleAction::Feedback> feedback)
    {
        RCLCPP_INFO(__node->get_logger(), "Received feedback: current_step = %d", feedback->current_step);
    }

    void cancel_specific_goal(rclcpp_action::ClientGoalHandle<interface_pkg::action::SampleAction>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_WARN(__node->get_logger(), "No active goal to cancel");
            return;
        }

        RCLCPP_INFO(__node->get_logger(), "Canceling current action goal");

        __action_client->async_cancel_goal(goal_handle, std::bind(&SampleClient::CancelCallback, this, std::placeholders::_1));
    }

    void CancelCallback(std::shared_ptr<action_msgs::srv::CancelGoal_Response> response)
    {
        if (response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE)
        {
            RCLCPP_INFO(__node->get_logger(), "Goal canceled successfully");
        }
        else
        {
            RCLCPP_ERROR(__node->get_logger(), "Failed to cancel goal, error code: %d", response->return_code);
        }
    }
};