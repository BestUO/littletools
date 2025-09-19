#pragma once
#include "behaviortree_cpp/action_node.h"
#include <behaviortree_cpp/basic_types.h>
#include <iostream>

class ThinkWhatToSay : public BT::SyncActionNode {
public:
    ThinkWhatToSay(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::OutputPort<std::string>("text"), BT::OutputPort<std::string>("enum_out")};
    }

    // This Action writes a value into the port "text"
    BT::NodeStatus tick() override {
        // the output may change at each tick(). Here we keep it simple.
        setOutput("text", "The answer is 42");
        setOutput("enum_out", "NORTH");

        std::string text;
        if (config().blackboard->get("message", text)) {
            printf("tick text:%s status:%d\n", text.c_str(), (int) status());
        } else {
            printf("tick with empty message, status:%d\n", (int) status());
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class SaySomething : public BT::SyncActionNode {
public:
    // If your Node has ports, you must use this constructor signature
    SaySomething(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config) {}

    // It is mandatory to define this STATIC method.
    static BT::PortsList providedPorts() {
        // This action has a single input port called "text"
        return {BT::InputPort<std::string>("text"), BT::InputPort<std::string>("test")};
    }

    // Override the virtual function tick()
    BT::NodeStatus tick() override {
        BT::Expected<std::string> msg  = getInput<std::string>("text");
        BT::Expected<std::string> test = getInput<std::string>("test");
        // Check if expected is valid. If not, throw its error
        if (!msg) { throw BT::RuntimeError("missing required input [text]: ", msg.error()); }
        // use the method value() to extract the valid text.
        std::cout << "Robot says: " << msg.value() << std::endl;
        std::cout << "test: " << test.value() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};