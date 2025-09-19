#pragma once
#include "behaviortree_cpp/action_node.h"
#include <behaviortree_cpp/basic_types.h>

// 自定义枚举
enum class Direction { NORTH, SOUTH, EAST, WEST };

// 自定义字符串映射 - 使用 convertFromString 模板特化
template<>
[[nodiscard]] inline Direction BT::convertFromString<Direction>(BT::StringView str) {
    if (str == "NORTH" || str == "UP") { return Direction::NORTH; }
    if (str == "SOUTH" || str == "DOWN") { return Direction::SOUTH; }
    if (str == "EAST") { return Direction::EAST; }
    if (str == "WEST") { return Direction::WEST; }

    throw BT::RuntimeError(std::string("Cannot convert this to Direction: ") + static_cast<std::string>(str));
}

class North : public BT::SyncActionNode {
public:
    North(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        printf("Robot is moving NORTH\n");
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() { return {}; }
};

class South : public BT::SyncActionNode {
public:
    South(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        printf("Robot is moving SOUTH\n");
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() { return {}; }
};

class East : public BT::SyncActionNode {
public:
    East(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        printf("Robot is moving EAST\n");
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() { return {}; }
};

class West : public BT::SyncActionNode {
public:
    West(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        printf("Robot is moving WEST\n");
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() { return {}; }
};

class EnumTestDefault : public BT::SyncActionNode {
public:
    EnumTestDefault(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        printf("Robot is moving EnumTestDefault\n");
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() { return {}; }
};