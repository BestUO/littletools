#include "bt_test/bt_test.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "bt_test/enum_test.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/controls/switch_node.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

int main() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<North>("North");
    factory.registerNodeType<South>("South");
    factory.registerNodeType<East>("East");
    factory.registerNodeType<West>("West");
    factory.registerNodeType<EnumTestDefault>("EnumTestDefault");


    const std::string package_share_dir = ament_index_cpp::get_package_share_directory("bt_test");
    const std::string tree_xml          = package_share_dir + "/config/bt_test.xml";

    auto tree             = factory.createTreeFromFile(tree_xml);
    auto publisher_groot2 = std::make_shared<BT::Groot2Publisher>(tree, 1667);

    while (true) {
        auto status = tree.tickOnce();
        if (status != BT::NodeStatus::RUNNING) { break; }

        std::string value;
        (void) tree.rootBlackboard()->get("text", value);
        //sleep is asynchronous, so the output may be a little more
        // printf("ThinkWhatToSay:%s\n", value.c_str());
    }
    return 0;
}