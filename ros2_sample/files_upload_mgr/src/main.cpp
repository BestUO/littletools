#include "files_upload_mgr/iot_server.hpp"
#include "files_upload_mgr/iot_simple_handle.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("files_upload_mgr_node");

    const std::string log_config =
            ament_index_cpp::get_package_share_directory("files_upload_mgr") + "/config/logging.properties";
    if (!yjhlog::Initialize(log_config)) return EXIT_FAILURE;//日志初始化

    nlohmann::json config;
    try {
        const std::string file_upload_mgr_config =
                ament_index_cpp::get_package_share_directory("files_upload_mgr") + "/config/config.json";
        std::ifstream ifs(file_upload_mgr_config);
        config = nlohmann::json::parse(ifs, nullptr, false);
    } catch (const std::exception &e) {
        LOG_ERROR("Failed to parse config rule JSON: {}", e.what());
        return EXIT_FAILURE;
    }

    auto node_base_info = std::make_shared<app_nodes::files_upload_mgr::NodeBaseInfo>(
            node, node->create_callback_group(rclcpp::CallbackGroupType::Reentrant),
            node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive), config,
            std::make_shared<iot::NodeToIotClient>(node));


    app_nodes::files_upload_mgr::IOTServer<app_nodes::files_upload_mgr::IotSimpleHandle> file_upload_mgr_iot_server(
            node_base_info);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();

    return 0;
}