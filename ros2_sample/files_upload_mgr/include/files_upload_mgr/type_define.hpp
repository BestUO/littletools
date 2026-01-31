#pragma once

#include "common_utils/json_utils/json_converter.h"
#include "magic_enum/magic_enum.hpp"
#include "node_to_iot/node_to_iot.hpp"
#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace app_nodes::files_upload_mgr {
    inline static const std::string files_upload_path    = std::string(std::getenv("HOME")) + "/resource/files_upload/";
    inline static const std::string files_info_path      = files_upload_path + "files_info.json";
    inline static const std::string config_dirs_zip_path = files_upload_path + "config_dirs.zip";
    struct CommonRet {
        enum CommonStatus {
            SUCCESS = 0,
            FAILED,
            NOT_SUPPORTED_YET,
            JSON_ERROR,
            CONNECT_SERVER_FAILED = 20,
            CALL_SERVER_TIMEOUT,
            CALL_SERVER_FAIL,
        };

        CommonRet(CommonStatus code, nlohmann::json &&data = nlohmann::json::object())
            : data(std::move(data)), code(code), msg(magic_enum::enum_name(code)) {}

        CommonRet(CommonStatus code, std::string &&msg, nlohmann::json &&data = nlohmann::json::object())
            : data(std::move(data)), code(code), msg(std::move(msg)) {}

        long timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();
        nlohmann::json data = nlohmann::json::object();
        CommonStatus code   = SUCCESS;
        std::string msg     = "";

        friend void to_json(nlohmann::json &j, const CommonRet &p) {
            j = nlohmann::json{
                    {"timestamp", p.timestamp},
                    {"data", p.data},
                    {"code", p.code},
                    {"msg", p.msg},
            };
        }
    };

    struct DirElement {
        std::string desc;
        std::string module_name;
        std::string command;
        std::string dir_path;
    };
    JSON_CONVERTER(DirElement, value) {
        add_field("desc", &value.desc);
        add_field("module_name", &value.module_name);
        add_field("command", &value.command);
        add_field("dir_path", &value.dir_path);
    }

    struct ConfigElement {
        std::string desc;
        std::vector<DirElement> dirs;
    };
    JSON_CONVERTER(ConfigElement, value) {
        add_field("desc", &value.desc);
        add_field("dirs", &value.dirs);
    }

    struct FileUploadMgrConfig {
        ConfigElement log_dirs;
        ConfigElement config_dirs;
    };
    JSON_CONVERTER(FileUploadMgrConfig, value) {
        add_field("log_dirs", &value.log_dirs);
        add_field("config_dirs", &value.config_dirs);
    }

    struct NodeBaseInfo {
        std::shared_ptr<rclcpp::Node> node;
        rclcpp::CallbackGroup::SharedPtr service_cb_group;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group;
        FileUploadMgrConfig config;
        std::shared_ptr<iot::NodeToIotClient> node_to_iot_client_ptr;
    };

    struct DirInfo {
        std::string dir_path;
        long modify_time;
    };
    JSON_CONVERTER(DirInfo, value) {
        add_field("dir_path", &value.dir_path);
        add_field("modify_time", &value.modify_time);
    }

    struct DirsInfo {
        std::map<std::string, DirInfo> dirs;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(DirsInfo, dirs)
    };

    struct FileInfo {
        std::string name;  //文件名
        std::string path;  //文件绝对路径
        long created;      // timestamp ms
        long size;         //文件大小
        int type;          // 0: file, 1: dir
        int level;         //所属目录层级
        std::string parent;//上级目录绝对路径
    };
    JSON_CONVERTER(FileInfo, value) {
        add_field("created", &value.created);
        add_field("level", &value.level);
        add_field("name", &value.name);
        add_field("parent", &value.parent);
        add_field("path", &value.path);
        add_field("size", &value.size);
        add_field("type", &value.type);
    }
}// namespace app_nodes::files_upload_mgr