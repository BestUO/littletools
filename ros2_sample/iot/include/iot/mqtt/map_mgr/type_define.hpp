#pragma once
#include "common_utils/json_utils/json_converter.h"
#include <nlohmann_json_ros/json.hpp>

namespace iot::mqtt::map_mgr {
    struct TestA {
        int a;
    };
    JSON_CONVERTER(TestA, value) { add_field("a", &value.a); }

    struct TestB {
        TestA a;
        std::string value;
    };
    JSON_CONVERTER(TestB, value) {
        add_field("TestA", &value.a);
        add_field("value", &value.value);
    }
}// namespace iot::mqtt::map_mgr
