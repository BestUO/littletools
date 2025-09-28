#pragma once

#include "iot/mqtt/mqtt_callback_base.hpp"
#include "nlohmann_json_ros/json.hpp"
#include "type_define.hpp"

namespace iot::mqtt::map_mgr {
    class MqttCallback : public iot::mqtt::MqttCallbackBase {
    public:
        MqttCallback() {
            RegisterFunction("testa_void", [this](struct TestA t) { this->Testa_void(t); });
            RegisterFunction("testa_int", [this](struct TestA t) { return this->Testa_int(t); });
            RegisterFunction("testb_void", [this](struct TestB t) { this->Testb_void(t); });
            RegisterFunction("testb_int", [this](struct TestB t) { this->Testb_int(t); });
            RegisterFunction("testbconst_void", [this](const TestB& t) { this->Testbconst_void(t); });
            RegisterFunction("testbconst_int", [this](struct TestB t) { this->Testbconst_int(t); });
        }

    private:
        void Testa_void(TestA t) { printf("testa_void: %d\n", t.a); }
        int Testa_int(TestA t) {
            printf("testa_int: %d\n", t.a);
            return t.a;
        }

        void Testb_void(TestB t) { printf("testb_void: %d\n", t.a.a); }
        int Testb_int(TestB t) {
            printf("testb_int: %d\n", t.a.a);
            return t.a.a;
        }

        void Testbconst_void(const TestB &t) { printf("testbconst_void: %d\n", t.a.a); }
        int Testbconst_int(TestB &t) {
            printf("testbconst_int: %d\n", t.a.a);
            return t.a.a;
        }
    };
}// namespace iot::mqtt::map_mgr
