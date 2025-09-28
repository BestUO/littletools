#pragma once

#include "fun_executor.hpp"
#include <set>

namespace iot::mqtt {
    class MqttCallbackBase {
    public:
        MqttCallbackBase()          = default;
        virtual ~MqttCallbackBase() = default;

        FunExecutor &GetFunExecutor() { return fun_executor_; }
        std::set<std::string> &GetSubscribedTopics() { return subscribed_topics_; }

    protected:
        FunExecutor fun_executor_;
        std::set<std::string> subscribed_topics_;

        template<typename Function>
        void RegisterFunction(const std::string &topic, Function &&func) {
            subscribed_topics_.insert(topic);
            fun_executor_.RegisterFunction(topic, std::forward<Function>(func));
        }
    };

    template<typename T>
    concept CALLBACKBASE = std::derived_from<T, MqttCallbackBase>;
}// namespace iot::mqtt