#pragma once
#include "type_define.hpp"
#include <nlohmann/json.hpp>

namespace app_nodes::files_upload_mgr {
    template<typename T>
    class ClassBase {
    public:
        virtual ~ClassBase() = default;
        template<typename... Args>
        CommonRet Call(Args &&...args) {
            return static_cast<T *>(this)->CallImpl(std::forward<Args>(args)...);
        }

        template<typename... Args>
        CommonRet CallImpl(Args &&...args) {
            (void) std::initializer_list<int>{((void) args, 0)...};
            return {CommonRet::CommonStatus::NOT_SUPPORTED_YET, nlohmann::json::object()};
        }
    };
}// namespace app_nodes::files_upload_mgr