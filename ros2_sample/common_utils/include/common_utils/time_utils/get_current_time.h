/**
 * ===============================================================
 *  @copyright Copyright (c) yijiahe Co. Ltd. All Rights Reserved.
 *  @file get_current_time_s.h
 *  @author fanlong
 *  @date 2025-08-27 
 *  @Description 获取当前时间
 * ===============================================================
 */

#ifndef BUILD_GET_CURRENT_TIME_H
#define BUILD_GET_CURRENT_TIME_H

#include <chrono>
#include <cstdint>

namespace common_utils {
    //获取当前时间，单位秒
    inline int64_t GetCurrentTimeS() {
        auto now = std::chrono::system_clock::now();
        auto unix_timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        return unix_timestamp;
    }

    inline  int64_t GetCurrentTimeMs() {
        auto now = std::chrono::system_clock::now();
        auto unix_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        return unix_timestamp;
    }

}
#endif //BUILD_GET_CURRENT_TIME_S_H