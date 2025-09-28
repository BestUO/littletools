/**
 * ===============================================================
 *  @copyright Copyright (c) yijiahe Co. Ltd. All Rights Reserved.
 *  @file split_string.h
 *  @author fanlong
 *  @date 2025-07-09 
 *  @Description 实现简单的字符串分割功能
 * ===============================================================
 */

#ifndef SPLIT_STRING_H
#define SPLIT_STRING_H

#include <sstream>
#include <string>
#include <vector>

namespace common_utils {
    /**
     * @brief 简易字符串分割函数
     * @param str 待分割的字符串
     * @param delimiter 分隔符
     * @return 分割后的字符串集合
     */
    std::vector<std::string> SplitString(const std::string &str, const char delimiter);

}


#endif //SPLIT_STRING_H
