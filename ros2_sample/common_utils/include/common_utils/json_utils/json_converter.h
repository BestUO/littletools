/**
 * ===============================================================
 *  @copyright Copyright (c) yijiahe Co. Ltd. All Rights Reserved.
 *  @file json_converter.h
 *  @author fanlong
 *  @date 2025-07-16 
 *  @Description 自定义宏，自动生成json转换函数from_json, to_json，支持嵌套,值得推广
 * ===============================================================
 */

#ifndef JSON_CONVERTER_H
#define JSON_CONVERTER_H

#include "nlohmann_json_ros/json.hpp"
#include <optional>
#include <string>

namespace common_utils {
//宏定义
#define JSON_CONVERTER(Type, value)                                                                                    \
    template<class AddField>                                                                                           \
    void _JsonTypeDefinition(Type &, AddField &);                                                                      \
                                                                                                                       \
    inline void to_json(nlohmann::json &js, const Type &p) {                                                           \
        auto op = [&js](const char *name, auto *val) { js[name] = *val; };                                             \
        _JsonTypeDefinition(const_cast<Type &>(p), op);                                                                \
        js["__type"] = #Type;                                                                                          \
    }                                                                                                                  \
                                                                                                                       \
    inline void from_json(const nlohmann::json &js, Type &p) {                                                         \
        auto op = [&js](const char *name, auto *v) { js.at(name).get_to(*v); };                                        \
        _JsonTypeDefinition(p, op);                                                                                    \
    }                                                                                                                  \
                                                                                                                       \
    template<class AddField>                                                                                           \
    inline void _JsonTypeDefinition(Type &value, AddField &add_field)

    template<typename T>
    std::optional<T> GetJsonValue(const nlohmann::json &js, const std::string &key) {
        if (js.contains(key) && !js[key].is_null()) { return js[key].get<T>(); }
        return std::nullopt;
    }
}// namespace common_utils

#endif//JSON_CONVERTER_H
