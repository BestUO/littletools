#pragma once

#include <sys/types.h>
#include <cstddef>
#include <iostream>
#include <type_traits>

enum TypeId : uint8_t
{
    NONE,
    UINT8_T,
    UINT16_T,
    UINT32_T,
    UINT64_T,
    INT8_T,
    INT16_T,
    INT32_T,
    INT64_T,
    FLOAT,
    DOUBLE,
    MAP_CONTAINER,
    STRING
};

template <typename T>
TypeId GetIntergralTypeId()
{
    if constexpr (std::is_same_v<uint8_t, T>)
    {
        return UINT8_T;
    }
    else if constexpr (std::is_same_v<uint16_t, T>)
    {
        return UINT16_T;
    }
    else if constexpr (std::is_same_v<uint32_t, T>)
    {
        return UINT32_T;
    }
    else if constexpr (std::is_same_v<uint64_t, T>)
    {
        return UINT64_T;
    }
    else if constexpr (std::is_same_v<int8_t, T>)
    {
        return INT8_T;
    }
    else if constexpr (std::is_same_v<int16_t, T>)
    {
        return INT16_T;
    }
    else if constexpr (std::is_same_v<int32_t, T>)
    {
        return INT32_T;
    }
    else if constexpr (std::is_same_v<int64_t, T>)
    {
        return INT64_T;
    }
    else if constexpr (std::is_same_v<float, T>)
    {
        return FLOAT;
    }
    else if constexpr (std::is_same_v<double, T>)
    {
        return DOUBLE;
    }
    else
    {
        return NONE;
        static_assert("GetIntergralTypeId error");
    }
}

inline uint8_t GetDataSize(TypeId tid)
{
    switch (tid)
    {
    case UINT8_T:
        return 1;
    case UINT16_T:
        return 2;
    case UINT32_T:
        return 4;
    case UINT64_T:
        return 8;
    case INT8_T:
        return 1;
    case INT16_T:
        return 2;
    case INT32_T:
        return 4;
    case INT64_T:
        return 8;
    case FLOAT:
        return 4;
    case DOUBLE:
        return 8;
    default:
        return 0;
    }
}
