#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>
#include <list>
#include <set>
#include <tuple>
#include <utility>
#include "global.hpp"
#include "uuid.hpp"

// inline void readBuffer(const char* buf, uint16_t& offset, UUID& data)
// {
//     UUID tmpData;
//     memcpy(&tmpData, buf + offset, sizeof(UUID));
//     offset += sizeof(UUID);
//     data = UUID{EndianSwap<>::swap(tmpData.hi),
//     EndianSwap<>::swap(tmpData.lo)};
// }

// inline void writeBuffer(const UUID& data, std::string& buf)
// {
//     auto tmpData
//         = UUID{EndianSwap<>::swap(data.hi), EndianSwap<>::swap(data.lo)};
//     buf.append((char*)&tmpData, sizeof(UUID));
// }

// inline char* writeBuffer(const UUID& data, char* buf)
// {
//     auto tmpData
//         = UUID{EndianSwap<>::swap(data.hi), EndianSwap<>::swap(data.lo)};
//     memcpy(buf, (char*)&tmpData, sizeof(UUID));
//     return buf + sizeof(UUID);
// }

// inline uint16_t CalculateSize(const UUID& data)
// {
//     return sizeof(data);
// }

inline void readBuffer(const char* buf, uint16_t& offset, std::string& data)
{
    uint16_t* len_ptr = (uint16_t*)(buf + offset);
    auto lenBigEndian = EndianSwap<>::swap(*len_ptr);
    offset += sizeof(uint16_t);
    data = std::string(buf + offset, lenBigEndian);
    offset += lenBigEndian;
}

inline char* writeBuffer(const std::string& data, char* buf)
{
    auto lenBigEndian = EndianSwap<>::swap((uint16_t)data.size());
    memcpy(buf, (char*)&lenBigEndian, sizeof(uint16_t));
    memcpy(buf + sizeof(uint16_t), data.data(), data.size());
    return buf + sizeof(uint16_t) + data.size();
}

inline const uint16_t CalculateSize(const std::string& data)
{
    return data.size() + sizeof(uint16_t);
}

template <uint16_t N>
inline void readBuffer(const char* buf, uint16_t& offset, char (&data)[N])
{
    uint16_t* len_ptr = (uint16_t*)(buf + offset);
    offset += sizeof(uint16_t);
    const auto lenBigEndian = EndianSwap<>::swap(*len_ptr);
    if (lenBigEndian > N)
    {
        throw std::runtime_error(
            "serialized char array exceeds destination size");
    }

    memcpy(data, buf + offset, lenBigEndian);
    offset += lenBigEndian;
}

template <uint16_t N>
inline char* writeBuffer(const char (&data)[N], char* buf)
{
    auto lenBigEndian = EndianSwap<>::swap(N);
    memcpy(buf, (char*)&lenBigEndian, sizeof(uint16_t));
    memcpy(buf + sizeof(uint16_t), data, N);
    return buf + sizeof(uint16_t) + N;
}

template <uint16_t N>
inline const uint16_t CalculateSize(const char (&data)[N])
{
    return N + sizeof(uint16_t);
}

template <typename T>
struct has_serialize
{
private:
    template <typename C>
    static constexpr auto check(
        C*) -> decltype(std::declval<C>().serialize(), std::true_type{});

    template <typename C>
    static constexpr auto check_char(C*)
        -> decltype(std::declval<const C&>().serialize(std::declval<char*>()),
            std::true_type{});

    template <typename>
    static constexpr std::false_type check(...);

    template <typename>
    static constexpr std::false_type check_char(...);

public:
    static constexpr bool value = decltype(check<T>(nullptr))::value
        || decltype(check_char<T>(nullptr))::value;
};

template <typename T>
inline void readBuffer(const char* buf, uint16_t& offset, T& data)
{
    if constexpr (has_serialize<T>::value)
    {
        offset += data.deserialize(buf + offset);
    }
    else
    {
        T tmpData;
        memcpy(&tmpData, buf + offset, sizeof(T));
        offset += sizeof(T);
        data = EndianSwap<>::swap(tmpData);
    }
}

template <typename T>
inline char* writeBuffer(const T& data, char* buf)
{
    if constexpr (has_serialize<T>::value)
    {
        return data.serialize(buf);
    }
    else
    {
        auto tmpData = EndianSwap<>::swap(data);
        memcpy(buf, (char*)&tmpData, sizeof(data));
        return buf + sizeof(data);
    }
}

template <typename T>
inline const uint16_t CalculateSize(const T& data)
{
    if constexpr (has_serialize<T>::value)
    {
        return data.CalculateSize();
    }
    else
    {
        return sizeof(data);
    }
}

template <typename... Args>
inline void readBuffer(const char* buf,
    uint16_t& offset,
    std::tuple<Args...>& data)
{
    std::apply(
        [&buf, &offset](auto&... values) {
            (readBuffer(buf, offset, values), ...);
        },
        data);
}

template <typename... Args>
inline char* writeBuffer(const std::tuple<Args...>& data, char* buf)
{
    char* cursor = buf;
    std::apply(
        [&cursor](const auto&... values) {
            ((cursor = writeBuffer(values, cursor)), ...);
        },
        data);
    return cursor;
}

template <typename... Args>
inline const uint16_t CalculateSize(const std::tuple<Args...>& data)
{
    uint16_t size = 0;
    std::apply(
        [&size](const auto&... values) {
            ((size += CalculateSize(values)), ...);
        },
        data);
    return size;
}

template <typename T>
inline void readBuffer(const char* buf, uint16_t& offset, std::set<T>& data)
{
    uint16_t* len_ptr = (uint16_t*)(buf + offset);
    auto lenBigEndian = EndianSwap<>::swap(*len_ptr);
    offset += sizeof(uint16_t);
    for (uint16_t i = 0; i < lenBigEndian; ++i)
    {
        T item;
        readBuffer(buf, offset, item);
        data.insert(item);
    }
}

template <typename T>
inline char* writeBuffer(const std::set<T>& data, char* buf)
{
    auto lenBigEndian = EndianSwap<>::swap(uint16_t(data.size()));
    memcpy(buf, (char*)&lenBigEndian, sizeof(uint16_t));
    char* p = buf + sizeof(uint16_t);
    for (const auto& item : data)
        p = writeBuffer(item, p);
    return p;
}

template <typename T>
inline const uint16_t CalculateSize(const std::set<T>& data)
{
    uint16_t size = 0;
    for (auto& item : data)
        size += CalculateSize(item);
    return size + sizeof(uint16_t);
}

template <typename T>
inline void readBuffer(const char* buf, uint16_t& offset, std::vector<T>& data)
{
    uint16_t* len_ptr = (uint16_t*)(buf + offset);
    auto lenBigEndian = EndianSwap<>::swap(*len_ptr);
    offset += sizeof(uint16_t);
    for (uint16_t i = 0; i < lenBigEndian; ++i)
    {
        T item;
        readBuffer(buf, offset, item);
        data.emplace_back(item);
    }
}

template <typename T>
inline char* writeBuffer(const std::vector<T>& data, char* buf)
{
    auto lenBigEndian = EndianSwap<>::swap(uint16_t(data.size()));
    memcpy(buf, (char*)&lenBigEndian, sizeof(uint16_t));
    char* p = buf + sizeof(uint16_t);
    for (auto& item : data)
        p = writeBuffer(item, p);
    return p;
}

template <typename T>
inline const uint16_t CalculateSize(const std::vector<T>& data)
{
    uint16_t size = 0;
    for (auto& item : data)
        size += CalculateSize(item);
    return size + sizeof(uint16_t);
}

template <typename T>
char* SerializeImpl(char* buf, const T& arg)
{
    return writeBuffer(arg, buf);
}

template <typename First, typename... Rest>
char* SerializeImpl(char* buf, const First& first, const Rest&... rest)
{
    buf = SerializeImpl(buf, first);
    return SerializeImpl(buf, rest...);
}

template <typename T>
uint16_t DeserializeImpl(const char* buf, uint16_t& offset, T& arg)
{
    readBuffer(buf, offset, arg);
    return offset;
}

template <typename First, typename... Rest>
uint16_t DeserializeImpl(const char* buf,
    uint16_t& offset,
    First& first,
    Rest&... rest)
{
    DeserializeImpl(buf, offset, first);
    DeserializeImpl(buf, offset, rest...);
    return offset;
}

template <typename... Rest>
uint16_t const CalculateSizeImpl(Rest&... rest)
{
    return (::CalculateSize(rest) + ...);
}

#define GEN_SERIALIZE(...)                                                  \
    char* serialize(char* _serialization_buffer) const                      \
    {                                                                       \
        return SerializeImpl(_serialization_buffer, __VA_ARGS__);           \
    }                                                                       \
    std::string serialize() const                                           \
    {                                                                       \
        std::string _serialization_result(CalculateSize(), '\0');           \
        SerializeImpl(_serialization_result.data(), __VA_ARGS__);           \
        return _serialization_result;                                       \
    }                                                                       \
    uint16_t deserialize(const char* _deserialization_buffer)               \
    {                                                                       \
        uint16_t _deserialization_offset = 0;                               \
        DeserializeImpl(                                                    \
            _deserialization_buffer, _deserialization_offset, __VA_ARGS__); \
        return _deserialization_offset;                                     \
    }                                                                       \
    uint16_t CalculateSize() const                                          \
    {                                                                       \
        return CalculateSizeImpl(__VA_ARGS__);                              \
    }

// #define GEN_SERIALIZE_WITH_NAME(CLASS_NAME, ...)  \
//     static CLASS_NAME FromBuffer(const char* buf) \
//     {                                             \
//         CLASS_NAME obj;                           \
//         obj.deserialize(buf);                     \
//         return obj;                               \
//     }                                             \
//     GEN_SERIALIZE(__VA_ARGS__)
