#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>
#include <list>
#include <set>
#include "template.hpp"
#include "uuid.hpp"

inline void readBuffer(const char* buf, uint16_t& offset, UUID& data)
{
    UUID tmpData;
    memcpy(&tmpData, buf + offset, sizeof(UUID));
    offset += sizeof(UUID);
    data = UUID{EndianSwap<>::swap(tmpData.hi), EndianSwap<>::swap(tmpData.lo)};
}

inline void writeBuffer(const UUID& data, std::string& buf)
{
    auto tmpData
        = UUID{EndianSwap<>::swap(data.hi), EndianSwap<>::swap(data.lo)};
    buf.append((char*)&tmpData, sizeof(UUID));
}

inline char* writeBuffer(const UUID& data, char* buf)
{
    auto tmpData
        = UUID{EndianSwap<>::swap(data.hi), EndianSwap<>::swap(data.lo)};
    memcpy(buf, (char*)&tmpData, sizeof(UUID));
    return buf + sizeof(UUID);
}

inline uint16_t CalculateSize(const UUID& data)
{
    return sizeof(data);
}

inline void readBuffer(const char* buf, uint16_t& offset, std::string& data)
{
    uint16_t* len_ptr = (uint16_t*)(buf + offset);
    auto lenBigEndian = EndianSwap<>::swap(*len_ptr);
    offset += sizeof(uint16_t);
    data = std::string(buf + offset, lenBigEndian);
    offset += lenBigEndian;
}

inline void writeBuffer(const std::string& data, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap((uint16_t)data.size());
    buf.append((char*)&lenBigEndian, sizeof(uint16_t)).append(data);
}

inline char* writeBuffer(const std::string& data, char* buf)
{
    auto lenBigEndian = EndianSwap<>::swap((uint16_t)data.size());
    memcpy(buf, (char*)&lenBigEndian, sizeof(uint16_t));
    memcpy(buf + sizeof(uint16_t), data.data(), data.size());
    return buf + sizeof(uint16_t) + data.size();
}

inline uint16_t CalculateSize(const std::string& data)
{
    return data.size() + sizeof(uint16_t);
}

inline void writeBuffer(std::string_view data, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap((uint16_t)data.size());
    buf.append((char*)&lenBigEndian, sizeof(uint16_t)).append(data);
}

inline char* writeBuffer(std::string_view data, char* buf)
{
    auto lenBigEndian = EndianSwap<>::swap((uint16_t)data.size());
    memcpy(buf, (char*)&lenBigEndian, sizeof(uint16_t));
    memcpy(buf + sizeof(uint16_t), data.data(), data.size());
    return buf + sizeof(uint16_t) + data.size();
}

inline uint16_t CalculateSize(std::string_view data)
{
    return data.size() + sizeof(uint16_t);
}

inline void readBuffer(const char* buf, uint16_t& offset, char* data)
{
    uint16_t* len_ptr = (uint16_t*)(buf + offset);
    auto lenBigEndian = EndianSwap<>::swap(*len_ptr);
    offset += sizeof(uint16_t);
    memcpy(data, buf + offset, lenBigEndian);
    offset += lenBigEndian;
}

template <uint16_t N>
inline void writeBuffer(const char (&data)[N], std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap(N);
    buf.append((char*)&lenBigEndian, sizeof(uint16_t)).append(data, N);
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
inline uint16_t CalculateSize(const char (&data)[N])
{
    return N + sizeof(uint16_t);
}

template <typename T>
struct has_serialize
{
private:
    template <typename C>
    static constexpr auto check(C*)
        -> decltype(std::declval<C>().serialize(), std::true_type{});

    template <typename C>
    static constexpr auto check(C*)
        -> decltype(std::declval<C>().serialize(std::declval<char*>()),
            std::declval<char*>(),
            std::true_type{});

    template <typename>
    static constexpr std::false_type check(...);

public:
    static constexpr bool value = decltype(check<T>(nullptr))::value;
};

template <typename T, std::enable_if_t<!has_serialize<T>::value, int> = 0>
inline void readBuffer(const char* buf, uint16_t& offset, T& data)
{
    T tmpData;
    memcpy(&tmpData, buf + offset, sizeof(T));
    offset += sizeof(T);
    data = EndianSwap<>::swap(tmpData);
}

template <typename T, std::enable_if_t<!has_serialize<T>::value, int> = 0>
inline void writeBuffer(const T& data, std::string& buf)
{
    auto tmpData = EndianSwap<>::swap(data);
    buf.append((char*)&tmpData, sizeof(data));
}

template <typename T, std::enable_if_t<!has_serialize<T>::value, int> = 0>
inline char* writeBuffer(const T& data, char* buf)
{
    auto tmpData = EndianSwap<>::swap(data);
    memcpy(buf, (char*)&tmpData, sizeof(data));
    return buf + sizeof(data);
}

template <typename T, std::enable_if_t<!has_serialize<T>::value, int> = 0>
inline uint16_t CalculateSize(const T& data)
{
    return sizeof(data);
}

template <typename T, std::enable_if_t<has_serialize<T>::value, int> = 0>
inline void readBuffer(const char* buf, uint16_t& offset, T& data)
{
    offset += data.deserialize(buf + offset);
}

template <typename T, std::enable_if_t<has_serialize<T>::value, int> = 0>
inline void writeBuffer(const T& data, std::string& buf)
{
    auto s = data.serialize();
    buf.append(s);
}

template <typename T, std::enable_if_t<has_serialize<T>::value, int> = 0>
inline char* writeBuffer(const T& data, char* buf)
{
    return data.serialize(buf);
}

template <typename T, std::enable_if_t<has_serialize<T>::value, int> = 0>
inline uint16_t CalculateSize(T& data)
{
    return data.CalculateSize();
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
inline void writeBuffer(const std::set<T>& data, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap(uint16_t(data.size()));
    buf.append((char*)&lenBigEndian, sizeof(uint16_t));
    for (const auto& item : data)
        writeBuffer(item, buf);
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
inline uint16_t CalculateSize(const std::set<T>& data)
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
inline void writeBuffer(const std::vector<T>& data, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap((uint16_t)data.size());
    buf.append((char*)&lenBigEndian, sizeof(uint16_t));
    for (auto& item : data)
        writeBuffer(item, buf);
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
inline uint16_t CalculateSize(const std::vector<T>& data)
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
uint16_t CalculateSizeImpl(Rest&... rest)
{
    return (::CalculateSize(rest) + ...);
}

#define GEN_SERIALIZE(...)                         \
    char* serialize(char* buf) const               \
    {                                              \
        return SerializeImpl(buf, __VA_ARGS__);    \
    }                                              \
    uint16_t deserialize(const char* buf)          \
    {                                              \
        uint16_t offset;                           \
        DeserializeImpl(buf, offset, __VA_ARGS__); \
        return offset;                             \
    }                                              \
    uint16_t CalculateSize()                       \
    {                                              \
        return CalculateSizeImpl(__VA_ARGS__);     \
    }
