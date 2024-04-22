#pragma once

#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>
#include <list>
#include <set>
#include "template.hpp"
#include "uuid.hpp"

template <typename T>
struct has_serialize
{
private:
    template <typename C>
    static constexpr auto check(C*)
        -> decltype(std::declval<C>().serialize(), std::true_type{});

    template <typename>
    static constexpr std::false_type check(...);

public:
    static constexpr bool value = decltype(check<T>(nullptr))::value;
};

template <typename T, std::enable_if_t<!has_serialize<T>::value, int> = 0>
inline void readBuffer(const char* buf, uint16_t& offset, T& data)
{
    uint16_t len = 0;
    T tmpData;
    memcpy(&len, buf + offset, sizeof(uint16_t));
    auto lenBigEndian = EndianSwap<>::swap(len);
    offset += sizeof(uint16_t);
    memcpy(&tmpData, buf + offset, lenBigEndian);
    offset += lenBigEndian;
    data = EndianSwap<>::swap(tmpData);
}

template <typename T, std::enable_if_t<has_serialize<T>::value, int> = 0>
inline void readBuffer(const char* buf, uint16_t& offset, T& data)
{
    uint16_t len = 0;
    memcpy(&len, buf + offset, sizeof(uint16_t));
    auto lenBigEndian = EndianSwap<>::swap(len);
    offset += sizeof(uint16_t);
    data.deserialize(buf + offset, lenBigEndian);
    offset += lenBigEndian;
}

template <typename T, std::enable_if_t<!has_serialize<T>::value, int> = 0>
inline void writeBuffer(const T& data, uint16_t len, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap(len);
    auto tmpData      = EndianSwap<>::swap(data);
    buf.append((char*)&lenBigEndian, sizeof(uint16_t))
        .append((char*)&tmpData, len);
}

template <typename T, std::enable_if_t<has_serialize<T>::value, int> = 0>
inline void writeBuffer(T& data, uint16_t len, std::string& buf)
{
    (void)len;
    auto s            = data.serialize();
    auto lenBigEndian = EndianSwap<>::swap((uint16_t)s.size());
    buf.append((char*)&lenBigEndian, sizeof(uint16_t)).append(s);
}

template <>
inline void readBuffer(const char* buf, uint16_t& offset, UUID& data)
{
    uint16_t len = 0;
    UUID tmpData;
    memcpy(&len, buf + offset, sizeof(uint16_t));
    auto lenBigEndian = EndianSwap<>::swap(len);
    offset += sizeof(uint16_t);
    memcpy(&tmpData, buf + offset, lenBigEndian);
    offset += lenBigEndian;
    data = UUID{
        EndianSwap<>::swap(tmpData.hi64()), EndianSwap<>::swap(tmpData.lo64())};
}

template <>
inline void writeBuffer(const UUID& data, uint16_t len, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap(len);
    auto tmpData      = UUID{
        EndianSwap<>::swap(data.hi64()), EndianSwap<>::swap(data.lo64())};
    buf.append((char*)&lenBigEndian, sizeof(uint16_t))
        .append((char*)&tmpData, len);
}

template <>
inline void readBuffer(const char* buf, uint16_t& offset, std::string& data)
{
    uint16_t len = 0;
    memcpy(&len, buf + offset, sizeof(uint16_t));
    auto lenBigEndian = EndianSwap<>::swap(len);
    offset += sizeof(uint16_t);
    data = std::string(buf + offset, lenBigEndian);
    offset += lenBigEndian;
}

template <>
inline void writeBuffer(const std::string& data, uint16_t len, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap(len);
    buf.append((char*)&lenBigEndian, sizeof(uint16_t)).append(data);
}

template <typename T, std::enable_if_t<has_serialize<T>::value, int> = 0>
inline void readBuffer(const char* buf, uint16_t& offset, std::set<T>& data)
{
    uint16_t len = 0;
    memcpy(&len, buf + offset, sizeof(uint16_t));
    auto lenBigEndian = EndianSwap<>::swap(len);
    offset += sizeof(uint16_t);
    for (uint16_t i = 0; i < lenBigEndian; ++i)
    {
        std::string str;
        readBuffer(buf, offset, str);
        data.insert(T(str.data(), str.size()));
    }
}

template <typename T, std::enable_if_t<!has_serialize<T>::value, int> = 0>
inline void readBuffer(const char* buf, uint16_t& offset, std::set<T>& data)
{
    uint16_t len = 0;
    memcpy(&len, buf + offset, sizeof(uint16_t));
    auto lenBigEndian = EndianSwap<>::swap(len);
    offset += sizeof(uint16_t);
    for (uint16_t i = 0; i < lenBigEndian; ++i)
    {
        T item;
        readBuffer(buf, offset, item);
        data.insert(item);
    }
}

template <typename T, std::enable_if_t<has_serialize<T>::value, int> = 0>
inline void writeBuffer(const std::set<T>& data, uint16_t len, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap(len);
    buf.append((char*)&lenBigEndian, sizeof(uint16_t));
    for (const auto& item : data)
    {
        std::string structcontext = item.serialize();
        writeBuffer(structcontext, structcontext.size(), buf);
    }
}

template <typename T, std::enable_if_t<!has_serialize<T>::value, int> = 0>
inline void writeBuffer(const std::set<T>& data, uint16_t len, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap(len);
    buf.append((char*)&lenBigEndian, sizeof(uint16_t));
    for (const auto& item : data)
    {
        writeBuffer(item, sizeof(item), buf);
    }
}

template <typename T, std::enable_if_t<has_serialize<T>::value, int> = 0>
inline void readBuffer(const char* buf, uint16_t& offset, std::vector<T>& data)
{
    uint16_t len = 0;
    memcpy(&len, buf + offset, sizeof(uint16_t));
    auto lenBigEndian = EndianSwap<>::swap(len);
    offset += sizeof(uint16_t);
    for (uint16_t i = 0; i < lenBigEndian; ++i)
    {
        std::string str;
        readBuffer(buf, offset, str);
        data.emplace_back(T(str.data(), str.size()));
    }
}

template <typename T, std::enable_if_t<!has_serialize<T>::value, int> = 0>
inline void readBuffer(const char* buf, uint16_t& offset, std::vector<T>& data)
{
    uint16_t len = 0;
    memcpy(&len, buf + offset, sizeof(uint16_t));
    auto lenBigEndian = EndianSwap<>::swap(len);
    offset += sizeof(uint16_t);
    for (uint16_t i = 0; i < lenBigEndian; ++i)
    {
        T item;
        readBuffer(buf, offset, item);
        data.emplace_back(item);
    }
}

template <typename T, std::enable_if_t<has_serialize<T>::value, int> = 0>
inline void writeBuffer(std::vector<T>& data, uint16_t len, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap(len);
    buf.append((char*)&lenBigEndian, sizeof(uint16_t));
    for (auto& item : data)
    {
        std::string structcontext = item.serialize();
        writeBuffer(structcontext, structcontext.size(), buf);
    }
}

template <typename T, std::enable_if_t<!has_serialize<T>::value, int> = 0>
inline void writeBuffer(std::vector<T>& data, uint16_t len, std::string& buf)
{
    auto lenBigEndian = EndianSwap<>::swap(len);
    buf.append((char*)&lenBigEndian, sizeof(uint16_t));
    for (auto& item : data)
    {
        writeBuffer(item, sizeof(item), buf);
    }
}