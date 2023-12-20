#pragma once
#include <sys/types.h>
#include <cstddef>
#include <iostream>
#include <type_traits>

#include "concept_template.hpp"
#include "serializer_type.hpp"

namespace serialize
{

template <typename T>
std::string ValueToString(T&& item)
{
    using type = std::remove_cvref_t<decltype(item)>;
    std::string s;
    if constexpr (
        std::is_integral_v<
            type> || std::is_same_v<float, type> || std::is_same_v<double, type>)
    {
        auto tid     = GetIntergralTypeId<type>();
        uint16_t len = sizeof(uint16_t) + sizeof(TypeId) + GetDataSize(tid);
        s.append((char*)&len, sizeof(uint16_t));
        s.append((char*)&tid, sizeof(TypeId));
        s.append((char*)&item, sizeof(item));
    }
    else if constexpr (map_container<type>)
    {
        auto tid         = TypeId::MAP_CONTAINER;
        uint16_t mapsize = item.size();
        uint16_t len     = sizeof(uint16_t) + sizeof(TypeId) + sizeof(uint16_t);
        s.append((char*)&len, sizeof(uint16_t));
        s.append((char*)&tid, sizeof(TypeId));
        s.append((char*)&mapsize, sizeof(mapsize));
        for (auto&& element : item)
        {
            s += ValueToString(element.first) + ValueToString(element.second);
        }
    }
    else if constexpr (string<type>)
    {
        auto tid = TypeId::STRING;
        auto len = sizeof(uint16_t) + sizeof(TypeId) + item.size();
        s.append((char*)&len, sizeof(uint16_t));
        s.append((char*)&tid, sizeof(TypeId));
        s.append(item);
    }
    else if constexpr (std::is_class_v<type>)
    {
        s = VisitorStructMember(item, [](auto&&... items) {
            return (ValueToString(items) + ...);
        });
    }
    return s;
}

template <typename T>
std::string Serialize(T&& t)
{
    return VisitorStructMember(t, [](auto&&... items) {
        return (ValueToString(items) + ...);
    });
}
}  // namespace serialize