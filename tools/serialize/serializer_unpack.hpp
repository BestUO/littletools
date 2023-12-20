#pragma once
#include <sys/types.h>
#include <cstddef>
#include <iostream>
#include <type_traits>
#include <stdlib.h>
#include <charconv>

#include "concept_template.hpp"
#include "serializer_type.hpp"

namespace serialize
{

class SerializeString
{
public:
    SerializeString(std::string_view s)
        : __s(s)
    { }

    std::string_view GetNextValueString()
    {
        std::string_view lenstr = __s.substr(__pos, 2);
        size_t len              = 0;
        __s.copy((char*)&len, 2, __pos);
        auto tmp = __pos;
        __pos += len;
        return __s.substr(tmp, len);
    }

private:
    std::string_view __s;
    size_t __pos = 0;
};

template <typename T>
void SetValueInStruct(T&& item, SerializeString& s)
{
    using type = std::remove_cvref_t<decltype(item)>;
    if constexpr (
        std::is_integral_v<
            type> || std::is_same_v<float, type> || std::is_same_v<double, type>)
    {
        auto segment = s.GetNextValueString();
        TypeId tid;
        segment.copy((char*)&tid, sizeof(TypeId), sizeof(uint16_t));
        segment.copy(
            (char*)&item, GetDataSize(tid), sizeof(uint16_t) + sizeof(TypeId));
    }
    else if constexpr (map_container<type>)
    {
        auto segment = s.GetNextValueString();
        uint16_t mapsize;
        segment.copy((char*)&mapsize,
            sizeof(uint16_t),
            sizeof(uint16_t) + sizeof(TypeId));

        typename type::key_type a;
        typename type::mapped_type b;
        item.clear();
        for (uint16_t i = 0; i < mapsize; i++)
        {
            SetValueInStruct(a, s);
            SetValueInStruct(b, s);
            item.insert({a, b});
        }
    }
    else if constexpr (string<type>)
    {
        auto segment = s.GetNextValueString();
        size_t stringlen;
        segment.copy((char*)&stringlen,
            sizeof(uint16_t),
            sizeof(uint16_t) + sizeof(TypeId));
        item = segment.substr(sizeof(uint16_t) + sizeof(TypeId), stringlen);
    }
    else if constexpr (std::is_class_v<type>)
    {
        VisitorStructMember(item, [&s](auto&&... items) {
            SerializeString serializestring(s);
            (SetValueInStruct(items, serializestring), ...);
        });
    }
}

template <typename T>
T Deserialize(std::string_view s)
{
    T t{};
    VisitorStructMember(t, [&s](auto&&... items) {
        SerializeString serializestring(s);
        (SetValueInStruct(items, serializestring), ...);
    });
    return t;
}
}  // namespace serialize