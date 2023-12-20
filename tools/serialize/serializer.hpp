#pragma once
#include <iostream>
#include "serializer_pack.hpp"
#include "serializer_unpack.hpp"
namespace serialize
{

class SimpleSerializer
{
public:
    template <typename T>
    static std::string Serialize(T&& t)
    {
        return serialize::Serialize(t);
    }

    template <typename T>
    static T Deserialize(std::string&& s)
    {
        return serialize::Deserialize<T>(s);
    }
};
}  // namespace serialize