#pragma once

#include <any>
#include <type_traits>
#include <ostream>
class VoidType
{
public:
    friend std::ostream& operator<<(std::ostream& out, const VoidType&)
    {
        out << "VoidType";
        return out;
    }
};

template <typename T>
class AnyWrapper
{
public:
    AnyWrapper() = default;
    AnyWrapper(T value)
        : _value(value)
    { }

    T GetValue()
    {
        return _value;
    }

private:
    T _value;
};

template <>
class AnyWrapper<void>
{
public:
    AnyWrapper() = default;
    auto GetValue()
    {
        return _value;
    }

private:
    VoidType _value;
};