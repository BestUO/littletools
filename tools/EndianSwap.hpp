#pragma once
#include <iostream>

template <typename T>
constexpr T maxOfTwo(T a, T b)
{
    return (a > b) ? a : b;
}

template <typename T>
constexpr T maxOfTotalArgs(T arg)
{
    return arg;
}

template <typename T, typename... Args>
constexpr T maxOfTotalArgs(T arg, Args... args)
{
    T currentMax = maxOfTotalArgs(args...);
    return maxOfTwo(arg, currentMax);
}

template <bool ISLITTLEENDIAN = __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__>
class EndianSwap
{
public:
    template <typename T>
    static T swap(T value)
    {

        if constexpr (!ISLITTLEENDIAN)
        {
            return value;
        }
        else
        {
            T result;
            char* src = reinterpret_cast<char*>(&value);
            char* dst = reinterpret_cast<char*>(&result);
            for (size_t i = 0; i < sizeof(T); ++i)
            {
                dst[i] = src[sizeof(T) - i - 1];
            }
            return result;
        }
    };
};