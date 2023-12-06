#pragma once
#include <iostream>

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