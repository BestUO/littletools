#pragma once

#include <stddef.h>
#include <stdint.h>

template <size_t BITSET_SIZE = 64>
class Bitset
{
public:
    using BITSET_TYPE                     = uint32_t;
    static constexpr size_t BITSET_M_SIZE = sizeof(BITSET_TYPE) * 8;
    static constexpr size_t BITSET_ARRAY_SIZE
        = (BITSET_SIZE + BITSET_M_SIZE - 1) / BITSET_M_SIZE;
    using BITSET_ARRAY = BITSET_TYPE[BITSET_ARRAY_SIZE];

    bool GetBit(size_t index) const
    {
        return GetBit(__bitset, index);
    }

    static bool GetBit(const BITSET_ARRAY& bitset, size_t index)
    {
        if (index < BITSET_SIZE)
        {
            return (bitset[index / BITSET_M_SIZE]
                       & (1U << (index % BITSET_M_SIZE)))
                != 0;
        }
        return false;
    }

    void SetBit(size_t index)
    {
        SetBit(__bitset, index);
    }

    static void SetBit(BITSET_ARRAY& bitset, size_t index)
    {
        if (index < BITSET_SIZE)
        {
            bitset[index / BITSET_M_SIZE] |= (1U << (index % BITSET_M_SIZE));
        }
    }

    void ClearBit(size_t index)
    {
        ClearBit(__bitset, index);
    }

    static void ClearBit(BITSET_ARRAY& bitset, size_t index)
    {
        if (index < BITSET_SIZE)
        {
            bitset[index / BITSET_M_SIZE] &= ~(1U << (index % BITSET_M_SIZE));
        }
    }

    static void ClearAllBit(BITSET_ARRAY& bitset)
    {
        for (size_t i = 0; i < BITSET_ARRAY_SIZE; ++i)
        {
            bitset[i] = 0;
        }
    }

    int32_t GetZeroIndex() const
    {
        for (size_t i = 0; i < BITSET_ARRAY_SIZE; ++i)
        {
            BITSET_TYPE mask;
            if (i == BITSET_ARRAY_SIZE - 1 && BITSET_SIZE % BITSET_M_SIZE != 0)
                mask = (1U << (BITSET_SIZE % BITSET_M_SIZE)) - 1;
            else
                mask = static_cast<BITSET_TYPE>(-1);

            BITSET_TYPE inverted = ~__bitset[i] & mask;
            if (inverted != 0)
            {
                int bit_pos  = __builtin_ffs(inverted);
                size_t index = (i * BITSET_M_SIZE) + (bit_pos - 1);
                return static_cast<int32_t>(index);
            }
        }

        return -1;
    }

    int32_t PopCount() const
    {
        return PopCount(__bitset);
    }

    static int32_t PopCount(const BITSET_ARRAY& bitset)
    {
        int32_t count = 0;
        for (size_t i = 0; i < BITSET_ARRAY_SIZE; ++i)
        {
            count += __builtin_popcount(bitset[i]);
        }
        return count;
    }

    bool IsEmpty() const
    {
        return IsEmpty(__bitset);
    }

    static bool IsEmpty(const BITSET_ARRAY& bitset)
    {
        int32_t count = 0;
        for (size_t i = 0; i < BITSET_ARRAY_SIZE; ++i)
        {
            if (bitset[i] != 0)
                return false;
        }
        return true;
    }

    void BitsetCopy(BITSET_ARRAY& bitset) const
    {
        for (size_t i = 0; i < BITSET_ARRAY_SIZE; ++i)
        {
            bitset[i] = __bitset[i];
        }
    }

private:
    BITSET_ARRAY __bitset = {0};
};