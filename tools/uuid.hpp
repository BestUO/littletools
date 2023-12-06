
#pragma once
#include <string>
#include <inttypes.h>

class UUID
{
public:
    // Generate a version 4 UUID
    static UUID gen()
    {
        UUID r;
        r.m_a = ((std::uint64_t)rand() << 31) | (std::uint64_t)rand();
        r.m_b = ((std::uint64_t)rand() << 31) | (std::uint64_t)rand();
        const std::uint64_t t = rand();
        r.m_a ^= (t << 62);
        r.m_b ^= ((t >> 2) << 62);
        return r;
    }

    UUID();
    constexpr UUID(std::uint64_t a, std::uint64_t b)
        : m_a(a)
        , m_b(b)
    { }

    bool operator==(const UUID& r) const
    {
        return m_a == r.m_a && m_b == r.m_b;
    }
    bool operator!=(const UUID& r) const
    {
        return m_a != r.m_a || m_b != r.m_b;
    }
    bool operator<(const UUID& r) const
    {
        return m_a == r.m_a ? m_b < r.m_b : m_a < r.m_a;
    }
    std::string toString() const
    {
        char buf[64] = {0};
        snprintf(buf, sizeof(buf), "%016" PRIx64 "-%016" PRIx64, m_a, m_b);
        return buf;
    }
    std::uint64_t hash() const
    {
        return m_a ^ m_b;
    }

private:
    std::uint64_t m_a = 0;
    std::uint64_t m_b = 0;
};
