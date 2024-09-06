
#pragma once
#include <string>
#include <inttypes.h>
#include <thread>

class UUID
{
public:
    // Generate a version 4 UUID
    static UUID gen()
    {
        std::uint64_t hi
            = ((std::uint64_t)rand() << 31) | (std::uint64_t)rand();
        std::uint64_t lo
            = ((std::uint64_t)rand() << 31) | (std::uint64_t)rand();
        const std::uint64_t t = rand();
        hi ^= (t << 62);
        lo ^= ((t >> 2) << 62);
        lo ^= static_cast<std::uint64_t>(
            std::hash<std::thread::id>{}(std::this_thread::get_id()));
        return UUID{hi, lo};
    }

    bool operator==(const UUID& r) const
    {
        return hi == r.hi && lo == r.lo;
    }
    bool operator!=(const UUID& r) const
    {
        return hi != r.hi || lo != r.lo;
    }
    bool operator<(const UUID& r) const
    {
        return hi == r.hi ? lo < r.lo : hi < r.hi;
    }
    std::size_t operator()(const UUID& uuid) const
    {
        return hash();
    }
    std::string toString() const
    {
        char buf[64] = {0};
        snprintf(buf, sizeof(buf), "%016" PRIx64 "-%016" PRIx64, hi, lo);
        return buf;
    }
    std::uint64_t hash() const
    {
        return hi ^ lo;
    }

    std::uint64_t hi = 0;
    std::uint64_t lo = 0;
};

template <>
struct std::hash<UUID>
{
    std::size_t operator()(const UUID& uuid) const noexcept
    {
        return uuid.hash();
    }
};