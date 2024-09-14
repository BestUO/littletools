#include <iostream>
#include <vector>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/simple_serialize.hpp"

struct Base
{
    int index;
    std::string s;
    std::vector<int> v;
    std::set<int> st;
    char buf[3];
    UUID t;
    std::string serialize()
    {
        std::string context;
        writeBuffer(index, context);
        writeBuffer(s, context);
        writeBuffer(v, context);
        writeBuffer(st, context);
        writeBuffer(buf, context);
        writeBuffer(t, context);
        return context;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, index);
        readBuffer(buf, offset, s);
        readBuffer(buf, offset, v);
        readBuffer(buf, offset, st);
        readBuffer(buf, offset, this->buf);
        readBuffer(buf, offset, t);
        return offset;
    }
};

struct Test
{
    Base base;
    std::string s;
    std::string serialize()
    {
        std::string context;
        writeBuffer(base, context);
        writeBuffer(s, context);

        return context;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, base);
        readBuffer(buf, offset, s);
        return offset;
    }
};

TEST_CASE("simple_serialize")
{
    Test test{
        Base{1, "123", {4, 5, 6}, {7, 8, 9}, {'a', 'b', 'c'}, UUID::gen()},
        "456"};
    auto s = test.serialize();
    Test test2;
    test2.deserialize(s.data());
    CHECK(test2.base.index == 1);
    CHECK(test2.base.s == "123");
    CHECK(test2.base.v == std::vector<int>{4, 5, 6});
    CHECK(test2.base.st == std::set<int>{7, 8, 9});
    CHECK(strcmp(test2.base.buf, "abc") == 0);
    CHECK(test2.base.t == test.base.t);
    CHECK(test2.s == "456");
}

template <typename T>
char* SerializeImpl(char* buf, const T& arg)
{
    return writeBuffer(arg, buf);
}

template <typename First, typename... Rest>
char* SerializeImpl(char* buf, const First& first, const Rest&... rest)
{
    buf = SerializeImpl(buf, first);
    return SerializeImpl(buf, rest...);
}

template <typename T>
uint16_t DeserializeImpl(const char* buf, uint16_t& offset, T& arg)
{
    readBuffer(buf, offset, arg);
    return offset;
}

template <typename First, typename... Rest>
uint16_t DeserializeImpl(const char* buf,
    uint16_t& offset,
    First& first,
    Rest&... rest)
{
    DeserializeImpl(buf, offset, first);
    DeserializeImpl(buf, offset, rest...);
    return offset;
}

template <typename... Rest>
uint16_t CalculateSizeImpl(Rest&... rest)
{
    return (::CalculateSize(rest) + ...);
}

#define GEN_SERIALIZE(...)                         \
    char* serialize(char* buf)                     \
    {                                              \
        return SerializeImpl(buf, __VA_ARGS__);    \
    }                                              \
    uint16_t deserialize(const char* buf)          \
    {                                              \
        uint16_t offset;                           \
        DeserializeImpl(buf, offset, __VA_ARGS__); \
        return offset;                             \
    }                                              \
    uint16_t CalculateSize()                       \
    {                                              \
        return CalculateSizeImpl(__VA_ARGS__);     \
    }

struct BaseNew
{
    int index;
    std::string s;
    std::vector<int> v;
    std::set<int> st;
    char c[3];
    UUID t;
    // GEN_SERIALIZE(index, s, v, st, c, t)
    char* serialize(char* buf)
    {
        buf = writeBuffer(index, buf);
        buf = writeBuffer(s, buf);
        buf = writeBuffer(v, buf);
        buf = writeBuffer(st, buf);
        buf = writeBuffer(c, buf);
        buf = writeBuffer(t, buf);
        return buf;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, index);
        readBuffer(buf, offset, s);
        readBuffer(buf, offset, v);
        readBuffer(buf, offset, st);
        readBuffer(buf, offset, c);
        readBuffer(buf, offset, t);
        return offset;
    }

    uint16_t CalculateSize()
    {
        return ::CalculateSize(index) + ::CalculateSize(s) + ::CalculateSize(v)
            + ::CalculateSize(st) + ::CalculateSize(c) + ::CalculateSize(t);
    }
};

struct TestNew
{
    BaseNew base;
    std::string s;
    // GEN_SERIALIZE(base, s)
    char* serialize(char* buf)
    {
        buf = writeBuffer(base, buf);
        buf = writeBuffer(s, buf);

        return buf;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, base);
        readBuffer(buf, offset, s);
        return offset;
    }

    uint16_t CalculateSize()
    {
        return ::CalculateSize(base) + ::CalculateSize(s);
    }
};

TEST_CASE("simple_serialize_new")
{
    TestNew test{
        BaseNew{1, "123", {4, 5, 6}, {7, 8, 9}, {'a', 'b', 'c'}, UUID::gen()},
        "456"};
    std::string s(test.CalculateSize(), '\0');
    test.serialize(s.data());
    TestNew test2;
    test2.deserialize(s.data());
    CHECK(test2.base.index == 1);
    CHECK(test2.base.s == "123");
    CHECK(test2.base.v == std::vector<int>{4, 5, 6});
    CHECK(test2.base.st == std::set<int>{7, 8, 9});
    CHECK(strcmp(test2.base.c, "abc") == 0);
    CHECK(test2.base.t == test.base.t);
    CHECK(test2.s == "456");
}

TEST_CASE("simple_serialize_bench")
{
    Test test{Base{1, "123", {4, 5, 6}, {7, 8, 9}, {'a', 'b', 'c'}}, "456"};
    TestNew test_new{
        BaseNew{1, "123", {4, 5, 6}, {7, 8, 9}, {'a', 'b', 'c'}}, "456"};
    uint32_t epochnum = 10;
    uint32_t total    = 1024 * 1024;

    ankerl::nanobench::Bench().epochs(epochnum).run("serialize_v1", [&]() {
        for (int i = 0; i < total; i++)
        {
            auto s = test.serialize();
            Test test2;
            test2.deserialize(s.data());
        }
    });

    ankerl::nanobench::Bench().epochs(epochnum).run("serialize_v2", [&]() {
        for (int i = 0; i < total; i++)
        {
            std::string s(test_new.CalculateSize(), '\0');
            test_new.serialize(s.data());
            TestNew test2;
            test2.deserialize(s.data());
        }
    });
}