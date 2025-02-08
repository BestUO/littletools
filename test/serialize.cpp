#include <iostream>
#include <vector>
#include <unistd.h>
#include <map>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/simple_serialize.hpp"
#include "tools/serialize/serializer.hpp"
#include "tools/serialize/serializer_core.hpp"

struct Base
{
    int index;
    std::string s;
    std::vector<int> v;
    std::set<int> st;
    char buf[3];
    UUID t;
    std::string serialize() const
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
    std::string serialize() const
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

TEST_CASE("serializer_simple_old")
{
    Test test{
        Base{1, "123", {4, 5, 6}, {7, 8, 9}, {'a', 'b', '\0'}, UUID::gen()},
        "456"};
    auto s = test.serialize();
    Test test2;
    test2.deserialize(s.data());
    CHECK(test2.base.index == 1);
    CHECK(test2.base.s == "123");
    CHECK(test2.base.v == std::vector<int>{4, 5, 6});
    CHECK(test2.base.st == std::set<int>{7, 8, 9});
    CHECK(strcmp(test2.base.buf, "ab") == 0);
    CHECK(test2.base.t == test.base.t);
    CHECK(test2.s == "456");
}

struct BaseNew
{
    int index;
    std::string s;
    std::vector<int> v;
    std::set<int> st;
    char c[3];
    UUID t;
    GEN_SERIALIZE(index, s, v, st, c, t)
    //////////////////////or/////////////////////////
    // char* serialize(char* buf) const
    // {
    //     buf = writeBuffer(index, buf);
    //     buf = writeBuffer(s, buf);
    //     buf = writeBuffer(v, buf);
    //     buf = writeBuffer(st, buf);
    //     buf = writeBuffer(c, buf);
    //     buf = writeBuffer(t, buf);
    //     return buf;
    // }

    // uint16_t deserialize(const char* buf)
    // {
    //     uint16_t offset = 0;
    //     readBuffer(buf, offset, index);
    //     readBuffer(buf, offset, s);
    //     readBuffer(buf, offset, v);
    //     readBuffer(buf, offset, st);
    //     readBuffer(buf, offset, c);
    //     readBuffer(buf, offset, t);
    //     return offset;
    // }

    // uint16_t CalculateSize()
    // {
    //     return ::CalculateSize(index) + ::CalculateSize(s) +
    //     ::CalculateSize(v)
    //         + ::CalculateSize(st) + ::CalculateSize(c) + ::CalculateSize(t);
    // }
    /////////////////////////////////////////////////
};

struct TestNew
{
    BaseNew base;
    std::string s;
    GEN_SERIALIZE(base, s)
    //////////////////////or/////////////////////////
    // char* serialize(char* buf) const
    // {
    //     buf = writeBuffer(base, buf);
    //     buf = writeBuffer(s, buf);

    //     return buf;
    // }

    // uint16_t deserialize(const char* buf)
    // {
    //     uint16_t offset = 0;
    //     readBuffer(buf, offset, base);
    //     readBuffer(buf, offset, s);
    //     return offset;
    // }

    // uint16_t CalculateSize()
    // {
    //     return ::CalculateSize(base) + ::CalculateSize(s);
    // }
    /////////////////////////////////////////////////
};

TEST_CASE("serializer_simple_new")
{
    TestNew test{
        BaseNew{1, "123", {4, 5, 6}, {7, 8, 9}, {'a', 'b', '\0'}, UUID::gen()},
        "456"};
    std::string s(test.CalculateSize(), '\0');
    test.serialize(s.data());
    TestNew test2;
    test2.deserialize(s.data());
    CHECK(test2.base.index == 1);
    CHECK(test2.base.s == "123");
    CHECK(test2.base.v == std::vector<int>{4, 5, 6});
    CHECK(test2.base.st == std::set<int>{7, 8, 9});
    CHECK(strcmp(test2.base.c, "ab") == 0);
    CHECK(test2.base.t == test.base.t);
    CHECK(test2.s == "456");
}

struct A
{
    int a                  = 1;
    float b                = 1.1;
    std::map<int, float> c = {{1, 1.1}, {2, 2.2}};
    std::string d          = "12@3";
    double e               = 1.234251;

    bool operator==(const A& a) const
    {
        for (auto&& tmp : a.c)
        {
            if (tmp.second != this->c.at(tmp.first))
            {
                return false;
            }
        }
        return this->a == a.a && this->b == a.b && this->d == a.d
            && this->e == a.e && this->c.size() == a.c.size();
    }
};

TEST_CASE("serializer_ali_")
{
    A source{2, 2.2, {{3, 3.3}, {4, 4.4}}, "####", 21.4353};
    std::string s = serialize::SimpleSerializer::Serialize(source);

    auto destination
        = serialize::SimpleSerializer::Deserialize<A>(std::move(s));
    CHECK(destination == source);
}

struct B
{
    A a;
    bool operator==(const B& b) const
    {
        return this->a == b.a;
    }
};
TEST_CASE("serializer_ali_2")
{
    B source({2, 2.2, {{3, 3.3}, {4, 4.4}}, "####", 21.4353});
    std::string s = serialize::SimpleSerializer::Serialize(source);

    auto destination
        = serialize::SimpleSerializer::Deserialize<B>(std::move(s));
    CHECK(destination == source);
}

struct C
{
    int a                  = 1;
    float b                = 1.1;
    std::map<int, float> c = {{1, 1.1}, {2, 2.2}};
    std::string d          = "12@3";
    double e               = 1.234251;
    A aa;
    bool operator==(const C& b) const
    {
        for (auto&& tmp : b.c)
        {
            if (tmp.second != this->c.at(tmp.first))
            {
                return false;
            }
        }
        return this->a == b.a && this->b == b.b && this->d == b.d
            && this->aa == b.aa;
    }
};
TEST_CASE("serializer_ali_3")
{
    C source(2,
        2.2,
        {{3, 3.3}, {4, 4.4}},
        "!!!",
        4.4521,
        {2, 2.2, {{3, 3.3}, {4, 4.4}}, "####", 21.4353});
    std::string s = serialize::SimpleSerializer::Serialize(source);

    auto destination
        = serialize::SimpleSerializer::Deserialize<C>(std::move(s));
    CHECK(destination == source);
}

TEST_CASE("serializer_bench")
{
    Test test{Base{1, "123", {4, 5, 6}, {7, 8, 9}, {'a', 'b', 'c'}}, "456"};
    TestNew test_new{
        BaseNew{1, "123", {4, 5, 6}, {7, 8, 9}, {'a', 'b', 'c'}}, "456"};
    uint32_t epochnum = 10;
    uint32_t total    = 1024 * 1024;

    ankerl::nanobench::Bench().epochs(epochnum).run(
        "serialize_simple_v1", [&]() {
            for (int i = 0; i < total; i++)
            {
                auto s = test.serialize();
                Test test2;
                test2.deserialize(s.data());
            }
        });

    ankerl::nanobench::Bench().epochs(epochnum).run(
        "serialize_simple_v2", [&]() {
            for (int i = 0; i < total; i++)
            {
                std::string s(test_new.CalculateSize(), '\0');
                test_new.serialize(s.data());
                TestNew test2;
                test2.deserialize(s.data());
            }
        });
    // ankerl::nanobench::Bench().epochs(epochnum).run("serialize_ali", [&]() {
    //     for (int i = 0; i < total; i++)
    //     {
    //         std::string s = serialize::SimpleSerializer::Serialize(test_new);
    //         auto destination
    //             = serialize::SimpleSerializer::Deserialize<TestNew>(
    //                 std::move(s));
    //     }
    // });
}