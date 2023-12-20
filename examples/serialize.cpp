#include <iostream>
#include <unistd.h>
#include <map>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/serialize/serializer.hpp"
#include "tools/serialize/serializer_core.hpp"

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

TEST_CASE("serializer")
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
TEST_CASE("serializer2")
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
TEST_CASE("serializer3")
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