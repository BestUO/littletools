#include <iostream>
#include <unordered_map>
#include <map>
#include <unistd.h>
#include <limits>
#include <list>
#include "tools/simple_list.hpp"
#include "doctest/doctest.h"
#include "nanobench.h"

TEST_CASE("simple_list_1")
{
    int total = 1024 * 102;
    int* v    = new int[total];
    for (uint32_t i = 0; i < total; i++)
        v[i] = i;

    SimpleList<int> simple_list;
    for (uint32_t i = 0; i < total / 2; i++)
        simple_list.AddNode(&v[i]);
    int count = 0;
    for (uint32_t i = 0; i < total / 2; i++)
        simple_list.RemoveNode(&v[i]);
    CHECK(simple_list.PopHead() == nullptr);
    count = 0;
    for (uint32_t i = total / 2; i < total; i++)
    {
        simple_list.AddNode(&v[i]);
        simple_list.RemoveNode(&v[i]);
    }
    CHECK(simple_list.PopHead() == nullptr);
    delete[] v;
}

TEST_CASE("simple_list_2")
{
    int* v = new int;

    SimpleList<int> simple_list;
    auto node_ptr = simple_list.AddNode(v);
    simple_list.RemoveNode(node_ptr);
    delete v;
}

TEST_CASE("simple_list_bench")
{
    int epochnum = 1;
    int total    = 1024 * 102 * 3;
    int* v       = new int[total];
    for (uint32_t i = 0; i < total; i++)
        v[i] = i;

    SimpleList<int> simple_list;
    ankerl::nanobench::Bench().epochs(epochnum).run("simple_list", [&]() {
        for (uint32_t i = 0; i < total / 2; i++)
            simple_list.AddNode(&v[i]);
        for (uint32_t i = 0; i < total / 2; i++)
            simple_list.RemoveNode(&v[i]);
        for (uint32_t i = total / 2; i < total; i++)
        {
            simple_list.AddNode(&v[i]);
            simple_list.RemoveNode(&v[i]);
        }
    });

    std::list<int*> l;
    ankerl::nanobench::Bench().epochs(epochnum).run("std::list", [&]() {
        for (uint32_t i = 0; i < total / 2; i++)
            l.emplace_back(&v[i]);
        for (uint32_t i = 0; i < total / 2; i++)
            l.remove(&v[i]);
        for (uint32_t i = total / 2; i < total; i++)
        {
            l.emplace_back(&v[i]);
            l.remove(&v[i]);
        }
    });
    delete[] v;
}