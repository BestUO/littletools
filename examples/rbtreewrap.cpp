#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/rbtree/RBTreeWrap.hpp"

TEST_CASE("RBTreeWrapV1")
{
    struct TestStruct
    {
        std::string key = "83";
        int age         = 10;
        int operator<(const TestStruct& t) const
        {
            return age < t.age;
        }
        bool operator==(const TestStruct& t) const
        {
            return key == t.key;
        }
    };
    rbtreewrap::v1::RBTreeWrap<TestStruct> tree;
    TestStruct val1 = {"67", 1};
    TestStruct val2 = {"1338", 5};
    TestStruct val3 = {"1337", 5};
    TestStruct val4 = {"83", 10};
    TestStruct val5 = {"84", 11};

    tree.AddObj(val5);
    tree.AddObj(val1);
    tree.AddObj(val2);
    tree.AddObj(val3);
    tree.AddObj(val4);

    auto t = tree.GetTopObjPtr();
    REQUIRE_EQ(t->age, val1.age);

    tree.PopTopObj();
    t = tree.GetTopObjPtr();
    REQUIRE_EQ(t->age, val2.age);

    tree.DeleteObj(*t);
    t = tree.GetTopObjPtr();
    REQUIRE_EQ(t->age, val3.age);

    t = tree.SearchObj(val4);
    REQUIRE_EQ(t->age, val4.age);
    t = tree.SearchObj([key = "84"](const TestStruct& b) {
        return key == b.key;
    });
    REQUIRE_EQ(t->age, val5.age);
}