#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/rbtree/rbtreewrap.hpp"

TEST_CASE("RBTreeWrap v1")
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
    CHECK_EQ(t->age, val1.age);

    tree.PopTopObj();
    t = tree.GetTopObjPtr();
    CHECK_EQ(t->age, val2.age);

    tree.DeleteObj(*t);
    t = tree.GetTopObjPtr();
    CHECK_EQ(t->age, val3.age);

    t = tree.SearchObj(val4);
    CHECK_EQ(t->age, val4.age);
    t = tree.SearchObj([key = "84"](const TestStruct& b) {
        return key == b.key;
    });
    CHECK_EQ(t->age, val5.age);
}

TEST_CASE("RBTreeWrap v2 int")
{
    rbtreewrap::v2::RBTreeWrap<int> rbtreenew;

    rbtreenew.AddObj(3);
    auto ptr = rbtreenew.AddObj(2);
    rbtreenew.AddObj(1);
    rbtreenew.AddObj(5);
    rbtreenew.AddObj(4);
    bool flag;
    int t;

    std::tie(flag, t) = rbtreenew.GetTopObjByFunction(
        std::function<int(const int&)>([](const int&) {
            return true;
        }));
    CHECK_EQ(t, 1);
    std::tie(flag, t) = rbtreenew.SearchObj(5);
    CHECK_EQ(t, 5);
    std::tie(flag, t) = rbtreenew.SearchObj([searchnum = 2](const int& key) {
        return searchnum == key;
    });
    CHECK_EQ(t, 2);

    // delete 1
    std::tie(flag, t) = rbtreenew.GetTopObjByFunctionAndDelete(
        [searchnum = 1](const int& key) {
            return searchnum == key;
        });
    CHECK_EQ(t, 1);
    std::tie(flag, t) = rbtreenew.SearchObj(1);
    CHECK_EQ(flag, false);

    // delete 2
    rbtreenew.DeleteObj(ptr);
    std::tie(flag, t) = rbtreenew.SearchObj(1);
    CHECK_EQ(flag, false);

    // top is 3
    std::tie(flag, t) = rbtreenew.GetTopObjByFunction([](const int&) {
        return true;
    });
    CHECK_EQ(t, 3);
}

TEST_CASE("RBTreeWrap v2 custom Struct")
{
    struct TestStruct
    {
        std::string key = "0";
        int age         = 0;
        int operator<(const TestStruct& t) const
        {
            return age < t.age;
        }
        bool operator==(const TestStruct& t) const
        {
            return key == t.key;
        }
    };
    rbtreewrap::v2::RBTreeWrap<TestStruct> rbtreenew;
    TestStruct val1 = {"67", 1};
    TestStruct val2 = {"1338", 5};
    TestStruct val3 = {"1337", 5};
    TestStruct val4 = {"83", 10};
    TestStruct val5 = {"84", 11};

    rbtreenew.AddObj(std::move(val5));
    rbtreenew.AddObj(std::move(val1));
    rbtreenew.AddObj(std::move(val2));
    rbtreenew.AddObj(std::move(val3));
    auto ptr = rbtreenew.AddObj(std::move(val4));

    bool flag;
    TestStruct t;

    std::tie(flag, t) = rbtreenew.GetTopObjByFunction(
        std::function<int(const TestStruct&)>([](const TestStruct&) {
            return true;
        }));
    CHECK_EQ(t, val1);
    std::tie(flag, t) = rbtreenew.SearchObj(val2);
    CHECK_EQ(t, val2);
    std::tie(flag, t)
        = rbtreenew.SearchObj([searchage = 10](const TestStruct& key) {
              return searchage == key.age;
          });
    CHECK_EQ(t, val4);

    // delete age 1
    std::tie(flag, t) = rbtreenew.GetTopObjByFunctionAndDelete(
        [searchage = 1](const TestStruct& key) {
            return searchage == key.age;
        });
    CHECK_EQ(t, val1);
    std::tie(flag, t) = rbtreenew.SearchObj(val1);
    CHECK_EQ(flag, false);

    // // delete age 10
    rbtreenew.DeleteObj(ptr);
    std::tie(flag, t) = rbtreenew.SearchObj(val4);
    CHECK_EQ(flag, false);

    // top is age 5
    std::tie(flag, t) = rbtreenew.GetTopObjByFunction([](const TestStruct&) {
        return true;
    });
    CHECK_EQ(t, val2);
}