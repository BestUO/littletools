#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/lrucache.hpp"

TEST_CASE("LRUCache")
{
    LRUCache<int, std::string> cache(1, 20);
    cache.StoreKeyValue(1, "1");
    CHECK(cache.GetValue(1).value() == "1");
    cache.StoreKeyValue(1, "2");
    cache.StoreKeyValue(3, "3");
    CHECK(cache.GetValue(1).value() == "2");
    CHECK(cache.GetValue(2) == std::nullopt);
}