#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/simplepoll/simplepoll.hpp"

TEST_CASE("simplepoll")
{
    SimplePoll<> simplepoll;
    simplepoll.bindAndSetCB("0.0.0.0",
        9987,
        "234.56.78.90",
        [](const char* data,
            uint16_t len,
            const std::string& ip,
            uint16_t port) {
            REQUIRE_EQ(std::string(data) == "123", true);
            return std::string();
        });
    usleep(10000);
    simplepoll.sendData("123", 3, "234.56.78.90", 9987);
    usleep(10000);
    simplepoll.stop();
}