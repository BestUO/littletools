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

TEST_CASE("simplepoll_performance")
{
    SimplePoll<> simplepoll;
    int i = 0;
    simplepoll.bindAndSetCB("127.0.0.1",
        12345,
        "",
        [&](const char* data,
            uint16_t len,
            const std::string& ip,
            uint16_t port) {
            i++;
            return "";
        });
    usleep(10000);

    std::thread sendthread([&] {
        auto fd = simplepoll.bindAndSetCB("127.0.0.1", 12345, "", nullptr);
        std::string data(200, 'a');
        usleep(500000);
        int j    = 0;
        auto now = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - now)
                   .count()
            < 1)
        {
            j++;
            SimplePoll<>::sendData(
                data.data(), data.size(), "127.0.0.1", 12345, -1);
        }
        std::cout << "send num: " << j << " ";
    });
    sleep(2);
    sendthread.join();
    std::cout << "old simplepoll recv num: " << i << std::endl;
    simplepoll.stop();
}