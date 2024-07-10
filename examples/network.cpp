#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/network/udp.hpp"
#include "tools/network/simple_poll.hpp"
#include "tools/network/network_manager.hpp"

TEST_CASE("network")
{
    network::NetWorkManager<network::SimplePoll<network::UDP<false>>>
        network_manager;
    network_manager.Start();
    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_UDP_inet_base")
{
    char buf[100] = {0};
    network::UDP<false> udp1, udp2;
    udp1.SetReuseAddrAndPort();
    udp1.SetAddr("127.0.0.1", 0);
    udp1.SetCallBack([](const char* data, size_t size) -> std::string {
        std::string str(data, size);
        CHECK(str == "from udp2");
        return "";
    });

    udp2.SetReuseAddrAndPort();
    udp2.SetAddr("127.0.0.1", 12345);
    udp2.SetCallBack([](const char* data, size_t size) -> std::string {
        std::string str(data, size);
        CHECK(str == "from udp1");
        return "from udp2";
    });

    udp1.Send("from udp1", udp2.GetAddr());

    udp2.Recv(buf, sizeof(buf));
    udp1.Recv(buf, sizeof(buf));
}

// TEST_CASE("network_UDP_inet_multicast")
// {
//     network::UDP<false> udp, udp2;
//     udp.SetReuseAddrAndPort();
//     udp.addToMultiCastImpl("127.0.0.1");
//     udp.SetAddr("127.0.0.1", 12345);
// }

// TEST_CASE("network_UDP_unix")
// {
//     network::UDP<true> udp;
//     udp.SetReuseAddrAndPort();
//     udp.addToMultiCastImpl("127.0.0.1");
//     udp.SetAddr("/tmp/test");
// }