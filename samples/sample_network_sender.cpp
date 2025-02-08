#include <chrono>
#include <iostream>
#include <thread>
#include "tools/network/network_manager.hpp"
#include "tools/network/udp.hpp"
#include "tools/network/simple_poll.hpp"
#include "tools/network/simple_epoll.hpp"
#include "tools/cpu_bind.hpp"

int main()
{
    network::inet_udp::UDP sendudp;
    sendudp.SetAddr("172.17.154.18", 12345);
    auto endpoint = network::SocketBase::CreateAddr("172.17.154.18", 23456);
    std::string data(200, 'a');
    int j      = 0;
    auto begin = std::chrono::high_resolution_clock::now();
    while (j < 20000000)
    {
        j++;
        sendudp.Send(data, endpoint);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "send num: " << j << " use time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     end - begin)
                     .count()
              << "ms" << std::endl;
    return 0;
}