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
    // netstat -su: receive errors, need modify SO_RCVBUF
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();

    auto recvudp = std::make_shared<network::inet_udp::UDP<>>();
    recvudp->SetAddr("172.17.154.18", 23456);
    recvudp->SetSocketBufSize(false, 2048000);
    std::atomic<int> qps = 0;
    char pad[64]         = {0};
    int total            = 0;

    recvudp->SetCallBack([&]<typename... Args>(Args... args) -> std::string {
        qps++;
        total++;
        return "";
    });
    network_manager.AddListenSocket(recvudp);

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "qps: " << qps << " total: " << total << std::endl;
        qps = 0;
    };
    network_manager.Stop();

    return 0;
}