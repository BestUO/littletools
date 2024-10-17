#include <chrono>
#include <iostream>
#include "tools/reliable_udp/large_msg_send.hpp"
#include "tools/network/network_global.hpp"

int main()
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();

    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 1024;
    constexpr int BAND_WIDTH               = 1024 * 1024 * 300;
    bool flag                              = false;
    auto first_timepoint                   = std::chrono::steady_clock::now();

    auto flow_control = std::make_shared<FlowControl>(BAND_WIDTH);
    RUDPLargeMsgSend<MAX_SEGMENT_COUNT, MAX_SEGMENT_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);

    std::string message(1024 * 100, '\100');
    auto now = std::chrono::high_resolution_clock::now();
    auto dst = network::SocketBase::CreateAddr("127.0.0.1", 9988);
    while (now + std::chrono::seconds(10)
        > std::chrono::high_resolution_clock::now())
    {
        rudp_large_msg_send.Send(std::move(message), dst);
    }

    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
    return 0;
}