#include <chrono>
#include <iostream>
#include "tools/reliable_udp/large_msg_recv.hpp"
#include "tools/network/network_global.hpp"

int main()
{
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();

    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 1024;
    constexpr int BAND_WIDTH               = 1024 * 1024 * 300;
    int recv_count                         = 0;
    bool flag                              = false;
    auto first_timepoint                   = std::chrono::steady_clock::now();

    RUDP::LargeMsg::RUDPLargeMsgRecv<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_recv("127.0.0.1",
            9988,
            [&recv_count, &flag, &first_timepoint](
                std::unique_ptr<char[]> recv) {
                recv_count++;
                if (recv_count % 10000 == 0)
                {
                    auto now_timepoint = std::chrono::steady_clock::now();
                    auto duration
                        = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now_timepoint - first_timepoint);
                    std::cout << "recv_count: " << recv_count
                              << " duration: " << duration.count() << "ms"
                              << std::endl;
                    recv_count      = 0;
                    first_timepoint = now_timepoint;
                }
            });
    sleep(12);
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    return 0;
}