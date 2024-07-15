#pragma once
#include <memory>
#include "udp.hpp"
#include "network_global.hpp"

namespace network
{
class ControlFd
{
public:
    ControlFd()
        : __control_socket(std::make_shared<network::inet_udp::UDP>())
    {
        __control_socket->SetAddr("127.0.0.1", 0);
        __control_socket->SetCallBack(
            []<typename... Args>(Args... args) -> std::string {
                return "";
            });
    }

    void Continue()
    {
        __control_socket->Send("c", __control_socket->GetAddr());
    }

protected:
    std::shared_ptr<network::inet_udp::UDP> __control_socket;
};
}  // namespace network