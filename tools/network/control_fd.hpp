#pragma once
#include "udp.hpp"
#include "network_params.hpp"

namespace network
{
template <SocketType T>
class ControlFd
{
public:
    ControlFd()
    {
        __control_socket.SetAddr("127.0.0.1", 0);
        __control_socket.SetCallBack(
            [](const char*, size_t size) -> std::string {
                return "";
            });
    }

    void Continue()
    {
        __control_socket.Send("c", __control_socket.GetAddr());
    }

protected:
    UDP<false> __control_socket;
};
}  // namespace network