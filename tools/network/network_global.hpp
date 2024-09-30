#pragma once

#include <type_traits>
#include <errno.h>
#include <stdio.h>
#include <string>

#include <net/if.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <type_traits>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <string>
#include <sys/types.h>
#include <ifaddrs.h>
#include <tuple>
#include <functional>
#include "network_global.hpp"
#include <iostream>

namespace network
{

enum Result
{
    SUCCESS,
    BIND_FAIL,
    SENDTO_FAIL,
    RECVFROM_FAIL,
    GETSOCKOPT_NONBLOCK_FAIL,
    SETSOCKOPT_NONBLOCK_FAIL,
    SETSOCKOPT_PORTREUSE_FAIL,
    SETSOCKOPT_ADDMEMBERSHIP_FAIL,
    SETSOCKOPT_MULTICASTIF_FAIL,
    SETSOCKOPT_RCVBUF_FAIL,
    SETSOCKOPT_SNDBUF_FAIL,
};

class ProtocolBase
{
public:
    virtual void SetCallBack(std::function<
        std::string(const char*, size_t size, const sockaddr& addr)> cb)
        = 0;
    virtual void Recv()           = 0;
    virtual int GetSocket() const = 0;
};

template <typename T>
concept SocketType = requires(T t)
{
    {t.Recv(std::declval<char*>(), size_t())};
    {&T::Send, t};
    {&T::SetCallBack, t};
};
#define MAX_SOCK_SIZE 255
#define MAX_BUF_SIZE 65535
}  // namespace network