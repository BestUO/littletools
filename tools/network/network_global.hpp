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
class ProtocolBase
{
public:
    virtual void SetCallBack(
        std::function<std::string(const char*, size_t size)> cb)
        = 0;
    virtual void Recv(char* buf, size_t size) = 0;
    virtual int GetSocket() const             = 0;
};

template <typename T>
concept SocketType = requires(T t)
{
    {t.Recv(std::declval<char*>(), size_t())};
    {&T::Send, t};
    {&T::SetCallBack, t};
};
#define MAX_SOCK_SIZE 512
#define MAX_BUF_SIZE 65536
}  // namespace network