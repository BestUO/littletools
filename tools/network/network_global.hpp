#pragma once

#include <type_traits>
#include <errno.h>
#include <stdio.h>
#include <string>

namespace network
{
class ProtocolBase
{ };

template <typename T>
concept SocketType = requires(T t)
{
    {
        t.GetSocket()
        } -> std::same_as<int>;
    {t.Recv(std::declval<char*>(), size_t())};
    {&T::Send};
    (t.GetAddr());
};
#define MAX_SOCK_SIZE 512
#define MAX_BUF_SIZE 65536
}  // namespace network