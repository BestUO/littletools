#pragma once

#include <net/if.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <type_traits>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include "network_global.hpp"

namespace network
{
class SocketBase
{
public:
    static sockaddr_in CreateAddr(const char* ip, uint16_t port)
    {
        sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        inet_pton(AF_INET, ip, &addr.sin_addr);
        return addr;
    }

    static sockaddr_un CreateAddr(const char* path)
    {
        sockaddr_un addr;
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);
        return addr;
    }

    static Result setNonBlocking(int fd)
    {
        int flags = fcntl(fd, F_GETFL, 0);
        if (flags == -1)
        {
            printf("errno:%d %s\n", errno, strerror(errno));
            return Result::GETSOCKOPT_NONBLOCK_FAIL;
        }
        else
        {
            flags |= O_NONBLOCK;
            if (fcntl(fd, F_SETFL, flags) == -1)
            {
                printf("errno:%d %s\n", errno, strerror(errno));
                return Result::SETSOCKOPT_NONBLOCK_FAIL;
            }
        }
        return Result::SUCCESS;
    }

    static bool IsNonBlocking(int fd)
    {
        int flags = fcntl(fd, F_GETFL, 0);
        if (flags == -1)
        {
            printf("errno:%d %s\n", errno, strerror(errno));
            return false;
        }
        return flags & O_NONBLOCK;
    }
};

template <bool USEUNIX, bool USEUDP>
class Socket
{
public:
    using sockaddr_type
        = std::conditional_t<USEUNIX, struct sockaddr_un, struct sockaddr_in>;
    Socket()
    {
        __sockfd = socket(
            USEUNIX ? AF_UNIX : AF_INET, USEUDP ? SOCK_DGRAM : SOCK_STREAM, 0);
        if (__sockfd < 0)
            printf("errno:%d %s\n", errno, strerror(errno));
    }

    ~Socket()
    {
        close(__sockfd);
    }

    template <bool U = USEUNIX, typename std::enable_if_t<!U, int> = 0>
    Result SetAddr(const char* ip, uint16_t port)
    {
        return BindSocket(SocketBase::CreateAddr(ip, port));
    }

    template <bool U = USEUNIX, typename std::enable_if_t<U, int> = 0>
    void SetAddr(const char* path)
    {
        unlink(path);
        BindSocket(SocketBase::CreateAddr(path));
    }

    static void SetSocketBufSize(int sock, bool ifsend, int buffersize)
    {
        if (sock > 0)
        {
            if (buffersize > 0)

            {
                if (-1
                    == setsockopt(sock,
                        SOL_SOCKET,
                        ifsend ? SO_SNDBUF : SO_RCVBUF,
                        reinterpret_cast<char*>(&buffersize),
                        sizeof(buffersize)))
                {
                    printf("set %s failed: %s\n",
                        ifsend ? "SO_SNDBUF" : "SO_RCVBUF",
                        strerror(errno));
                }
            }
            GetSocketBufSize(sock, ifsend);
        }
    }

    static void GetSocketBufSize(int sock, bool ifsend)
    {
        int buffer_size;
        socklen_t buffer_size_len = sizeof(buffer_size);
        if (sock > 0)
        {
            if (-1
                == getsockopt(sock,
                    SOL_SOCKET,
                    ifsend ? SO_SNDBUF : SO_RCVBUF,
                    reinterpret_cast<char*>(&buffer_size),
                    &buffer_size_len))
            {
                printf("get %s failed: %s\n",
                    ifsend ? "SO_SNDBUF" : "SO_RCVBUF",
                    strerror(errno));
            }
            else
            {
                printf(" get %s size: %d\n",
                    ifsend ? "SO_SNDBUF" : "SO_RCVBUF",
                    buffer_size);
            }
        }
    }

    sockaddr_type GetAddr() const
    {
        return __send_addr;
    }

    Result SetReuseAddrAndPort()
    {
        int opt = 1;
        if ((setsockopt(__sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))
                < 0)
            || (setsockopt(
                    __sockfd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt))
                < 0))
        {
            printf("errno:%d %s\n", errno, strerror(errno));
            return Result::SETSOCKOPT_PORTREUSE_FAIL;
        }
        return Result::SUCCESS;
    }

    int GetSocket() const
    {
        return __sockfd;
    }

    Result setNonBlocking()
    {
        return SocketBase::setNonBlocking(__sockfd);
    }

    bool IsNonBlocking()
    {
        return SocketBase::IsNonBlocking(__sockfd);
    }

protected:
    int __sockfd;
    sockaddr_type __send_addr;

    Result BindSocket(const sockaddr_type& addr)
    {
        if (::bind(__sockfd,
                reinterpret_cast<const struct sockaddr*>(&addr),
                sizeof(addr))
            < 0)
        {
            printf("errno:%d %s\n", errno, strerror(errno));
            return Result::BIND_FAIL;
        }
        else
        {
            socklen_t addr_size = sizeof(addr);
            getsockname(__sockfd, (struct sockaddr*)&__send_addr, &addr_size);
        }
        return Result::SUCCESS;
    }
};
}  // namespace network