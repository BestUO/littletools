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

    static void setNonBlocking(int fd)
    {
        int flags = fcntl(fd, F_GETFL, 0);
        if (flags == -1)
        {
            printf("errno:%d %s\n", errno, strerror(errno));
        }
        else
        {
            flags |= O_NONBLOCK;
            if (fcntl(fd, F_SETFL, flags) == -1)
            {
                printf("errno:%d %s\n", errno, strerror(errno));
            }
        }
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
        int domain = USEUNIX ? AF_UNIX : AF_INET;
        int type   = USEUDP ? SOCK_DGRAM : SOCK_STREAM;

        __sockfd = socket(domain, type, 0);
        if (__sockfd < 0)
            printf("errno:%d %s\n", errno, strerror(errno));
    }

    ~Socket()
    {
        close(__sockfd);
    }

    template <bool U = USEUNIX, typename std::enable_if_t<!U, int> = 0>
    void SetAddr(const char* ip, uint16_t port)
    {
        BindSocket(SocketBase::CreateAddr(ip, port));
    }

    template <bool U = USEUNIX, typename std::enable_if_t<U, int> = 0>
    void SetAddr(const char* path)
    {
        unlink(path);
        BindSocket(SocketBase::CreateAddr(path));
    }

    sockaddr_type GetAddr() const
    {
        return __send_addr;
    }

    void SetReuseAddrAndPort()
    {
        int opt = 1;
        if (setsockopt(__sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))
            < 0)
            printf("errno:%d %s\n", errno, strerror(errno));
        if (setsockopt(__sockfd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt))
            < 0)
            printf("errno:%d %s\n", errno, strerror(errno));
    }

    int GetSocket() const
    {
        return __sockfd;
    }

    void setNonBlocking()
    {
        SocketBase::setNonBlocking(__sockfd);
    }

    bool IsNonBlocking()
    {
        return SocketBase::IsNonBlocking(__sockfd);
    }

protected:
    int __sockfd;
    sockaddr_type __send_addr;

    void BindSocket(const sockaddr_type& addr)
    {
        if (bind(__sockfd,
                reinterpret_cast<const struct sockaddr*>(&addr),
                sizeof(addr))
            < 0)
            printf("errno:%d %s\n", errno, strerror(errno));
        else
        {
            socklen_t addr_size = sizeof(addr);
            getsockname(__sockfd, (struct sockaddr*)&__send_addr, &addr_size);
        }
    }
};
}  // namespace network