#pragma once

#include <net/if.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <mutex>
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
#include "socket.hpp"
#include <iostream>

namespace network
{

template <bool USEUNIX>
class UDPBase
    : public virtual ProtocolBase
    , public Socket<USEUNIX, true>
{
public:
    UDPBase()                          = default;
    UDPBase(const UDPBase&)            = delete;
    UDPBase& operator=(const UDPBase&) = delete;
    UDPBase(UDPBase&&)                 = delete;
    UDPBase& operator=(UDPBase&&)      = delete;
    ~UDPBase()
    {
        SetCallBack(nullptr);
    }

    using sockaddr_type
        = std::conditional_t<USEUNIX, struct sockaddr_un, struct sockaddr_in>;
    void SetCallBack(std::function<std::string(const char*, size_t size)> cb)
    {
        std::lock_guard<std::mutex> lock(__mutex4cb);
        __cb = cb;
    }

    int GetSocket() const
    {
        return Socket<USEUNIX, true>::GetSocket();
    };
    void Recv(char* buf, size_t size)
    {
        bool nonblock = SocketBase::IsNonBlocking(this->__sockfd);
        if (nonblock)
        {
            while (RecvOnce(this->__sockfd, buf, size))
            { }
        }
        else
        {
            RecvOnce(this->__sockfd, buf, size);
        }
    };

    void Send(const std::string& message, const sockaddr_type& sender_addr)
    {
        if (sendto(this->__sockfd,
                message.c_str(),
                message.size(),
                0,
                reinterpret_cast<sockaddr*>(
                    const_cast<sockaddr_type*>(&sender_addr)),
                sizeof(sender_addr))
            < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                ;
            else
                printf("errno:%d %s\n", errno, strerror(errno));
        }
    };

private:
    std::function<std::string(const char*, size_t size)> __cb;
    std::mutex __mutex4cb;
    std::string __response;

    void HandleData(const char* buf,
        size_t size,
        const sockaddr_type& sender_addr)
    {
        {
            std::lock_guard<std::mutex> lock(__mutex4cb);
            if (__cb)
                __response = std::move(this->__cb(buf, size));
            else
                __response.clear();
        }
        if (!__response.empty())
            Send(__response, sender_addr);
    }

    bool RecvOnce(int fd, char* buf, size_t size)
    {
        sockaddr_type addr;
        socklen_t addr_len = sizeof(addr);
        auto received      = recvfrom(
            fd, buf, size, 0, reinterpret_cast<sockaddr*>(&addr), &addr_len);

        if (received < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            { }
            else
                printf("errno:%d %s\n", errno, strerror(errno));
            return false;
        }
        else if (received == 0)
            return false;
        else
            HandleData(buf, received, addr);
        return true;
    }
};

namespace inet_udp
{
class UDP : public UDPBase<false>
{
public:
    void SetMultiCastSendNIC(std::string ip)
    {
        struct in_addr localInterface;
        localInterface.s_addr = inet_addr(ip.c_str());
        if (setsockopt(this->__sockfd,
                IPPROTO_IP,
                IP_MULTICAST_IF,
                reinterpret_cast<char*>(&localInterface),
                sizeof(localInterface))
            < 0)
            printf("errno:%d %s\n", errno, strerror(errno));
    }

    void ListenMultiCast(const std::string& multicastip)
    {
        bool flag                  = false;
        struct ifaddrs* ifAddrList = nullptr;
        struct ifaddrs* ifa        = nullptr;
        int r                      = getifaddrs(&ifAddrList);
        for (ifa = ifAddrList; ifa != nullptr; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET)
            {
                if (ifa->ifa_flags & IFF_MULTICAST && ifa->ifa_flags & IFF_UP)
                {
                    struct sockaddr_in* sa
                        = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
                    char ipAddress[INET_ADDRSTRLEN];
                    inet_ntop(
                        AF_INET, &(sa->sin_addr), ipAddress, INET_ADDRSTRLEN);
                    struct ip_mreq mreq;
                    (void)memset(&mreq, 0, sizeof(mreq));
                    mreq.imr_multiaddr.s_addr = inet_addr(multicastip.c_str());
                    mreq.imr_interface.s_addr = inet_addr(ipAddress);
                    if (setsockopt(this->__sockfd,
                            IPPROTO_IP,
                            IP_ADD_MEMBERSHIP,
                            &mreq,
                            sizeof(mreq))
                        == 0)
                        break;
                }
            }
        }
        freeifaddrs(ifAddrList);
    }

    bool ListenMultiCast(const std::string& multicastip,
        const char* interfaceip)
    {
        struct ip_mreq mreq;
        (void)memset(&mreq, 0, sizeof(mreq));
        mreq.imr_multiaddr.s_addr = inet_addr(multicastip.c_str());
        mreq.imr_interface.s_addr = inet_addr(interfaceip);
        if (setsockopt(this->__sockfd,
                IPPROTO_IP,
                IP_ADD_MEMBERSHIP,
                &mreq,
                sizeof(mreq))
            < 0)
        {
            printf("errno:%d %s\n", errno, strerror(errno));
            return false;
        }
        return true;
    }
};

}  // namespace inet_udp

namespace unix_udp
{
class UDP : public UDPBase<true>
{ };
}  // namespace unix_udp
}  // namespace network