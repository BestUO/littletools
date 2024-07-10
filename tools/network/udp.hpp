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
#include <sys/types.h>
#include <ifaddrs.h>
#include <tuple>
#include <functional>
#include "network_params.hpp"
#include "socket.hpp"
#include <iostream>

namespace network
{
template <bool USEUNIX>
class UDP : public Socket<USEUNIX, true>
{
public:
    void addToMultiCastImpl(const std::string& multicastip)
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

    bool addToMultiCastImpl(const std::string& multicastip,
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

    void SetCallBack(std::function<std::string(const char*, size_t size)> cb)
    {
        __cb = cb;
    }

    void Recv(char* buf, size_t size)
    {
        sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);
        auto received      = recvfrom(this->__sockfd,
            buf,
            size,
            0,
            reinterpret_cast<sockaddr*>(&addr),
            &addr_len);

        if (received < 0)
            printf("errno:%d %s\n", errno, strerror(errno));
        else if (received > 0)
            HandleData(buf, received, addr);
    };

    void Send(const std::string& message, const sockaddr_in& sender_addr)
    {
        if (sendto(this->__sockfd,
                message.c_str(),
                message.size(),
                0,
                reinterpret_cast<sockaddr*>(
                    const_cast<sockaddr_in*>(&sender_addr)),
                sizeof(sender_addr))
            < 0)
            printf("errno:%d %s\n", errno, strerror(errno));
    };

private:
    std::function<std::string(const char*, size_t size)> __cb;
    void HandleData(const char* buf,
        size_t size,
        const sockaddr_in& sender_addr)
    {
        auto response = __cb(buf, size);
        if (!response.empty())
            Send(response, sender_addr);
    }
};
}  // namespace network