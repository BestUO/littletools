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
#include "simple_allocate.hpp"

namespace network
{

template <bool USEUNIX, typename T>
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
    void SetCallBack(
        std::function<std::string(const char*, size_t, const sockaddr& addr)>
            cb)
    {
        std::lock_guard<std::mutex> lock(__mutex4cb);
        __cb = cb;
        // if (__cb == nullptr)
        //     close(this->__sockfd);
    }

    int GetSocket() const
    {
        return Socket<USEUNIX, true>::GetSocket();
    };

    void Recv()
    {
        bool nonblock = SocketBase::IsNonBlocking(this->__sockfd);
        if (nonblock)
        {
            while (RecvOnce(
                this->__sockfd, __allocate.GetBuf(), __allocate.GetBufSize()))
            { }
        }
        else
        {
            RecvOnce(
                this->__sockfd, __allocate.GetBuf(), __allocate.GetBufSize());
        }
    };

    void FreeBuf(char* buf)
    {
        __allocate.FreeBuf(buf);
    }

    Result Send(std::string_view message, const sockaddr_type& sender_addr)
    {
        if (sendto(this->__sockfd,
                message.data(),
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
            return Result::SENDTO_FAIL;
        }
        return Result::SUCCESS;
    };

private:
    std::function<std::string(const char*, size_t size, const sockaddr& addr)>
        __cb;
    std::mutex __mutex4cb;
    std::string __response;
    T __allocate;

    void HandleData(const char* buf,
        size_t size,
        const sockaddr_type& sender_addr)
    {
        {
            std::lock_guard<std::mutex> lock(__mutex4cb);
            if (__cb)
                __response = std::move(
                    this->__cb(buf, size, (const sockaddr&)sender_addr));
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
template <typename T = SimpleAllocate<>>
class UDP : public UDPBase<false, T>
{
public:
    Result SetMultiCastSendIf()
    {
        Result r                   = Result::SETSOCKOPT_MULTICASTIF_FAIL;
        struct ifaddrs* ifAddrList = nullptr;
        struct ifaddrs* ifa        = nullptr;
        getifaddrs(&ifAddrList);
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
                    if ((r = SetMultiCastSendIf(ipAddress)) == Result::SUCCESS)
                        break;
                }
            }
        }
        freeifaddrs(ifAddrList);
        return r;
    }

    Result SetMultiCastSendIf(const char* ip)
    {
        if (strlen(ip) == 0)
            return SetMultiCastSendIf();
        struct in_addr localInterface;
        localInterface.s_addr = inet_addr(ip);
        if (setsockopt(this->__sockfd,
                IPPROTO_IP,
                IP_MULTICAST_IF,
                reinterpret_cast<char*>(&localInterface),
                sizeof(localInterface))
            < 0)
        {
            printf("errno:%d %s\n", errno, strerror(errno));
            return Result::SETSOCKOPT_MULTICASTIF_FAIL;
        }
        return Result::SUCCESS;
    }

    Result AddMultiCast(const char* multicastip)
    {
        Result r                   = Result::SETSOCKOPT_ADDMEMBERSHIP_FAIL;
        struct ifaddrs* ifAddrList = nullptr;
        struct ifaddrs* ifa        = nullptr;
        getifaddrs(&ifAddrList);
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
                    if ((r = AddMultiCast(multicastip, ipAddress))
                        == Result::SUCCESS)
                        break;
                }
            }
        }
        freeifaddrs(ifAddrList);
        return r;
    }

    Result AddMultiCast(const char* multicastip, const char* interfaceip)
    {
        if (strlen(interfaceip) == 0)
            return AddMultiCast(multicastip);
        struct ip_mreq mreq;
        (void)memset(&mreq, 0, sizeof(mreq));
        mreq.imr_multiaddr.s_addr = inet_addr(multicastip);
        mreq.imr_interface.s_addr = inet_addr(interfaceip);
        if (setsockopt(this->__sockfd,
                IPPROTO_IP,
                IP_ADD_MEMBERSHIP,
                &mreq,
                sizeof(mreq))
            < 0)
        {
            printf("errno:%d %s\n", errno, strerror(errno));
            return Result::SETSOCKOPT_ADDMEMBERSHIP_FAIL;
        }
        return Result::SUCCESS;
    }

    Result SetSocketBufSize(bool ifsend, int buffersize)
    {
        auto sock = Socket<false, true>::GetSocket();
        Socket<false, true>::GetSocketBufSize(sock, ifsend);
        Socket<false, true>::SetSocketBufSize(sock, ifsend, buffersize);
        return Result::SUCCESS;
    }
};

}  // namespace inet_udp

namespace unix_udp
{
template <typename T = SimpleAllocate<>>
class UDP : public UDPBase<true, T>
{ };
}  // namespace unix_udp
}  // namespace network