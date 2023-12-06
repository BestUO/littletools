#pragma once

#include "../SimpleLog.hpp"

template <size_t N = 65536>
class SimplePollImpl : public SimpleLog
{
public:
    virtual std::string getSpecialSendNIC()                   = 0;
    virtual int setSpecialSendNIC(int sockfd, std::string ip) = 0;

    std::set<std::string> getLocalIPsImpl()
    {
        std::set<std::string> ips;
        struct ifaddrs* ifAddrList = nullptr;
        struct ifaddrs* ifa        = nullptr;
        auto r                     = getifaddrs(&ifAddrList);
        for (ifa = ifAddrList; r != -1 && ifa != nullptr; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == nullptr)
            {
                continue;
            }

            if (ifa->ifa_addr->sa_family == AF_INET)
            {
                if (ifa->ifa_flags & IFF_UP)
                {
                    struct sockaddr_in* sa
                        = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
                    char ipAddress[INET_ADDRSTRLEN];
                    inet_ntop(
                        AF_INET, &(sa->sin_addr), ipAddress, INET_ADDRSTRLEN);
                    ips.insert(ipAddress);
                }
            }
        }
        freeifaddrs(ifAddrList);
        return ips;
    }

    int setSocketImpl(int sockfd)
    {
        int r   = 0;
        int yes = 1;
        r = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        logOut("setsockopt SO_REUSEADDR r:", r);
        if (r == 0)
        {
            r = setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
            logOut("setsockopt SO_REUSEPORT r:", r);
        }
        return r;
    }

    int addToMultiCastImpl(int sockfd,
        const std::string& localip,
        const std::string& multicastip)
    {
        bool flag = false;
        (void)localip;
        struct ifaddrs* ifAddrList = nullptr;
        struct ifaddrs* ifa        = nullptr;
        int r                      = getifaddrs(&ifAddrList);
        for (ifa = ifAddrList; r != -1 && ifa != nullptr; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == nullptr)
            {
                logOut(ifa->ifa_name, " not support ifa->ifa_addr == nullptr");
                continue;
            }
            else
            {
                logOut(ifa->ifa_name, " sa_family:", ifa->ifa_addr->sa_family);
            }

            if (ifa->ifa_addr->sa_family == AF_INET)
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
                    r                         = setsockopt(sockfd,
                        IPPROTO_IP,
                        IP_ADD_MEMBERSHIP,
                        &mreq,
                        sizeof(mreq));
                    logOut("setsockopt IP_ADD_MEMBERSHIP ip:",
                        ipAddress,
                        " r:",
                        r);
                    flag |= (r == 0);
                }
                else
                {
                    logOut(ifa->ifa_name,
                        " not support",
                        " IFF_MULTICAST:",
                        (ifa->ifa_flags & IFF_MULTICAST),
                        " IFF_UP:",
                        (ifa->ifa_flags & IFF_UP));
                }
            }
        }
        freeifaddrs(ifAddrList);
        return flag ? 0 : -1;
    }

    int setSpecialSendNICImpl(int sockfd)
    {
        auto r = setSpecialSendNIC(sockfd, getSpecialSendNIC());
        if (r < 0)
        {
            struct ifaddrs* ifAddrList = nullptr;
            struct ifaddrs* ifa        = nullptr;
            r                          = getifaddrs(&ifAddrList);
            for (ifa = ifAddrList; r != -1 && ifa != nullptr;
                 ifa = ifa->ifa_next)
            {
                if (ifa->ifa_addr == nullptr)
                {
                    logOut(
                        ifa->ifa_name, " not support ifa->ifa_addr == nullptr");
                    continue;
                }
                else
                {
                    logOut(
                        ifa->ifa_name, " sa_family:", ifa->ifa_addr->sa_family);
                }

                if (ifa->ifa_addr->sa_family == AF_INET)
                {
                    if (ifa->ifa_flags & IFF_MULTICAST
                        && ifa->ifa_flags & IFF_UP)
                    {
                        struct sockaddr_in* sa
                            = reinterpret_cast<struct sockaddr_in*>(
                                ifa->ifa_addr);
                        char ipAddress[INET_ADDRSTRLEN];
                        inet_ntop(AF_INET,
                            &(sa->sin_addr),
                            ipAddress,
                            INET_ADDRSTRLEN);
                        r = setSpecialSendNIC(sockfd, ipAddress);
                        if (r >= 0)
                        {
                            break;
                        }
                    }
                    else
                    {
                        logOut(ifa->ifa_name,
                            " not support",
                            " IFF_MULTICAST:",
                            (ifa->ifa_flags & IFF_MULTICAST),
                            " IFF_UP:",
                            (ifa->ifa_flags & IFF_UP));
                    }
                }
            }
            freeifaddrs(ifAddrList);
        }
        return r;
    }
};