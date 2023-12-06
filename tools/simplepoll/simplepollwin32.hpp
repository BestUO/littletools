#pragma once

#include "service_registry/SimpleLog.h"

template <size_t N = 65536>
class SimplePollImpl : public SimpleLog
{
public:
    virtual std::string getSpecialSendNIC()                   = 0;
    virtual int setSpecialSendNIC(int sockfd, std::string ip) = 0;

    SimplePollImpl()
    {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
        {
            std::cout << "Failed to initialize Winsock." << std::endl;
        }
    };

    ~SimplePollImpl()
    {
        WSACleanup();
    };

    bool isMulticastEnabled(const PIP_ADAPTER_ADDRESSES pAdapter)
    {
        return (pAdapter->FirstMulticastAddress != nullptr);
    }

    bool getAllAdapters(PIP_ADAPTER_ADDRESSES& pAddresses)
    {
        ULONG outBufLen = 0;
        if (GetAdaptersAddresses(AF_INET, 0, nullptr, pAddresses, &outBufLen)
            == ERROR_BUFFER_OVERFLOW)
        {
            pAddresses = static_cast<PIP_ADAPTER_ADDRESSES>(malloc(outBufLen));
            if (!pAddresses)
            {
                std::cerr << "Memory allocation failed." << std::endl;
                return false;
            }

            if (GetAdaptersAddresses(
                    AF_INET, 0, nullptr, pAddresses, &outBufLen)
                != NO_ERROR)
            {
                std::cerr << "GetAdaptersAddresses failed." << std::endl;
                free(pAddresses);
                return false;
            }
        }
        else
        {
            std::cerr
                << "GetAdaptersAddresses failed to get required buffer size."
                << std::endl;
            return false;
        }

        return true;
    }

    std::set<std::string> getLocalIPsImpl()
    {
        std::set<std::string> ips;
        PIP_ADAPTER_ADDRESSES pAddresses = nullptr;

        if (getAllAdapters(pAddresses))
        {
            PIP_ADAPTER_ADDRESSES pCurrAddresses = pAddresses;

            while (pCurrAddresses)
            {
                if ((pCurrAddresses->OperStatus == IfOperStatusUp)
                    && (pCurrAddresses->FirstUnicastAddress->Address.lpSockaddr
                            ->sa_family
                        == AF_INET))
                {
                    struct sockaddr_in* sa
                        = reinterpret_cast<struct sockaddr_in*>(
                            pCurrAddresses->FirstUnicastAddress->Address
                                .lpSockaddr);

                    char ipAddress[INET_ADDRSTRLEN];
                    inet_ntop(
                        AF_INET, &(sa->sin_addr), ipAddress, INET_ADDRSTRLEN);
                    ips.insert(ipAddress);
                }

                pCurrAddresses = pCurrAddresses->Next;
            }
        }

        if (pAddresses)
        {
            free(pAddresses);
        }

        return ips;
    }

    int setSocketImpl(int sockfd)
    {
        int r   = 0;
        int yes = 1;
        r       = setsockopt(
            sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));
        return r;
    }

    int addToMultiCastImpl(int sockfd,
        const std::string& localip,
        const std::string& multicastip)
    {
        bool flag = false;
        (void)localip;

        PIP_ADAPTER_ADDRESSES pAddresses = nullptr;

        if (getAllAdapters(pAddresses))
        {
            PIP_ADAPTER_ADDRESSES pCurrAddresses = pAddresses;

            while (pCurrAddresses)
            {
                if ((pCurrAddresses->OperStatus == IfOperStatusUp)
                    && (isMulticastEnabled(pCurrAddresses))
                    && (pCurrAddresses->FirstUnicastAddress->Address.lpSockaddr
                            ->sa_family
                        == AF_INET))
                {
                    struct sockaddr_in* sa
                        = reinterpret_cast<struct sockaddr_in*>(
                            pCurrAddresses->FirstUnicastAddress->Address
                                .lpSockaddr);

                    char ipAddress[INET_ADDRSTRLEN];
                    inet_ntop(
                        AF_INET, &(sa->sin_addr), ipAddress, INET_ADDRSTRLEN);

                    struct ip_mreq mreq;
                    (void)memset(&mreq, 0, sizeof(mreq));
                    mreq.imr_multiaddr.s_addr = inet_addr(multicastip.c_str());
                    mreq.imr_interface.s_addr = sa->sin_addr.s_addr;

                    int r = setsockopt(sockfd,
                        IPPROTO_IP,
                        IP_ADD_MEMBERSHIP,
                        reinterpret_cast<char*>(&mreq),
                        sizeof(mreq));
                    std::cout
                        << "setsockopt IP_ADD_MEMBERSHIP ip: " << ipAddress
                        << " r: " << r << std::endl;

                    flag |= (r == 0);
                }

                pCurrAddresses = pCurrAddresses->Next;
            }
        }

        if (pAddresses)
        {
            free(pAddresses);
        }

        return flag ? 0 : -1;
    }

    int setSpecialSendNICImpl(int sockfd)
    {
        auto r = setSpecialSendNIC(sockfd, getSpecialSendNIC());
        if (r < 0)
        {
            PIP_ADAPTER_ADDRESSES pAddresses = nullptr;

            if (getAllAdapters(pAddresses))
            {
                PIP_ADAPTER_ADDRESSES pCurrAddresses = pAddresses;

                while (pCurrAddresses)
                {
                    if ((pCurrAddresses->OperStatus == IfOperStatusUp)
                        && (isMulticastEnabled(pCurrAddresses))
                        && (pCurrAddresses->FirstUnicastAddress->Address
                                .lpSockaddr->sa_family
                            == AF_INET))
                    {
                        struct sockaddr_in* sa
                            = reinterpret_cast<struct sockaddr_in*>(
                                pCurrAddresses->FirstUnicastAddress->Address
                                    .lpSockaddr);

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
                        else
                        {
                            std::cout << "ipAddress " << ipAddress
                                      << " not support multicast" << std::endl;
                        }
                    }

                    pCurrAddresses = pCurrAddresses->Next;
                }
            }

            if (pAddresses)
            {
                free(pAddresses);
            }
        }
        return r;
    }
};
