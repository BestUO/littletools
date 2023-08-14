#pragma once

#include <netinet/in.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <thread>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <tuple>
#include <vector>
#include <map>
#include <ifaddrs.h>
#include <iostream>
#include <atomic>
#include <sys/types.h>
#include <net/if.h>
#include <thread>

template <size_t N = 65536>
class SimplePoll
{
public:
    static SimplePoll<N>* getInstance()
    {
        static SimplePoll<N> instance;
        return &instance;
    }

    void wait()
    {
        if (__pollthread.joinable())
            __pollthread.join();
    }

    void stop()
    {
        __quit = true;
        wait();
        closeAndRemoveAll();
    }

    void start()
    {
        if (!__quit)
        {
            return;
        }
        else
        {
            __quit       = false;
            __pollthread = std::thread([this]() {
                Run();
            });
        }
    }

    int bindAndSetCB(const std::string& localip,
        uint16_t port,
        const std::string& multicast_ip,
        std::function<
            std::string(const char*, uint16_t, const std::string&, uint16_t)>
            handleIncomingData)
    {
        int r      = 0;
        int sockfd = createUDPSocket();
        if (sockfd != -1)
        {
            r = setSocket(sockfd);
        }
        if (sockfd != -1 && r != -1)
        {
            auto addr = setAddr(localip, port);
            r         = bindSocket(sockfd, addr);
            if (r != -1 && !multicast_ip.empty())
            {
                r = addToMultiCast(sockfd, localip, multicast_ip);
            }
        }
        if (sockfd != -1 && r != -1)
        {
            if (handleIncomingData)
            {
                setHandleIncomingData(sockfd, handleIncomingData);
                addToPoll(sockfd);
            }
        }
        else
        {
            closeSocket(sockfd);
            return -1;
        }
        return sockfd;
    }

    int sendData(const char* sendbuf,
        uint32_t sendlen,
        const std::string& toip,
        uint16_t toport,
        int sockfd = -1)
    {
        auto addr = setAddr(toip, toport);
        return sendData(sendbuf, sendlen, addr, sockfd);
    }

    void removeSocket(int sockfd)
    {
        std::lock_guard<std::mutex> guard(__mutex);
        __removeSocketfds.push_back(sockfd);
        __handleIncomingDataMap.erase(sockfd);
        for (int i = 0; i < __fdsCount; i++)
        {
            if (__poolfds[i].fd == sockfd)
            {
                __poolfds[i]                 = __poolfds[__fdsCount - 1];
                __poolfds[__fdsCount - 1].fd = -1;
                __fdsCount--;
                break;
            }
        }
    }

private:
#define MAX_SOCK_SIZE 512  // max number of sockets to poll
    std::thread __pollthread;
    std::atomic<bool> __quit = {true};
    char __buf[N]            = {};
    std::map<int,
        std::function<std::string(const char*,
            uint16_t,
            const std::string& ip,
            uint16_t port)>>
        __handleIncomingDataMap;
    struct pollfd __poolfds[MAX_SOCK_SIZE] = {};
    uint16_t __fdsCount                    = 0;
    std::mutex __mutex;
    std::vector<int> __removeSocketfds;

    SimplePoll()
    {
        start();
    };
    ~SimplePoll()
    {
        if (__pollthread.joinable())
        {
            __pollthread.join();
        }
        closeAndRemoveAll();
    };

    int createUDPSocket()
    {
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        return sockfd;
    }

    int setSocket(int sockfd)
    {
        int r   = 0;
        int yes = 1;
        r = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        if (r == 0)
        {
            r = setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
        }
        return r;
    }

    sockaddr_in setAddr(const std::string& ip, uint16_t port)
    {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        addr.sin_port        = htons(port);
        return addr;
    }

    int bindSocket(int sockfd, const sockaddr_in& addr)
    {
        return ::bind(sockfd,
            reinterpret_cast<const struct sockaddr*>(&addr),
            sizeof(addr));
    }

    void addToPoll(int sockfd)
    {
        std::lock_guard<std::mutex> guard(__mutex);
        __poolfds[__fdsCount].fd     = sockfd;
        __poolfds[__fdsCount].events = POLLIN;
        __fdsCount++;
    }

    void setHandleIncomingData(int sockfd,
        std::function<std::string(const char*,
            uint16_t,
            const std::string& ip,
            uint16_t port)> handleIncomingData)
    {
        std::lock_guard<std::mutex> guard(__mutex);
        __handleIncomingDataMap[sockfd] = handleIncomingData;
    }

    int addToMultiCast(int sockfd,
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
                continue;
            }

            if (ifa->ifa_addr->sa_family == AF_INET)
            {
                if (ifa->ifa_flags & IFF_MULTICAST)
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
                    flag |= (r == 0);
                }
            }
        }
        freeifaddrs(ifAddrList);

        return flag ? 0 : -1;
    }

    void Run()
    {
        while (!__quit)
        {
            struct pollfd poolfds[MAX_SOCK_SIZE] = {};
            uint16_t fds_count;
            {
                std::lock_guard<std::mutex> guard(__mutex);
                fds_count = __fdsCount;
                memcpy(poolfds, __poolfds, sizeof(poolfds[0]) * fds_count);
            }
            auto rc = poll(poolfds, fds_count, 1);
            if (rc < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                else
                {
                    break;
                }
            }
            else if (rc == 0)
            {
                continue;
            }
            else
            {
                for (int i = 0; i < fds_count; i++)
                {
                    checkAndHandleData(poolfds[i]);
                }
            }
            std::lock_guard<std::mutex> guard(__mutex);
            for (auto sockfd : __removeSocketfds)
            {
                closeSocket(sockfd);
            }
            __removeSocketfds.clear();
        }
    }

    std::tuple<uint32_t, sockaddr_in, std::string, uint16_t> receiveData(
        int sockfd,
        char* buf,
        uint32_t buflen)
    {
        sockaddr_in addr;
        (void)memset(&addr, 0, sizeof(addr));
        socklen_t addrLen = sizeof(addr);
        uint16_t len      = recvfrom(sockfd,
            buf,
            buflen,
            0,
            reinterpret_cast<sockaddr*>(&addr),
            &addrLen);

        char ipbuf[INET_ADDRSTRLEN] = {};
        inet_ntop(AF_INET, &addr.sin_addr, ipbuf, sizeof(ipbuf));
        auto port = ntohs(addr.sin_port);

        return std::make_tuple(len, addr, std::string(ipbuf), port);
    }

    void checkAndHandleData(const pollfd& pfd)
    {
        std::string str;
        uint32_t len;
        sockaddr_in addr;
        std::string ip;
        uint16_t port;
        if (pfd.revents & POLLIN)
        {
            std::tie(len, addr, ip, port) = receiveData(pfd.fd, __buf, N);
            std::lock_guard<std::mutex> guard(__mutex);
            if (__handleIncomingDataMap[pfd.fd])
            {
                str = std::move(
                    __handleIncomingDataMap[pfd.fd](__buf, len, ip, port));
            }
            // handle incomming data
        }
        else if (pfd.revents & POLLOUT)
        {
            // handle outgoing data
        }
        else if (pfd.revents & POLLERR)
        {
            // handle error
        }
        else if (pfd.revents & POLLHUP)
        {
            // handle hangup
        }
        else if (pfd.revents & POLLNVAL)
        {
            // handle invalid request
        }
        if (str.size() > 0)
        {
            sendto(pfd.fd,
                str.c_str(),
                str.size(),
                0,
                reinterpret_cast<const sockaddr*>(&addr),
                sizeof(addr));
        }
    }

    int sendData(const char* sendbuf,
        uint32_t sendlen,
        sockaddr_in& addr,
        int sockfd = -1)
    {
        int sendsockfd = sockfd;
        if (sendsockfd == -1)
        {
            sendsockfd = createUDPSocket();
        }

        auto r = sendto(sendsockfd,
            sendbuf,
            sendlen,
            0,
            reinterpret_cast<sockaddr*>(&addr),
            sizeof(addr));
        if (sockfd == -1)
        {
            closeSocket(sendsockfd);
        }
        return r;
    }

    void closeAndRemoveAll()
    {
        std::lock_guard<std::mutex> guard(__mutex);
        __handleIncomingDataMap.clear();

        for (int i = 0; i < __fdsCount; i++)
        {
            closeSocket(__poolfds[i].fd);
        }
        __fdsCount = 0;

        for (auto sockfd : __removeSocketfds)
        {
            closeSocket(sockfd);
        }
        __removeSocketfds.clear();
    }

    void closeSocket(int& sockfd)
    {
        if (sockfd != -1)
        {
            close(sockfd);
        }
        sockfd = -1;
    }
};