#pragma once

#include "../PlatformUtils.hpp"

#ifdef _WIN32
    #include "simplepollwin32.h"
#else
    #include "simplepollunix.hpp"
#endif

template <typename T>
void CLOSESOCKET(T& socket)
{
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

template <size_t N = 65536>
class SimplePoll : public SimplePollImpl<N>
{
public:
    SimplePoll()
    {
        start();
    };
    ~SimplePoll()
    {
        wait();
    };

    void wait()
    {
        if (m_thread.joinable())
        {
            m_thread.join();
        }
    }

    void stop()
    {
        sendStop();
        wait();
    }

    void start()
    {
        if (!m_quit)
        {
            return;
        }
        else
        {
            m_controlfd = bindAndSetCB("127.0.0.1",
                0,
                "",
                std::bind(&SimplePoll::handleIncomingData,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3,
                    std::placeholders::_4));
            setControlPort(m_controlfd);
            m_quit   = false;
            m_thread = std::thread([this]() {
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
        int result = 0;
        int sockfd = createUDPSocket();
        SimpleLog::logOut("createUDPSocket sockfd:",
            sockfd,
            " localip:",
            localip,
            " multicast_ip ip:",
            multicast_ip,
            " port:",
            port);
        if (sockfd != -1)
        {
            result = sockfd;
            int r  = setSocket(sockfd);
            if (r != -1)
            {
                auto addr = setAddr(localip, port);
                r         = bindSocket(sockfd, addr);
                SimpleLog::logOut("bindSocket r:", r);
                if (r != -1)
                {
                    if (!multicast_ip.empty())
                    {
                        r = addToMultiCast(sockfd, localip, multicast_ip);
                        SimpleLog::logOut("addToMultiCast r:", r);
                        if (r != -1)
                        { }
                        else
                        {
                            closeSocket(sockfd);
                            result = -4;
                        }
                    }
                }
                else
                {
                    closeSocket(sockfd);
                    result = -3;
                }
            }
            else
            {
                closeSocket(sockfd);
                result = -2;
            }
        }
        else
        {
            result = -1;
        }
        if (result > 0)
        {
            if (handleIncomingData)
            {
                setHandleIncomingData(sockfd, handleIncomingData);
                addToPoll(sockfd);
            }
            setSpecialSendNIC(sockfd);
            sendContinue();
        }
        return result;
    }

    static int sendData(const char* sendbuf,
        uint32_t sendlen,
        const std::string& toip,
        uint16_t toport,
        int sockfd = -1)
    {
        auto addr = setAddr(toip, toport);
        if (sockfd != -1)
            return send(sendbuf, sendlen, addr, sockfd);
        else
            return send(sendbuf, sendlen, addr);
    }

    void removeSocket(int sockfd)
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_removeSocketfds.push_back(sockfd);
        m_handleIncomingDataMap.erase(sockfd);
        for (int i = 0; i < m_fdsCount; i++)
        {
            if (m_poolfds[i].fd == sockfd)
            {
                m_poolfds[i]                 = m_poolfds[m_fdsCount - 1];
                m_poolfds[m_fdsCount - 1].fd = -1;
                m_fdsCount--;
                break;
            }
        }
    }

    std::set<std::string> getLocalIPs()
    {
        return SimplePollImpl<N>::getLocalIPsImpl();
    }

private:
#define MAX_SOCK_SIZE 512  // max number of sockets to poll
    std::thread m_thread;
    std::atomic<bool> m_quit = {true};
    char m_buf[N]            = {};
    std::map<int,
        std::function<std::string(const char*,
            uint16_t,
            const std::string& ip,
            uint16_t port)>>
        m_handleIncomingDataMap;
    struct pollfd m_poolfds[MAX_SOCK_SIZE] = {};
    uint16_t m_fdsCount                    = 0;
    std::mutex m_mutex;
    std::vector<int> m_removeSocketfds;
    int m_controlfd        = -1;
    uint16_t m_controlPort = 0;
    enum class MessageType : uint8_t
    {
        NONE,
        START,
        STOP,
        CONTINUE,
    };

    static int createUDPSocket()
    {
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        return sockfd;
    }

    int setSocket(int sockfd)
    {
        return SimplePollImpl<N>::setSocketImpl(sockfd);
    }

    static sockaddr_in setAddr(const std::string& ip, uint16_t port)
    {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        addr.sin_port        = htons(port);
        return addr;
    }

    static int bindSocket(int sockfd, const sockaddr_in& addr)
    {
        return ::bind(sockfd,
            reinterpret_cast<const struct sockaddr*>(&addr),
            sizeof(addr));
    }

    void addToPoll(int sockfd)
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_poolfds[m_fdsCount].fd     = sockfd;
        m_poolfds[m_fdsCount].events = POLLIN;
        m_fdsCount++;
    }

    void setHandleIncomingData(int sockfd,
        std::function<std::string(const char*,
            uint16_t,
            const std::string& ip,
            uint16_t port)> handleIncomingData)
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_handleIncomingDataMap[sockfd] = handleIncomingData;
    }

    int addToMultiCast(int sockfd,
        const std::string& localip,
        const std::string& multicastip)
    {
        return SimplePollImpl<N>::addToMultiCastImpl(
            sockfd, localip, multicastip);
    }

    void Run()
    {
        pthread_setname_np(pthread_self(), "ACCM-SimplePoll");
        while (!m_quit)
        {
            struct pollfd poolfds[MAX_SOCK_SIZE] = {};
            uint16_t fds_count;
            {
                std::lock_guard<std::mutex> guard(m_mutex);
                fds_count = m_fdsCount;
                memcpy(poolfds, m_poolfds, sizeof(poolfds[0]) * fds_count);
            }
            auto rc = POLL(poolfds, fds_count, -1);
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

            std::lock_guard<std::mutex> guard(m_mutex);
            for (auto sockfd : m_removeSocketfds)
            {
                closeSocket(sockfd);
            }
            m_removeSocketfds.clear();
        }
        closeAndRemoveAll();
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
            memset(m_buf, 0, N);
            std::tie(len, addr, ip, port) = receiveData(pfd.fd, m_buf, N);
            std::lock_guard<std::mutex> guard(m_mutex);
            if (m_handleIncomingDataMap[pfd.fd])
            {
                str = std::move(
                    m_handleIncomingDataMap[pfd.fd](m_buf, len, ip, port));
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
            send(str.c_str(), str.size(), addr, pfd.fd);
        }
    }

    static int send(const char* sendbuf, uint32_t sendlen, sockaddr_in& addr)
    {
        auto sendsockfd = createUDPSocket();
        auto r          = send(sendbuf, sendlen, addr, sendsockfd);
        closeSocket(sendsockfd);

        return r;
    }

    static int send(const char* sendbuf,
        uint32_t sendlen,
        sockaddr_in& addr,
        int sockfd)
    {
        auto r = sendto(sockfd,
            sendbuf,
            sendlen,
            0,
            reinterpret_cast<sockaddr*>(&addr),
            sizeof(addr));
        if (r < 0)
        {
            printf("sendto error: %s\n", strerror(errno));
        }
        return r;
    }

    void closeAndRemoveAll()
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_handleIncomingDataMap.clear();

        for (int i = 0; i < m_fdsCount; i++)
        {
            closeSocket(m_poolfds[i].fd);
        }
        m_fdsCount = 0;

        for (auto sockfd : m_removeSocketfds)
        {
            closeSocket(sockfd);
        }
        closeSocket(m_controlfd);
        m_removeSocketfds.clear();
    }

    template <typename T>
    static void closeSocket(T& sockfd)
    {
        CLOSESOCKET(sockfd);
        sockfd = -1;
    }

    std::string getSpecialSendNIC() override
    {
        auto env = getenv("ACFW_CM_MULICAST_SEND_NIC");
        if (env)
        {
            int r      = -1;
            int sockfd = createUDPSocket();
            auto addr  = setAddr(std::string(env), 0);
            r          = bindSocket(sockfd, addr);
            CLOSESOCKET(sockfd);
            if (r == 0)
            {
                return std::string(env);
            }
            else
            {
                return "";
            }
        }
        else
        {
            return "";
        }
    }

    int setSpecialSendNIC(int sockfd, std::string ip) override
    {
        int r = -1;
        if (!ip.empty())
        {
            struct in_addr localInterface;
            localInterface.s_addr = inet_addr(ip.c_str());
            r                     = setsockopt(sockfd,
                IPPROTO_IP,
                IP_MULTICAST_IF,
                reinterpret_cast<char*>(&localInterface),
                sizeof(localInterface));
            SimpleLog::logOut("ACFW_CM_MULICAST_SEND_NIC:", ip, " r:", r);
        }
        else
        {
            SimpleLog::logOut("setSpecialSendNIC IP empty");
        }
        return r;
    }

    int setSpecialSendNIC(int sockfd)
    {
        return SimplePollImpl<N>::setSpecialSendNICImpl(sockfd);
    }

    std::string handleIncomingData(const char* buf,
        uint16_t len,
        const std::string& ip,
        uint16_t port)
    {
        (void)ip;
        (void)port;
        if (len == 5)
        {
            if (std::string(buf, 4) == "ACCM")
            {
                uint8_t control = buf[4];
                SimpleLog::logOut(
                    "handleIncomingData control fd recv:", (int)control);
                if (control == (uint8_t)MessageType::STOP)
                {
                    m_quit = true;
                }
                else if (control == (uint8_t)MessageType::CONTINUE)
                { }
            }
        }
        return "";
    }

    void sendStop()
    {
        if (m_controlfd != -1)
        {
            char buf[5] = "ACCM";
            buf[4]      = (uint8_t)MessageType::STOP;
            sendData(buf, sizeof(buf), "127.0.0.1", m_controlPort, m_controlfd);
        }
    }

    void sendContinue()
    {
        if (m_controlfd != -1)
        {
            char buf[5] = "ACCM";
            buf[4]      = (uint8_t)MessageType::CONTINUE;
            sendData(buf, sizeof(buf), "127.0.0.1", m_controlPort, m_controlfd);
        }
    }

    void setControlPort(int fd)
    {
        if (fd != -1)
        {
            struct sockaddr_in addr;
            socklen_t addr_size = sizeof(addr);
            getsockname(fd, (struct sockaddr*)&addr, &addr_size);
            m_controlPort = ntohs(addr.sin_port);
        }
    }
};