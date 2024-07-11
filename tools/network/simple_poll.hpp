#pragma once

#include <poll.h>
#include <stdint.h>
#include <string.h>
#include <mutex>
#include <map>
#include <memory>
#include "control_fd.hpp"
#include "network_global.hpp"

namespace network
{
class SimplePoll : public ControlFd
{
public:
    SimplePoll()
    {
        AddListenSocket(this->__control_socket);
    }

    void AddListenSocket(std::shared_ptr<ProtocolBase> t)
    {
        auto fd = t->GetSocket();
        std::lock_guard<std::mutex> guard(__mutex);
        __pollfds[__fds_count].fd      = fd;
        __pollfds[__fds_count].events  = POLLIN;
        __pollfds[__fds_count].revents = 0;
        __fd2T[fd]                     = t;
        ++__fds_count;
        this->Continue();
    }

    void RemoveListenSocket(ProtocolBase* t)
    {
        auto fd = t->GetSocket();
        std::lock_guard<std::mutex> guard(__mutex);
        for (int i = 0; i < __fds_count; i++)
        {
            if (__pollfds[i].fd == fd)
            {
                __pollfds[i].fd = __pollfds[__fds_count - 1].fd;
                --__fds_count;
                break;
            }
        }
        __fd2T.erase(fd);
        this->Continue();
    }

protected:
    void NetWorkRunOnce()
    {
        int nfds                             = 0;
        struct pollfd pollfds[MAX_SOCK_SIZE] = {};
        {
            std::lock_guard<std::mutex> guard(__mutex);
            memcpy(pollfds, __pollfds, sizeof(pollfds[0]) * __fds_count);
            nfds = __fds_count;
        }
        auto rc = poll(pollfds, nfds, -1);
        if (rc < 0 && errno != EINTR)
        {
            printf("errno:%d %s\n", errno, strerror(errno));
        }
        else if (rc == 0)
        {
            printf("Client closed connection\n");
        }
        else
        {
            std::lock_guard<std::mutex> guard(__mutex);
            for (int i = 0; i < nfds; i++)
                HandleData(pollfds[i]);
        }
    };

private:
    struct pollfd __pollfds[MAX_SOCK_SIZE] = {};
    uint32_t __fds_count                   = 0;
    std::mutex __mutex;
    char __buf[MAX_BUF_SIZE] = {0};
    std::map<int, std::shared_ptr<ProtocolBase>> __fd2T;

    void HandleData(const pollfd& pfd)
    {
        std::string response;
        if (pfd.revents & POLLIN)
        {
            __fd2T[pfd.fd]->Recv(__buf, MAX_BUF_SIZE);
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
    }
};
}  // namespace network
