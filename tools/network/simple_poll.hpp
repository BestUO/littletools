#pragma once

#include <poll.h>
#include <stdint.h>
#include <string.h>
#include <bitset>
#include <mutex>
#include <map>
#include "control_fd.hpp"
#include "network_params.hpp"

namespace network
{
template <SocketType T>
class SimplePoll : public ControlFd<T>
{
protected:
    SimplePoll()
    {
        AddSocket(this->__control_socket);
    }

    void AddSocket(const T& t)
    {
        auto fd = t.GetSocket();
        std::lock_guard<std::mutex> guard(__mutex);
        __pollfds[__fds_count].fd      = fd;
        __pollfds[__fds_count].events  = POLLIN;
        __pollfds[__fds_count].revents = 0;
        __fd2T[fd]                     = t;
        ++__fds_count;
        this->Continue();
    }

    void RemoveSocket(const T& t)
    {
        auto fd = t.GetSocket();
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
        t.Close();
        this->Continue();
    }

    void NetWorkRunOnce()
    {
        struct pollfd pollfds[MAX_SOCK_SIZE] = {};
        {
            std::lock_guard<std::mutex> guard(__mutex);
            memcpy(pollfds, __pollfds, sizeof(pollfds[0]) * __fds_count);
        }
        auto rc = poll(pollfds, __fds_count, -1);
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
            for (int i = 0; i < __fds_count; i++)
                HandleData(pollfds[i]);
        }
    };

private:
    struct pollfd __pollfds[MAX_SOCK_SIZE] = {};
    uint32_t __fds_count                   = 0;
    std::mutex __mutex;
    char __buf[MAX_BUF_SIZE] = {0};
    std::map<int, T> __fd2T;

    void HandleData(const pollfd& pfd)
    {
        std::string response;
        if (pfd.revents & POLLIN)
        {
            __fd2T[pfd.fd].Recv(__buf, MAX_BUF_SIZE);
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
