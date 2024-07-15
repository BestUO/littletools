#pragma once

#include <stdint.h>
#include <string.h>
#include <sys/epoll.h>
#include <mutex>
#include <map>
#include <memory>
#include "control_fd.hpp"
#include "network_global.hpp"

namespace network
{
class SimpleEpoll : public ControlFd
{
public:
    SimpleEpoll()
    {
        __epoll_fd = epoll_create1(0);
        AddListenSocket(this->__control_socket);
    }

    void AddListenSocket(std::shared_ptr<ProtocolBase> t, bool edge = false)
    {
        int fd = t->GetSocket();
        std::lock_guard<std::mutex> guard(__mutex);
        epoll_event ev = {EPOLLIN | EPOLLERR | EPOLLHUP | EPOLLPRI
                | (edge ? EPOLLET : EPOLLIN),
            {.fd = fd}};
        epoll_ctl(__epoll_fd, EPOLL_CTL_ADD, fd, &ev);
        __fd2T[fd] = t;
        this->Continue();
    }

    void RemoveListenSocket(std::shared_ptr<ProtocolBase> t)
    {
        auto fd = t->GetSocket();
        t->SetCallBack(nullptr);
        std::lock_guard<std::mutex> guard(__mutex);
        epoll_event ev = {0, {0}};
        epoll_ctl(__epoll_fd, EPOLL_CTL_DEL, fd, &ev);
        __fd2T.erase(fd);

        this->Continue();
    }

protected:
    void NetWorkRunOnce()
    {
        int num_events
            = epoll_wait(__epoll_fd, __recv_events, MAX_SOCK_SIZE, -1);
        for (int i = 0; i < num_events; ++i)
        {
            if (__recv_events[i].events & EPOLLIN)
            {
                std::shared_ptr<ProtocolBase> tmp = nullptr;
                {
                    std::lock_guard<std::mutex> lck(__mutex);
                    if (__fd2T.find(__recv_events[i].data.fd) != __fd2T.end())
                        tmp = __fd2T[__recv_events[i].data.fd];
                }
                if (tmp)
                    tmp->Recv(__buf, MAX_BUF_SIZE);
            }
            else if (__recv_events[i].events & EPOLLOUT)
            {
                // handle outgoing data
            }
            else if (__recv_events[i].events & EPOLLERR)
            {
                // handle error
            }
            else if (__recv_events[i].events & EPOLLHUP)
            {
                // handle hangup
            }
        }
    };

private:
    // struct pollfd __pollfds[MAX_SOCK_SIZE] = {};
    std::mutex __mutex;
    char __buf[MAX_BUF_SIZE] = {0};
    std::map<int, std::shared_ptr<ProtocolBase>> __fd2T;
    int __epoll_fd;
    epoll_event __recv_events[MAX_SOCK_SIZE];
};
}  // namespace network