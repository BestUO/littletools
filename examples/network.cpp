#include <iostream>
#include <memory>
#include <thread>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/network/udp.hpp"
#include "tools/network/simple_poll.hpp"
#include "tools/network/simple_epoll.hpp"
#include "tools/network/network_manager.hpp"
#include "tools/cpu_bind.hpp"

TEST_CASE("network_UDP_inet_base")
{
    network::inet_udp::UDP udp1, udp2;
    udp1.SetReuseAddrAndPort();
    udp1.SetAddr("127.0.0.1", 0);
    udp1.SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });

    udp2.SetReuseAddrAndPort();
    udp2.SetAddr("127.0.0.1", 12345);
    udp2.SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    udp1.Send("from udp1", udp2.GetAddr());

    char buf[100] = {0};
    udp2.Recv();
    udp1.Recv();
}

TEST_CASE("network_UDP_inet_multicast")
{
    network::inet_udp::UDP udp1, udp2;

    udp1.SetAddr("0.0.0.0", 12345);
    udp1.SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });

    udp2.SetAddr("0.0.0.0", 23456);
    udp2.AddMultiCast("234.56.78.90");
    udp2.SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    udp1.Send(
        "from udp1", network::SocketBase::CreateAddr("234.56.78.90", 23456));

    char buf[100] = {0};
    udp2.Recv();
    udp1.Recv();
}

TEST_CASE("network_UDP_unix_base")
{
    network::unix_udp::UDP udp1, udp2;
    udp1.SetReuseAddrAndPort();
    udp1.SetAddr("/tmp/unix_socket_test1");
    udp1.SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });

    udp2.SetReuseAddrAndPort();
    udp2.SetAddr("/tmp/unix_socket_test2");
    udp2.SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    udp1.Send("from udp1", udp2.GetAddr());

    char buf[100] = {0};
    udp2.Recv();
    udp1.Recv();
}

TEST_CASE("network_SimplePoll")
{
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();
    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_SimplePoll_UDP_inet_base_add_poll")
{
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();

    auto udp1 = std::make_shared<network::inet_udp::UDP<>>();
    auto udp2 = std::make_shared<network::inet_udp::UDP<>>();
    udp1->SetReuseAddrAndPort();
    udp1->SetAddr("127.0.0.1", 0);
    udp1->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });
    udp2->SetReuseAddrAndPort();
    udp2->SetAddr("127.0.0.1", 12345);
    udp2->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    network_manager.AddListenSocket(udp1);
    network_manager.AddListenSocket(udp2);

    udp1->Send("from udp1", udp2->GetAddr());
    udp1->Send(
        "from udp1", network::SocketBase::CreateAddr("127.0.0.1", 12345));
    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_SimplePoll_UDP_inet_multicast_add_poll")
{
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();

    auto udp1 = std::make_shared<network::inet_udp::UDP<>>();
    auto udp2 = std::make_shared<network::inet_udp::UDP<>>();

    udp1->SetAddr("0.0.0.0", 12345);
    udp1->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });

    udp2->SetAddr("0.0.0.0", 23456);
    udp2->AddMultiCast("234.56.78.90");
    udp2->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    network_manager.AddListenSocket(udp1);
    network_manager.AddListenSocket(udp2);

    udp1->Send(
        "from udp1", network::SocketBase::CreateAddr("234.56.78.90", 23456));

    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_SimplePoll_UDP_unix_base_add_poll")
{
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();

    auto udp1 = std::make_shared<network::unix_udp::UDP<>>();
    auto udp2 = std::make_shared<network::unix_udp::UDP<>>();
    udp1->SetReuseAddrAndPort();
    udp1->SetAddr("/tmp/unix_socket_test1");
    udp1->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });
    udp2->SetReuseAddrAndPort();
    udp2->SetAddr("/tmp/unix_socket_test2");
    udp2->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    network_manager.AddListenSocket(udp1);
    network_manager.AddListenSocket(udp2);

    udp1->Send("from udp1", udp2->GetAddr());
    udp1->Send(
        "from udp1", network::SocketBase::CreateAddr("/tmp/unix_socket_test2"));
    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_SimplePoll_UDP_merge_inet_unix")
{
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();

    auto udp1 = std::make_shared<network::unix_udp::UDP<>>();
    auto udp2 = std::make_shared<network::unix_udp::UDP<>>();
    udp1->SetReuseAddrAndPort();
    udp1->SetAddr("/tmp/unix_socket_test1");
    udp1->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });
    udp2->SetReuseAddrAndPort();
    udp2->SetAddr("/tmp/unix_socket_test2");
    udp2->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    auto udp3 = std::make_shared<network::inet_udp::UDP<>>();
    auto udp4 = std::make_shared<network::inet_udp::UDP<>>();
    udp3->SetReuseAddrAndPort();
    udp3->SetAddr("127.0.0.1", 0);
    udp3->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp4");
            return "";
        });
    udp4->SetReuseAddrAndPort();
    udp4->SetAddr("127.0.0.1", 12345);
    udp4->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp3");
            return "from udp4";
        });

    network_manager.AddListenSocket(udp1);
    network_manager.AddListenSocket(udp2);
    network_manager.AddListenSocket(udp3);
    network_manager.AddListenSocket(udp4);

    udp1->Send("from udp1", udp2->GetAddr());
    udp1->Send(
        "from udp1", network::SocketBase::CreateAddr("/tmp/unix_socket_test2"));
    udp3->Send("from udp3", udp4->GetAddr());
    udp3->Send(
        "from udp3", network::SocketBase::CreateAddr("127.0.0.1", 12345));
    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_SimplePoll_UDP_inet_performance")
{
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();

    auto recvudp = std::make_shared<network::inet_udp::UDP<>>();
    recvudp->SetAddr("127.0.0.1", 23456);
    int i = 0;
    recvudp->SetCallBack([&]<typename... Args>(Args... args) -> std::string {
        i++;
        return "";
    });
    network_manager.AddListenSocket(recvudp);

    std::thread sendthread([] {
        CPUBind::BindCPU();
        network::inet_udp::UDP sendudp;
        sendudp.SetAddr("127.0.0.1", 12345);
        auto endpoint = network::SocketBase::CreateAddr("127.0.0.1", 23456);
        std::string data(200, 'a');
        usleep(500000);
        int j    = 0;
        auto now = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - now)
                   .count()
            < 10)
        {
            j++;
            sendudp.Send(data, endpoint);
        }
        std::cout << "send num: " << j / 10 << " ";
    });
    sleep(2);
    sendthread.join();
    std::cout << "simple_poll udp inet qps: " << i / 10 << std::endl;
    network_manager.Stop();
}

TEST_CASE("network_SimplePoll_UDP_unix_performance")
{
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();

    auto recvudp = std::make_shared<network::unix_udp::UDP<>>();
    recvudp->SetAddr("/tmp/123");
    int i = 0;
    recvudp->SetCallBack([&]<typename... Args>(Args... args) -> std::string {
        i++;
        return "";
    });
    network_manager.AddListenSocket(recvudp);

    std::thread sendthread([] {
        CPUBind::BindCPU();
        network::unix_udp::UDP sendudp;
        sendudp.SetAddr("/tmp/456");
        auto endpoint = network::SocketBase::CreateAddr("/tmp/123");
        std::string data(200, 'a');
        usleep(500000);
        int j    = 0;
        auto now = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - now)
                   .count()
            < 10)
        {
            j++;
            sendudp.Send(data, endpoint);
        }
        std::cout << "send num: " << j / 10 << " ";
    });
    sleep(2);
    sendthread.join();
    std::cout << "simple_poll udp unix qps: " << i / 10 << std::endl;
    network_manager.Stop();
}

TEST_CASE("network_SimplePoll_nonblock_UDP_inet_performance")
{
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();

    auto recvudp = std::make_shared<network::inet_udp::UDP<>>();
    recvudp->SetAddr("127.0.0.1", 23456);
    int i = 0;
    recvudp->SetCallBack([&]<typename... Args>(Args... args) -> std::string {
        i++;
        return "";
    });
    recvudp->setNonBlocking();
    network_manager.AddListenSocket(recvudp);

    std::thread sendthread([] {
        CPUBind::BindCPU();
        network::inet_udp::UDP sendudp;
        sendudp.SetAddr("127.0.0.1", 12345);
        auto endpoint = network::SocketBase::CreateAddr("127.0.0.1", 23456);
        std::string data(200, 'a');
        usleep(500000);
        int j    = 0;
        auto now = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - now)
                   .count()
            < 10)
        {
            j++;
            sendudp.Send(data, endpoint);
        }
        std::cout << "send num: " << j / 10 << " ";
    });
    sleep(2);
    sendthread.join();
    std::cout << "simple_poll nonblock udp inet qps: " << i / 10 << std::endl;
    network_manager.Stop();
}

TEST_CASE("network_SimplePoll_nonblock_UDP_unix_performance")
{
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();

    auto recvudp = std::make_shared<network::unix_udp::UDP<>>();
    recvudp->SetAddr("/tmp/123");
    int i = 0;
    recvudp->SetCallBack([&]<typename... Args>(Args... args) -> std::string {
        i++;
        return "";
    });
    recvudp->setNonBlocking();
    network_manager.AddListenSocket(recvudp);

    std::thread sendthread([] {
        CPUBind::BindCPU();
        network::unix_udp::UDP sendudp;
        sendudp.SetAddr("/tmp/456");
        auto endpoint = network::SocketBase::CreateAddr("/tmp/123");
        std::string data(200, 'a');
        usleep(500000);
        int j    = 0;
        auto now = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - now)
                   .count()
            < 10)
        {
            j++;
            sendudp.Send(data, endpoint);
        }
        std::cout << "send num: " << j / 10 << " ";
    });
    sleep(2);
    sendthread.join();
    std::cout << "simple_poll nonblock udp unix qps: " << i / 10 << std::endl;
    network_manager.Stop();
}

TEST_CASE("network_SimpleEpoll")
{
    network::NetWorkManager<network::SimpleEpoll> network_manager;
    network_manager.Start();
    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_SimpleEpoll_UDP_inet_base_add_poll")
{
    network::NetWorkManager<network::SimpleEpoll> network_manager;
    network_manager.Start();

    auto udp1 = std::make_shared<network::inet_udp::UDP<>>();
    auto udp2 = std::make_shared<network::inet_udp::UDP<>>();
    udp1->SetReuseAddrAndPort();
    udp1->SetAddr("127.0.0.1", 0);
    udp1->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });
    udp2->SetReuseAddrAndPort();
    udp2->SetAddr("127.0.0.1", 12345);
    udp2->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    network_manager.AddListenSocket(udp1);
    network_manager.AddListenSocket(udp2);

    udp1->Send("from udp1", udp2->GetAddr());
    udp1->Send(
        "from udp1", network::SocketBase::CreateAddr("127.0.0.1", 12345));
    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_SimpleEpoll_UDP_inet_multicast_add_poll")
{
    network::NetWorkManager<network::SimpleEpoll> network_manager;
    network_manager.Start();

    auto udp1 = std::make_shared<network::inet_udp::UDP<>>();
    auto udp2 = std::make_shared<network::inet_udp::UDP<>>();

    udp1->SetAddr("0.0.0.0", 12345);
    udp1->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });

    udp2->SetAddr("0.0.0.0", 23456);
    udp2->AddMultiCast("234.56.78.90");
    udp2->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    network_manager.AddListenSocket(udp1);
    network_manager.AddListenSocket(udp2);

    udp1->Send(
        "from udp1", network::SocketBase::CreateAddr("234.56.78.90", 23456));

    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_SimpleEpoll_UDP_unix_base_add_poll")
{
    network::NetWorkManager<network::SimpleEpoll> network_manager;
    network_manager.Start();

    auto udp1 = std::make_shared<network::unix_udp::UDP<>>();
    auto udp2 = std::make_shared<network::unix_udp::UDP<>>();
    udp1->SetReuseAddrAndPort();
    udp1->SetAddr("/tmp/unix_socket_test1");
    udp1->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });
    udp2->SetReuseAddrAndPort();
    udp2->SetAddr("/tmp/unix_socket_test2");
    udp2->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    network_manager.AddListenSocket(udp1);
    network_manager.AddListenSocket(udp2);

    udp1->Send("from udp1", udp2->GetAddr());
    udp1->Send(
        "from udp1", network::SocketBase::CreateAddr("/tmp/unix_socket_test2"));
    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_SimpleEpoll_UDP_merge_inet_unix")
{
    network::NetWorkManager<network::SimpleEpoll> network_manager;
    network_manager.Start();

    auto udp1 = std::make_shared<network::unix_udp::UDP<>>();
    auto udp2 = std::make_shared<network::unix_udp::UDP<>>();
    udp1->SetReuseAddrAndPort();
    udp1->SetAddr("/tmp/unix_socket_test1");
    udp1->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp2");
            return "";
        });
    udp2->SetReuseAddrAndPort();
    udp2->SetAddr("/tmp/unix_socket_test2");
    udp2->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp1");
            return "from udp2";
        });

    auto udp3 = std::make_shared<network::inet_udp::UDP<>>();
    auto udp4 = std::make_shared<network::inet_udp::UDP<>>();
    udp3->SetReuseAddrAndPort();
    udp3->SetAddr("127.0.0.1", 0);
    udp3->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp4");
            return "";
        });
    udp4->SetReuseAddrAndPort();
    udp4->SetAddr("127.0.0.1", 12345);
    udp4->SetCallBack(
        [](const char* data, size_t size, const sockaddr&) -> std::string {
            std::string str(data, size);
            CHECK(str == "from udp3");
            return "from udp4";
        });

    network_manager.AddListenSocket(udp1);
    network_manager.AddListenSocket(udp2);
    network_manager.AddListenSocket(udp3);
    network_manager.AddListenSocket(udp4);

    udp1->Send("from udp1", udp2->GetAddr());
    udp1->Send(
        "from udp1", network::SocketBase::CreateAddr("/tmp/unix_socket_test2"));
    udp3->Send("from udp3", udp4->GetAddr());
    udp3->Send(
        "from udp3", network::SocketBase::CreateAddr("127.0.0.1", 12345));
    usleep(1000);
    network_manager.Stop();
}

TEST_CASE("network_SimpleEpoll_level_UDP_inet_performance")
{
    network::NetWorkManager<network::SimpleEpoll> network_manager;
    network_manager.Start();

    auto recvudp = std::make_shared<network::inet_udp::UDP<>>();
    recvudp->SetAddr("127.0.0.1", 23456);
    int i = 0;
    recvudp->SetCallBack([&]<typename... Args>(Args... args) -> std::string {
        i++;
        return "";
    });
    network_manager.AddListenSocket(recvudp);

    std::thread sendthread([] {
        CPUBind::BindCPU();
        network::inet_udp::UDP sendudp;
        sendudp.SetAddr("127.0.0.1", 12345);
        auto endpoint = network::SocketBase::CreateAddr("127.0.0.1", 23456);
        std::string data(200, 'a');
        usleep(500000);
        int j    = 0;
        auto now = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - now)
                   .count()
            < 10)
        {
            j++;
            sendudp.Send(std::to_string(j), endpoint);
        }
        std::cout << "send num: " << j / 10 << " ";
    });
    sleep(2);
    sendthread.join();
    std::cout << "simple_Epoll udp inet qps: " << i / 10 << std::endl;
    network_manager.Stop();
}

TEST_CASE("network_SimpleEpoll_level_UDP_unix_performance")
{
    network::NetWorkManager<network::SimpleEpoll> network_manager;
    network_manager.Start();

    auto recvudp = std::make_shared<network::unix_udp::UDP<>>();
    recvudp->SetAddr("/tmp/123");
    int i = 0;
    recvudp->SetCallBack([&]<typename... Args>(Args... args) -> std::string {
        i++;
        return "";
    });
    network_manager.AddListenSocket(recvudp);

    std::thread sendthread([] {
        CPUBind::BindCPU();
        network::unix_udp::UDP sendudp;
        sendudp.SetAddr("/tmp/456");
        auto endpoint = network::SocketBase::CreateAddr("/tmp/123");
        std::string data(200, 'a');
        usleep(500000);
        int j    = 0;
        auto now = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - now)
                   .count()
            < 10)
        {
            j++;
            sendudp.Send(data, endpoint);
        }
        std::cout << "send num: " << j / 10 << " ";
    });
    sleep(2);
    sendthread.join();
    std::cout << "simple_Epoll udp unix qps: " << i / 10 << std::endl;
    network_manager.Stop();
}

TEST_CASE("network_SimpleEpoll_edge_UDP_inet_performance")
{
    network::NetWorkManager<network::SimpleEpoll> network_manager;
    network_manager.Start();

    auto recvudp = std::make_shared<network::inet_udp::UDP<>>();
    recvudp->SetAddr("127.0.0.1", 23456);
    int i = 0;
    recvudp->SetCallBack([&]<typename... Args>(Args... args) -> std::string {
        i++;
        return "";
    });
    recvudp->setNonBlocking();
    network_manager.AddListenSocket(recvudp, true);

    std::thread sendthread([] {
        CPUBind::BindCPU();
        network::inet_udp::UDP sendudp;
        sendudp.SetAddr("127.0.0.1", 12345);
        auto endpoint = network::SocketBase::CreateAddr("127.0.0.1", 23456);
        std::string data(200, 'a');
        usleep(500000);
        int j    = 0;
        auto now = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - now)
                   .count()
            < 1)
        {
            j++;
            sendudp.Send(std::to_string(j), endpoint);
        }
        std::cout << "send num: " << j << " ";
    });
    sleep(2);
    sendthread.join();
    std::cout << "simple_Epoll edge udp inet qps: " << i << std::endl;
    network_manager.Stop();
}

TEST_CASE("network_SimpleEpoll_edge_UDP_unix_performance")
{
    network::NetWorkManager<network::SimpleEpoll> network_manager;
    network_manager.Start();

    auto recvudp = std::make_shared<network::unix_udp::UDP<>>();
    recvudp->SetAddr("/tmp/123");
    int i = 0;
    recvudp->SetCallBack([&]<typename... Args>(Args... args) -> std::string {
        i++;
        return "";
    });
    recvudp->setNonBlocking();
    network_manager.AddListenSocket(recvudp, true);

    std::thread sendthread([] {
        CPUBind::BindCPU();
        network::unix_udp::UDP sendudp;
        sendudp.SetAddr("/tmp/456");
        auto endpoint = network::SocketBase::CreateAddr("/tmp/123");
        std::string data(200, 'a');
        usleep(500000);
        int j    = 0;
        auto now = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - now)
                   .count()
            < 1)
        {
            j++;
            sendudp.Send(data, endpoint);
        }
        std::cout << "send num: " << j << " ";
    });
    sleep(2);
    sendthread.join();
    std::cout << "simple_Epoll edge udp unix qps: " << i << std::endl;
    network_manager.Stop();
}