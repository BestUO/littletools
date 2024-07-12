#include <iostream>
#include <unistd.h>
#include <memory>
#include <vector>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/raft/Raft.h"
#include "tools/simplepoll/simplepoll.hpp"

TEST_CASE("Raft_1Node")
{
    SimplePoll<> simplePoll;
    std::string multicastIP = "234.56.78.90";
    uint16_t multicastPort  = 9987;
    Raft raft(RaftInfos::RaftBaseInfo{
        std::chrono::milliseconds{100}, multicastIP, multicastPort, 1});

    simplePoll.bindAndSetCB("0.0.0.0",
        multicastPort,
        multicastIP,
        std::bind(&Raft::HandleIncomingData,
            &raft,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3,
            std::placeholders::_4));
    auto sendsocket = simplePoll.bindAndSetCB("0.0.0.0",
        0,
        "",
        std::bind(&Raft::HandleIncomingData,
            &raft,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3,
            std::placeholders::_4));

    raft.SetSendSocket(sendsocket);
    raft.Start();
    sleep(1);
    CHECK(RaftInfos::Role::LEADER == raft.GetRole());
    simplePoll.stop();
}

TEST_CASE("Raft_10Nodes")
{
    SimplePoll<> simplePoll;
    std::string multicastIP = "234.56.78.90";
    uint16_t multicastPort  = 9987;
    uint8_t num             = 10;
    std::vector<std::shared_ptr<Raft>> rafts;
    std::vector<std::thread> threads;
    for (int i = 0; i < num; i++)
    {
        threads.push_back(std::thread(
            [simplePollptr    = &simplePoll,
                multicastIP   = multicastIP,
                multicastPort = multicastPort,
                num = num](std::vector<std::shared_ptr<Raft>>& vrafts) {
                auto raft = std::make_shared<Raft>(
                    RaftInfos::RaftBaseInfo{std::chrono::milliseconds{100},
                        multicastIP,
                        multicastPort,
                        num});
                simplePollptr->bindAndSetCB("0.0.0.0",
                    multicastPort,
                    multicastIP,
                    std::bind(&Raft::HandleIncomingData,
                        raft,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        std::placeholders::_3,
                        std::placeholders::_4));
                auto sendsocket = simplePollptr->bindAndSetCB("0.0.0.0",
                    0,
                    "",
                    std::bind(&Raft::HandleIncomingData,
                        raft,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        std::placeholders::_3,
                        std::placeholders::_4));

                raft->SetSendSocket(sendsocket);
                vrafts.push_back(raft);
                raft->Start();
            },
            std::ref(rafts)));
    }

    sleep(3);
    int leadernum = 0, fllowernum = 0;
    for (int i = 0; i < num; i++)
    {
        leadernum += rafts[i]->GetRole() == RaftInfos::Role::LEADER ? 1 : 0;
        fllowernum += rafts[i]->GetRole() == RaftInfos::Role::FOLLOWER ? 1 : 0;
    }
    CHECK(1 == leadernum);
    CHECK(num - 1 == fllowernum);

    simplePoll.stop();
    for (int i = 0; i < num; i++)
    {
        threads[i].join();
    }
}

TEST_CASE("Raft_10Nodes2")
{
    SimplePoll<> simplePoll;
    std::string multicastIP = "234.56.78.90";
    uint16_t multicastPort  = 9987;
    uint8_t num             = 10;
    std::vector<std::shared_ptr<Raft>> rafts;
    std::vector<std::thread> threads;
    for (int i = 0; i < num; i++)
    {
        threads.push_back(std::thread(
            [simplePollptr    = &simplePoll,
                multicastIP   = multicastIP,
                multicastPort = multicastPort](
                std::vector<std::shared_ptr<Raft>>& vrafts) {
                auto raft = std::make_shared<Raft>(
                    RaftInfos::RaftBaseInfo{std::chrono::milliseconds{100},
                        multicastIP,
                        multicastPort,
                        1});
                simplePollptr->bindAndSetCB("0.0.0.0",
                    multicastPort,
                    multicastIP,
                    std::bind(&Raft::HandleIncomingData,
                        raft,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        std::placeholders::_3,
                        std::placeholders::_4));
                auto sendsocket = simplePollptr->bindAndSetCB("0.0.0.0",
                    0,
                    "",
                    std::bind(&Raft::HandleIncomingData,
                        raft,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        std::placeholders::_3,
                        std::placeholders::_4));

                raft->SetSendSocket(sendsocket);
                vrafts.push_back(raft);
                raft->Start();
            },
            std::ref(rafts)));
    }

    sleep(3);
    int leadernum = 0, fllowernum = 0;
    for (int i = 0; i < num; i++)
    {
        leadernum += rafts[i]->GetRole() == RaftInfos::Role::LEADER ? 1 : 0;
        fllowernum += rafts[i]->GetRole() == RaftInfos::Role::FOLLOWER ? 1 : 0;
    }
    CHECK(1 == leadernum);
    CHECK(num - 1 == fllowernum);

    simplePoll.stop();
    for (int i = 0; i < num; i++)
    {
        threads[i].join();
    }
}

TEST_CASE("Raft_add1Node")
{
    SimplePoll<> simplePoll;
    std::vector<std::shared_ptr<Raft>> rafts;
    std::string multicastIP = "234.56.78.90";
    uint16_t multicastPort  = 9987;
    for (int i = 0; i < 2; i++)
    {
        auto raft = std::make_shared<Raft>(RaftInfos::RaftBaseInfo{
            std::chrono::milliseconds{100}, multicastIP, multicastPort, 1});

        simplePoll.bindAndSetCB("0.0.0.0",
            multicastPort,
            multicastIP,
            std::bind(&Raft::HandleIncomingData,
                raft,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4));
        auto sendsocket = simplePoll.bindAndSetCB("0.0.0.0",
            0,
            "",
            std::bind(&Raft::HandleIncomingData,
                raft,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                std::placeholders::_4));

        raft->SetSendSocket(sendsocket);
        raft->Start();
        sleep(1);
        CHECK((i == 0 ? RaftInfos::Role::LEADER : RaftInfos::Role::FOLLOWER)
            == raft->GetRole());
        rafts.push_back(raft);
    }
    simplePoll.stop();
}