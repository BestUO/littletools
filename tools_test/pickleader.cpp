#include <cstring>
#include <iostream>
#include <unistd.h>
#include <memory>
#include <vector>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/raft/Raft.h"
#include "tools/network/network_manager.hpp"
#include "tools/network/simple_poll.hpp"
#include "tools/timermanager.hpp"

TEST_CASE("Raft_struct_serialize_CommonInfo")
{
    RaftCommandType::CommonInfo a;
    auto s = a.serialize();
    RaftCommandType::CommonInfo b(s.data(), s.size());
    CHECK(a.messageType == b.messageType);
    CHECK(a.term == b.term);
    CHECK(a.uuid == b.uuid);
    CHECK(memcmp(a.version, b.version, 15) == 0);
    CHECK(a.timestamp == b.timestamp);
}

TEST_CASE("Raft_struct_serialize_HeartBeat")
{
    RaftCommandType::HeartBeat a;
    auto s = a.serialize();
    RaftCommandType::HeartBeat b(s.data(), s.size());
    CHECK(a.commonInfo.uuid == b.commonInfo.uuid);
    CHECK(a.commonInfo.timestamp == b.commonInfo.timestamp);
}

TEST_CASE("Raft_struct_serialize_Vote")
{
    RaftCommandType::Vote a;
    auto s = a.serialize();
    RaftCommandType::Vote b(s.data(), s.size());
    CHECK(a.commonInfo.uuid == b.commonInfo.uuid);
    CHECK(a.commonInfo.timestamp == b.commonInfo.timestamp);
}

TEST_CASE("Raft_struct_serialize_VoteResponse")
{
    RaftCommandType::VoteResponse a;
    a.voteGranted = true;
    auto s        = a.serialize();
    RaftCommandType::VoteResponse b(s.data(), s.size());
    CHECK(a.commonInfo.uuid == b.commonInfo.uuid);
    CHECK(a.commonInfo.timestamp == b.commonInfo.timestamp);
    CHECK(a.voteGranted == b.voteGranted);
}

TEST_CASE("Raft_struct_serialize_VoteResult")
{
    RaftCommandType::VoteResult a;
    auto s = a.serialize();
    RaftCommandType::VoteResult b(s.data(), s.size());
    CHECK(a.commonInfo.uuid == b.commonInfo.uuid);
    CHECK(a.commonInfo.timestamp == b.commonInfo.timestamp);
}

TEST_CASE("Raft_1Node")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();

    Raft raft(RaftInfos::RaftBaseInfo{
        std::chrono::milliseconds{100}, "234.56.78.90", "", 9987, 1});
    raft.InitRaft();
    network_manager.AddListenSocket(raft.GetNormalUdp());
    network_manager.AddListenSocket(raft.GetMulticastUdp());
    raft.Start();

    sleep(1);
    CHECK(RaftInfos::Role::LEADER == raft.GetRole());
    network_manager.Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("Raft_10Nodes_group_size_10")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();
    std::string multicastIP = "234.56.78.90";
    uint16_t multicastPort  = 9987;
    uint8_t num             = 10;
    std::vector<std::shared_ptr<Raft>> rafts;
    std::vector<std::thread> threads;
    for (int i = 0; i < num; i++)
    {
        auto raft_ptr = std::make_shared<Raft>(
            RaftInfos::RaftBaseInfo{std::chrono::milliseconds{100},
                multicastIP,
                "",
                multicastPort,
                num});
        raft_ptr->InitRaft();
        network_manager.AddListenSocket(raft_ptr->GetNormalUdp());
        network_manager.AddListenSocket(raft_ptr->GetMulticastUdp());
        rafts.push_back(raft_ptr);
        threads.push_back(std::thread([raft_ptr = raft_ptr]() {
            raft_ptr->Start();
        }));
    }
    for (int i = 0; i < num; i++)
    {
        threads[i].join();
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
    network_manager.Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("Raft_10Nodes_group_size_1")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();
    std::string multicastIP = "234.56.78.90";
    uint16_t multicastPort  = 9987;
    uint8_t num             = 10;
    std::vector<std::shared_ptr<Raft>> rafts;
    std::vector<std::thread> threads;
    for (int i = 0; i < num; i++)
    {
        auto raft_ptr = std::make_shared<Raft>(RaftInfos::RaftBaseInfo{
            std::chrono::milliseconds{100}, multicastIP, "", multicastPort, 1});
        raft_ptr->InitRaft();
        network_manager.AddListenSocket(raft_ptr->GetNormalUdp());
        network_manager.AddListenSocket(raft_ptr->GetMulticastUdp());
        rafts.push_back(raft_ptr);
        threads.push_back(std::thread([raft_ptr = raft_ptr]() {
            raft_ptr->Start();
        }));
    }
    for (int i = 0; i < num; i++)
    {
        threads[i].join();
    }
    sleep(1);
    int leadernum = 0, fllowernum = 0;
    for (int i = 0; i < num; i++)
    {
        leadernum += rafts[i]->GetRole() == RaftInfos::Role::LEADER ? 1 : 0;
        fllowernum += rafts[i]->GetRole() == RaftInfos::Role::FOLLOWER ? 1 : 0;
    }
    CHECK(1 == leadernum);
    CHECK(num - 1 == fllowernum);
    network_manager.Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("Raft_addNode_in_runtime")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    std::vector<std::shared_ptr<Raft>> rafts;
    network::NetWorkManager<network::SimplePoll> network_manager;
    network_manager.Start();
    std::string multicastIP = "234.56.78.90";
    uint16_t multicastPort  = 9987;
    uint8_t num             = 10;
    for (int i = 0; i < num; i++)
    {
        auto raft_ptr = std::make_shared<Raft>(RaftInfos::RaftBaseInfo{
            std::chrono::milliseconds{100}, multicastIP, "", multicastPort, 1});
        raft_ptr->InitRaft();
        network_manager.AddListenSocket(raft_ptr->GetNormalUdp());
        network_manager.AddListenSocket(raft_ptr->GetMulticastUdp());

        raft_ptr->Start();
        CHECK((i == 0 ? RaftInfos::Role::LEADER : RaftInfos::Role::FOLLOWER)
            == raft_ptr->GetRole());
        rafts.push_back(raft_ptr);
    }
    sleep(1);
    int leadernum = 0, fllowernum = 0;
    for (int i = 0; i < num; i++)
    {
        leadernum += rafts[i]->GetRole() == RaftInfos::Role::LEADER ? 1 : 0;
        fllowernum += rafts[i]->GetRole() == RaftInfos::Role::FOLLOWER ? 1 : 0;
    }
    CHECK(1 == leadernum);
    CHECK(num - 1 == fllowernum);
    network_manager.Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}