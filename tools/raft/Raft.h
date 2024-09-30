#pragma once

#include <chrono>
#include <mutex>
#include <memory>
#include "RaftStructs.h"
#include "../simplelog.hpp"
#include "tools/network/udp.hpp"

class Raft : public SimpleLog
{
public:
    Raft(const RaftInfos::RaftBaseInfo& baseinfo);
    ~Raft();
    std::string HandleData(const char* buf, uint16_t len);
    RaftInfos::Role GetRole();
    void Start();
    void Stop();
    std::shared_ptr<network::inet_udp::UDP<>> GetNormalUdp() const;
    std::shared_ptr<network::inet_udp::UDP<>> GetMulticastUdp() const;
    void InitRaft();

private:
    RaftInfos __infos;
    std::recursive_mutex __mutex;
    uint8_t __voteAgreeCount                                  = 0;
    std::shared_ptr<network::inet_udp::UDP<>> __normal_udp    = nullptr;
    std::shared_ptr<network::inet_udp::UDP<>> __multicast_udp = nullptr;
    network::inet_udp::UDP<>::sockaddr_type __multicast_addr;

    std::chrono::milliseconds GetRandTimeout() const;
    std::string HandlerHeartBeat(const RaftCommandType::HeartBeat& cmd);
    std::string HandlerVote(const RaftCommandType::Vote& cmd);
    std::string handlerVoteResponse(const RaftCommandType::VoteResponse& cmd);
    void SendHeartBeat();
    void CheckHeartBeat();
    void Vote();
    std::string VotePrepare();
};