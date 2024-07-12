#pragma once

#include <chrono>
#include <mutex>
#include "RaftStructs.h"
#include "../simplelog.hpp"

class Raft : public SimpleLog
{
public:
    Raft() = default;
    Raft(const RaftInfos::RaftBaseInfo& baseinfo);
    ~Raft();
    std::string HandleIncomingData(const char* buf,
        uint16_t len,
        const std::string& ip,
        uint16_t port);
    RaftInfos::Role GetRole();
    void Start();
    void Stop();
    void SetSendSocket(int32_t socket);
    void SetBaseInfo(const RaftInfos::RaftBaseInfo& baseinfo);

private:
    RaftInfos __infos;
    std::recursive_mutex __mutex;
    uint8_t __voteAgreeCount = 0;
    int32_t __sendSocket     = -1;

    std::chrono::milliseconds GetRandTimeout() const;
    std::string HandlerHeartBeat(const RaftCommandType::HeartBeat& cmd);
    std::string HandlerVote(const RaftCommandType::Vote& cmd);
    std::string handlerVoteResponse(const RaftCommandType::VoteResponse& cmd);
    void SendHeartBeat();
    void CheckHeartBeat();
    void Vote();
    std::string VotePrepare();
};