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
    std::string handleIncomingData(const char* buf,
        uint16_t len,
        const std::string& ip,
        uint16_t port);
    RaftInfos::Role getRole();
    void start();
    void setSendSocket(int32_t socket);

private:
    RaftInfos m_infos;
    std::recursive_mutex m_mutex;
    uint8_t m_voteAgreeCount = 0;
    int32_t m_sendSocket     = -1;

    std::chrono::milliseconds getRandTimeout() const;
    std::string handlerHeartBeat(const RaftCommandType::HeartBeat& cmd);
    std::string handlerVote(const RaftCommandType::Vote& cmd);
    std::string handlerVoteResponse(const RaftCommandType::VoteResponse& cmd);
    void sendHeartBeat();
    void checkHeartBeat();
    void Vote();
    std::string votePrepare();
};