#include "Raft.h"
#include <chrono>
#include <mutex>
#include "../simplepoll/simplepoll.hpp"
#include "../timermanager.hpp"
#include "../template.hpp"

Raft::Raft(const RaftInfos::RaftBaseInfo& baseinfo)
    : m_infos(baseinfo)
{ }

Raft::~Raft()
{
    stop();
}

void Raft::setBaseInfo(const RaftInfos::RaftBaseInfo& baseinfo)
{
    m_infos = baseinfo;
}

void Raft::setSendSocket(int32_t bindsocket)
{
    m_sendSocket = bindsocket;
}

void Raft::start()
{
    checkHeartBeat();
    while (m_infos.leader_uuid == UUID{0, 0})
    {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }
}

void Raft::stop()
{
    timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
        m_infos.self_uuid);
}

std::string Raft::handleIncomingData(const char* buf,
    uint16_t len,
    const std::string& ip,
    uint16_t port)
{
    if (len < RaftCommandType::CommonInfo::size())
    {
        return "";
    }
    else
    {
        if (strcmp(buf + 8, RAFT_VERSION))
        {
            logOut((void*)this,
                ": handleIncomingData: serializename ",
                std::string(buf + 8, 15),
                " != ",
                RAFT_VERSION);
            return "";
        }
        else
        {
            (void)ip;
            (void)port;
            std::string responsedata;
            RaftCommandType::MessageType type
                = *(RaftCommandType::MessageType*)(buf + 4);
            type = RaftCommandType::MessageType(
                EndianSwap<>::swap((uint16_t)type));

            if (type == RaftCommandType::MessageType::HEARTBEAT)
            {
                responsedata
                    = handlerHeartBeat(RaftCommandType::HeartBeat(buf, len));
            }
            else if (type == RaftCommandType::MessageType::VOTE)
            {
                responsedata = handlerVote(RaftCommandType::Vote(buf, len));
            }
            else if (type == RaftCommandType::MessageType::VOTERESPONSE)
            {
                responsedata = handlerVoteResponse(
                    RaftCommandType::VoteResponse(buf, len));
            }
            return responsedata;
        }
    }
}

std::chrono::milliseconds Raft::getRandTimeout() const
{
    auto timeout = std::rand() % 100;
    return std::chrono::milliseconds(timeout);
}

void Raft::checkHeartBeat()
{
    timermanager::TimerManager<UUID>::GetInstance()->AddAlarm(
        m_infos.base_info.heartbeat_interval * 3 + getRandTimeout(),
        m_infos.self_uuid,
        "checkHeartBeat",
        [this]() {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            if (std::chrono::steady_clock::now()
                    - m_infos.last_heartbeat_timepoint
                < m_infos.base_info.heartbeat_interval * 3)
            { }
            else
            {
                logOut(m_infos.leader_uuid.toString(), " timeout");
                Vote();
            }
        },
        m_infos.base_info.heartbeat_interval * 3);
}

std::string Raft::votePrepare()
{
    if (m_infos.role == RaftInfos::Role::LEADER)
    {
        timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
            m_infos.self_uuid, "sendHeartBeat");
    }
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    m_infos.term++;
    m_infos.role        = RaftInfos::Role::CANDICATE;
    m_infos.leader_uuid = {0, 0};
    m_voteAgreeCount    = 0;

    RaftCommandType::Vote vote;
    vote.commonInfo.term = m_infos.term;
    vote.commonInfo.uuid = m_infos.self_uuid;

    logOut(UUID(vote.commonInfo.uuid).toString(),
        " term is ",
        vote.commonInfo.term,
        " send vote");

    return vote.serialize();
}

void Raft::Vote()
{
    std::string vote = votePrepare();
    SimplePoll<>::sendData(vote.data(),
        vote.size(),
        m_infos.base_info.control_multicast_ip,
        m_infos.base_info.control_multicast_port,
        m_sendSocket);
}

std::string Raft::handlerVote(const RaftCommandType::Vote& cmd)
{
    logOut(m_infos.self_uuid.toString(),
        " handle remote vote ",
        UUID(cmd.commonInfo.uuid).toString(),
        " local term is ",
        m_infos.term,
        " remote term is ",
        cmd.commonInfo.term);
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

    RaftCommandType::VoteResponse response;
    if (cmd.commonInfo.uuid == m_infos.self_uuid)
    {
        response.voteGranted = true;
    }
    else
    {
        if (cmd.commonInfo.term <= m_infos.term)
        {
            response.voteGranted = false;
        }
        else
        {
            if (m_infos.role == RaftInfos::Role::LEADER)
            {
                timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
                    m_infos.self_uuid, "sendHeartBeat");
            }
            m_infos.leader_uuid  = {0, 0};
            m_infos.term         = cmd.commonInfo.term;
            m_infos.role         = RaftInfos::Role::FOLLOWER;
            response.voteGranted = true;
        }
    }
    response.commonInfo.term = m_infos.term;
    response.commonInfo.uuid = m_infos.self_uuid;

    return response.serialize();
}

std::string Raft::handlerVoteResponse(const RaftCommandType::VoteResponse& cmd)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    logOut(m_infos.self_uuid.toString(),
        " recv voteresponse from ",
        UUID(cmd.commonInfo.uuid).toString(),
        " vote result is ",
        cmd.voteGranted,
        " voteAgreeCount ",
        (int)m_voteAgreeCount,
        " local term ",
        m_infos.term,
        " remote term ",
        cmd.commonInfo.term);
    if (cmd.commonInfo.term > m_infos.term)
    {
        m_infos.role     = RaftInfos::Role::FOLLOWER;
        m_infos.term     = cmd.commonInfo.term;
        m_voteAgreeCount = 0;
    }
    else if (cmd.commonInfo.term == m_infos.term)
    {
        if (cmd.voteGranted && m_infos.role == RaftInfos::Role::CANDICATE)
        {
            m_voteAgreeCount++;
            if (m_voteAgreeCount > m_infos.base_info.cluster_size / 2)
            {
                m_infos.role        = RaftInfos::Role::LEADER;
                m_infos.leader_uuid = m_infos.self_uuid;
                m_voteAgreeCount    = 0;
                timermanager::TimerManager<UUID>::GetInstance()->AddAlarm(
                    std::chrono::milliseconds{0},
                    m_infos.self_uuid,
                    "sendHeartBeat",
                    [this]() {
                        sendHeartBeat();
                    },
                    m_infos.base_info.heartbeat_interval);
                logOut("pick leader ", m_infos.leader_uuid.toString());
            }
        }
    }

    return "";
}

void Raft::sendHeartBeat()
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    RaftCommandType::HeartBeat heartbeat;
    heartbeat.commonInfo.term = m_infos.term;
    heartbeat.commonInfo.uuid = m_infos.self_uuid;
    auto data                 = heartbeat.serialize();
    SimplePoll<>::sendData(data.data(),
        data.size(),
        m_infos.base_info.control_multicast_ip,
        m_infos.base_info.control_multicast_port,
        m_sendSocket);
}

std::string Raft::handlerHeartBeat(const RaftCommandType::HeartBeat& cmd)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    // logOut(m_infos.self_uuid.toString(),
    //     " recv heartbeat from ",
    //     Uuid(cmd.commonInfo.uuid).toString(),
    //     " local term ",
    //     m_infos.term,
    //     " remote term ",
    //     cmd.commonInfo.term);

    if (cmd.commonInfo.term > m_infos.term)
    {
        timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
            m_infos.self_uuid, "sendHeartBeat");
        m_infos.role                     = RaftInfos::Role::FOLLOWER;
        m_infos.term                     = cmd.commonInfo.term;
        m_infos.leader_uuid              = cmd.commonInfo.uuid;
        m_infos.last_heartbeat_timepoint = std::chrono::steady_clock::now();
    }
    else if (cmd.commonInfo.term == m_infos.term)
    {
        if (m_infos.role != RaftInfos::Role::LEADER)
        {
            m_infos.role                     = RaftInfos::Role::FOLLOWER;
            m_infos.leader_uuid              = cmd.commonInfo.uuid;
            m_infos.last_heartbeat_timepoint = std::chrono::steady_clock::now();
        }
        else
        {
            if (m_infos.leader_uuid == cmd.commonInfo.uuid)
            { }
            else if (m_infos.leader_uuid < cmd.commonInfo.uuid)
            {
                timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
                    m_infos.self_uuid, "sendHeartBeat");
                m_infos.role        = RaftInfos::Role::FOLLOWER;
                m_infos.leader_uuid = cmd.commonInfo.uuid;
            }
            else
            { }
            m_infos.last_heartbeat_timepoint = std::chrono::steady_clock::now();
        }
    }
    return "";
}

RaftInfos::Role Raft::getRole()
{
    return m_infos.role;
}