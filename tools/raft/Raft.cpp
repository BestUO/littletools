#include "Raft.h"
#include <chrono>
#include <mutex>
#include "tools/timermanager.hpp"
#include "tools/template.hpp"

Raft::Raft()
{
    InitUdp();
}

Raft::Raft(const RaftInfos::RaftBaseInfo& baseinfo)
    : __infos(baseinfo)
    , __normal_udp(std::make_shared<network::inet_udp::UDP>())
    , __multicast_udp(std::make_shared<network::inet_udp::UDP>())
{
    InitUdp();
}

Raft::~Raft()
{
    Stop();
}

void Raft::SetBaseInfo(const RaftInfos::RaftBaseInfo& baseinfo)
{
    __infos = baseinfo;
    InitUdp();
}

void Raft::InitUdp()
{
    __multicast_addr = network::SocketBase::CreateAddr(
        __infos.base_info.control_multicast_ip.c_str(),
        __infos.base_info.control_multicast_port);
    __normal_udp->SetAddr("0.0.0.0", 0);
    __normal_udp->SetCallBack(
        [this](const char* buf, size_t len) -> std::string {
            return HandleData(buf, len);
        });
    __multicast_udp->SetReuseAddrAndPort();
    __multicast_udp->SetAddr(
        "0.0.0.0", __infos.base_info.control_multicast_port);
    __multicast_udp->SetCallBack(
        [this](const char* buf, size_t len) -> std::string {
            return HandleData(buf, len);
        });

    if (!__infos.base_info.control_multicast_send_if.empty())
    {
        __normal_udp->SetMultiCastSendIf(
            __infos.base_info.control_multicast_send_if.c_str());
        __multicast_udp->AddMultiCast(
            __infos.base_info.control_multicast_ip.c_str(),
            __infos.base_info.control_multicast_send_if.c_str());
    }
    else
    {
        __normal_udp->SetMultiCastSendIf();
        __multicast_udp->AddMultiCast(
            __infos.base_info.control_multicast_ip.c_str());
    }
}

void Raft::Start()
{
    CheckHeartBeat();
    while (__infos.leader_uuid == UUID{0, 0})
    {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }
}

void Raft::Stop()
{
    timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
        __infos.self_uuid);
}

std::string Raft::HandleData(const char* buf, uint16_t len)
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
            std::string responsedata;
            RaftCommandType::MessageType type
                = *(RaftCommandType::MessageType*)(buf + 4);
            type = RaftCommandType::MessageType(
                EndianSwap<>::swap((uint16_t)type));

            if (type == RaftCommandType::MessageType::HEARTBEAT)
            {
                responsedata
                    = HandlerHeartBeat(RaftCommandType::HeartBeat(buf, len));
            }
            else if (type == RaftCommandType::MessageType::VOTE)
            {
                responsedata = HandlerVote(RaftCommandType::Vote(buf, len));
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

std::chrono::milliseconds Raft::GetRandTimeout() const
{
    auto timeout = std::rand() % 100;
    return std::chrono::milliseconds(timeout);
}

void Raft::CheckHeartBeat()
{
    timermanager::TimerManager<UUID>::GetInstance()->AddAlarm(
        __infos.base_info.heartbeat_interval * 3 + GetRandTimeout(),
        __infos.self_uuid,
        "CheckHeartBeat",
        [this]() {
            std::lock_guard<std::recursive_mutex> lock(__mutex);
            if (std::chrono::steady_clock::now()
                    - __infos.last_heartbeat_timepoint
                < __infos.base_info.heartbeat_interval * 3)
            { }
            else
            {
                logOut(__infos.leader_uuid.toString(), " timeout");
                Vote();
            }
        },
        __infos.base_info.heartbeat_interval * 3);
}

std::string Raft::VotePrepare()
{
    if (__infos.role == RaftInfos::Role::LEADER)
    {
        timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
            __infos.self_uuid, "SendHeartBeat");
    }
    std::lock_guard<std::recursive_mutex> lock(__mutex);
    __infos.term++;
    __infos.role        = RaftInfos::Role::CANDICATE;
    __infos.leader_uuid = {0, 0};
    __voteAgreeCount    = 0;

    RaftCommandType::Vote vote;
    vote.commonInfo.term = __infos.term;
    vote.commonInfo.uuid = __infos.self_uuid;

    logOut(UUID(vote.commonInfo.uuid).toString(),
        " term is ",
        vote.commonInfo.term,
        " send vote");

    return vote.serialize();
}

void Raft::Vote()
{
    __normal_udp->Send(VotePrepare(), __multicast_addr);
}

std::string Raft::HandlerVote(const RaftCommandType::Vote& cmd)
{
    logOut(__infos.self_uuid.toString(),
        " handle remote vote ",
        UUID(cmd.commonInfo.uuid).toString(),
        " local term is ",
        __infos.term,
        " remote term is ",
        cmd.commonInfo.term);
    std::lock_guard<std::recursive_mutex> lock(__mutex);

    RaftCommandType::VoteResponse response;
    if (cmd.commonInfo.uuid == __infos.self_uuid)
    {
        response.voteGranted = true;
    }
    else
    {
        if (cmd.commonInfo.term <= __infos.term)
        {
            response.voteGranted = false;
        }
        else
        {
            if (__infos.role == RaftInfos::Role::LEADER)
            {
                timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
                    __infos.self_uuid, "SendHeartBeat");
            }
            __infos.leader_uuid  = {0, 0};
            __infos.term         = cmd.commonInfo.term;
            __infos.role         = RaftInfos::Role::FOLLOWER;
            response.voteGranted = true;
        }
    }
    response.commonInfo.term = __infos.term;
    response.commonInfo.uuid = __infos.self_uuid;

    return response.serialize();
}

std::string Raft::handlerVoteResponse(const RaftCommandType::VoteResponse& cmd)
{
    std::lock_guard<std::recursive_mutex> lock(__mutex);
    logOut(__infos.self_uuid.toString(),
        " recv voteresponse from ",
        UUID(cmd.commonInfo.uuid).toString(),
        " vote result is ",
        cmd.voteGranted,
        " voteAgreeCount ",
        (int)__voteAgreeCount,
        " local term ",
        __infos.term,
        " remote term ",
        cmd.commonInfo.term);
    if (cmd.commonInfo.term > __infos.term)
    {
        __infos.role     = RaftInfos::Role::FOLLOWER;
        __infos.term     = cmd.commonInfo.term;
        __voteAgreeCount = 0;
    }
    else if (cmd.commonInfo.term == __infos.term)
    {
        if (cmd.voteGranted && __infos.role == RaftInfos::Role::CANDICATE)
        {
            __voteAgreeCount++;
            if (__voteAgreeCount > __infos.base_info.cluster_size / 2)
            {
                __infos.role        = RaftInfos::Role::LEADER;
                __infos.leader_uuid = __infos.self_uuid;
                __voteAgreeCount    = 0;
                timermanager::TimerManager<UUID>::GetInstance()->AddAlarm(
                    std::chrono::milliseconds{0},
                    __infos.self_uuid,
                    "SendHeartBeat",
                    [this]() {
                        SendHeartBeat();
                    },
                    __infos.base_info.heartbeat_interval);
                logOut("pick leader ", __infos.leader_uuid.toString());
            }
        }
    }

    return "";
}

void Raft::SendHeartBeat()
{
    std::lock_guard<std::recursive_mutex> lock(__mutex);
    RaftCommandType::HeartBeat heartbeat;
    heartbeat.commonInfo.term = __infos.term;
    heartbeat.commonInfo.uuid = __infos.self_uuid;
    __normal_udp->Send(heartbeat.serialize(), __multicast_addr);
}

std::string Raft::HandlerHeartBeat(const RaftCommandType::HeartBeat& cmd)
{
    std::lock_guard<std::recursive_mutex> lock(__mutex);
    // logOut(__infos.self_uuid.toString(),
    //     " recv heartbeat from ",
    //     Uuid(cmd.commonInfo.uuid).toString(),
    //     " local term ",
    //     __infos.term,
    //     " remote term ",
    //     cmd.commonInfo.term);

    if (cmd.commonInfo.term > __infos.term)
    {
        timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
            __infos.self_uuid, "SendHeartBeat");
        __infos.role                     = RaftInfos::Role::FOLLOWER;
        __infos.term                     = cmd.commonInfo.term;
        __infos.leader_uuid              = cmd.commonInfo.uuid;
        __infos.last_heartbeat_timepoint = std::chrono::steady_clock::now();
    }
    else if (cmd.commonInfo.term == __infos.term)
    {
        if (__infos.role != RaftInfos::Role::LEADER)
        {
            __infos.role                     = RaftInfos::Role::FOLLOWER;
            __infos.leader_uuid              = cmd.commonInfo.uuid;
            __infos.last_heartbeat_timepoint = std::chrono::steady_clock::now();
        }
        else
        {
            if (__infos.leader_uuid == cmd.commonInfo.uuid)
            { }
            else if (__infos.leader_uuid < cmd.commonInfo.uuid)
            {
                timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
                    __infos.self_uuid, "SendHeartBeat");
                __infos.role        = RaftInfos::Role::FOLLOWER;
                __infos.leader_uuid = cmd.commonInfo.uuid;
            }
            else
            { }
            __infos.last_heartbeat_timepoint = std::chrono::steady_clock::now();
        }
    }
    return "";
}

std::shared_ptr<network::inet_udp::UDP> Raft::GetNormalUdp() const
{
    return __normal_udp;
}

std::shared_ptr<network::inet_udp::UDP> Raft::GetMulticastUdp() const
{
    return __multicast_udp;
}

RaftInfos::Role Raft::GetRole()
{
    return __infos.role;
}