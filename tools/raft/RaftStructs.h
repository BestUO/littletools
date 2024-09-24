#pragma once

#include <chrono>
#include <string>
#include "../uuid.hpp"

#define RAFT_VERSION "RAFT-V2.0\0"

struct RaftInfos
{
    struct RaftBaseInfo
    {
        std::chrono::milliseconds heartbeat_interval
            = std::chrono::milliseconds(500);
        std::string control_multicast_ip      = "234.56.78.90";
        std::string control_multicast_send_if = "";
        uint16_t control_multicast_port       = 9987;
        uint8_t cluster_size                  = 1;
    };
    enum Role
    {
        FOLLOWER,
        CANDICATE,
        LEADER,
    };
    Role role     = Role::FOLLOWER;
    uint32_t term = 0;
    std::chrono::time_point<std::chrono::steady_clock> last_heartbeat_timepoint
        = std::chrono::steady_clock::now();
    UUID self_uuid          = UUID::gen();
    UUID leader_uuid        = {0, 0};
    uint32_t control_socket = -1;
    RaftBaseInfo base_info;

    RaftInfos() = default;
    RaftInfos(const RaftBaseInfo& baseinfo)
        : base_info(baseinfo)
    { }
};

struct RaftCommandType
{
    enum class MessageType : uint16_t
    {
        NONE,
        HEARTBEAT,
        VOTE,
        VOTERESPONSE,
        VOTERESULT,
    };

    struct CommonInfo
    {
        MessageType messageType = MessageType::VOTE;
        uint32_t term;
        UUID uuid;
        char version[15] = RAFT_VERSION;
        uint64_t timestamp
            = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                  .count();

        CommonInfo() = default;
        CommonInfo(MessageType m);
        CommonInfo(const char* buf, uint16_t size);
        std::string serialize() const;
        uint16_t deserialize(const char* buf);

        constexpr static uint16_t size()
        {
            return sizeof(messageType) + sizeof(version) + sizeof(uint16_t) * 2;
        }
    };

    struct HeartBeat
    {
        CommonInfo commonInfo = {MessageType::HEARTBEAT};

        HeartBeat() = default;
        HeartBeat(const char* buf, uint16_t size);
        std::string serialize() const;
    };

    struct Vote
    {
        CommonInfo commonInfo = {MessageType::VOTE};

        Vote() = default;
        Vote(const char* buf, uint16_t size);
        std::string serialize() const;
    };

    struct VoteResponse
    {
        CommonInfo commonInfo = {MessageType::VOTERESPONSE};
        bool voteGranted      = false;

        VoteResponse() = default;
        VoteResponse(const char* buf, uint16_t size);
        std::string serialize() const;
    };

    struct VoteResult
    {
        CommonInfo commonInfo = {MessageType::VOTERESULT};

        VoteResult() = default;
        VoteResult(const char* buf, uint16_t size);
        std::string serialize() const;
    };
};