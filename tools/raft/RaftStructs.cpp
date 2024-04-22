#include <string.h>
#include "RaftStructs.h"
#include "../template.hpp"
#include "../simple_serialize.hpp"

constexpr char RaftCommandType::CommonInfo::version[15];
RaftCommandType::CommonInfo::CommonInfo(MessageType m)
    : messageType(m)
{ }

RaftCommandType::CommonInfo::CommonInfo(const char* buf, uint16_t size)
{
    deserialize(buf, size);
}

std::string RaftCommandType::CommonInfo::serialize()
{
    std::string context;
    writeBuffer(messageType, sizeof(messageType), context);
    std::string ver(version, sizeof(version));
    writeBuffer(ver, ver.size(), context);
    writeBuffer(term, sizeof(term), context);
    writeBuffer(uuid, sizeof(uuid), context);
    writeBuffer(timestamp, sizeof(timestamp), context);

    return context;
}

void RaftCommandType::CommonInfo::deserialize(const char* buf, uint16_t size)
{
    (void)size;
    uint16_t offset = 0;
    readBuffer(buf, offset, messageType);
    std::string ver;
    readBuffer(buf, offset, ver);
    readBuffer(buf, offset, term);
    readBuffer(buf, offset, uuid);
    readBuffer(buf, offset, timestamp);
}

RaftCommandType::Vote::Vote(const char* buf, uint16_t size)
{
    uint16_t offset = 0;
    readBuffer(buf, offset, commonInfo);
}

std::string RaftCommandType::Vote::serialize()
{
    std::string context;
    writeBuffer(commonInfo, 0, context);
    return context;
}

RaftCommandType::VoteResponse::VoteResponse(const char* buf, uint16_t size)
{
    uint16_t offset = 0;
    readBuffer(buf, offset, commonInfo);
    readBuffer(buf, offset, voteGranted);
}

std::string RaftCommandType::VoteResponse::serialize()
{
    std::string context;
    writeBuffer(commonInfo, 0, context);
    writeBuffer(voteGranted, sizeof(voteGranted), context);
    return context;
}

RaftCommandType::HeartBeat::HeartBeat(const char* buf, uint16_t size)
{
    uint16_t offset = 0;
    readBuffer(buf, offset, commonInfo);
}

std::string RaftCommandType::HeartBeat::serialize()
{
    std::string context;
    writeBuffer(commonInfo, 0, context);
    return context;
}

RaftCommandType::VoteResult::VoteResult(const char* buf, uint16_t size)
{
    uint16_t offset = 0;
    readBuffer(buf, offset, commonInfo);
}

std::string RaftCommandType::VoteResult::serialize()
{
    std::string context;
    writeBuffer(commonInfo, 0, context);
    return context;
}