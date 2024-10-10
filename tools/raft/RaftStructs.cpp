#include <string.h>
#include "RaftStructs.h"
#include "tools/simple_serialize.hpp"

RaftCommandType::CommonInfo::CommonInfo(MessageType m)
    : messageType(m)
{ }

RaftCommandType::CommonInfo::CommonInfo(const char* buf, uint16_t size)
{
    deserialize(buf);
}

std::string RaftCommandType::CommonInfo::serialize() const
{
    std::string context;
    writeBuffer(messageType, context);
    writeBuffer(version, context);
    writeBuffer(term, context);
    writeBuffer(uuid, context);
    writeBuffer(timestamp, context);

    return context;
}

uint16_t RaftCommandType::CommonInfo::deserialize(const char* buf)
{
    uint16_t offset = 0;
    readBuffer(buf, offset, messageType);
    readBuffer(buf, offset, version);
    readBuffer(buf, offset, term);
    readBuffer(buf, offset, uuid);
    readBuffer(buf, offset, timestamp);
    return offset;
}

RaftCommandType::Vote::Vote(const char* buf, uint16_t size)
{
    uint16_t offset = 0;
    readBuffer(buf, offset, commonInfo);
}

std::string RaftCommandType::Vote::serialize() const
{
    std::string context;
    writeBuffer(commonInfo, context);
    return context;
}

RaftCommandType::VoteResponse::VoteResponse(const char* buf, uint16_t size)
{
    uint16_t offset = 0;
    readBuffer(buf, offset, commonInfo);
    readBuffer(buf, offset, voteGranted);
}

std::string RaftCommandType::VoteResponse::serialize() const
{
    std::string context;
    writeBuffer(commonInfo, context);
    writeBuffer(voteGranted, context);
    return context;
}

RaftCommandType::HeartBeat::HeartBeat(const char* buf, uint16_t size)
{
    uint16_t offset = 0;
    readBuffer(buf, offset, commonInfo);
}

std::string RaftCommandType::HeartBeat::serialize() const
{
    std::string context;
    writeBuffer(commonInfo, context);
    return context;
}

RaftCommandType::VoteResult::VoteResult(const char* buf, uint16_t size)
{
    uint16_t offset = 0;
    readBuffer(buf, offset, commonInfo);
}

std::string RaftCommandType::VoteResult::serialize() const
{
    std::string context;
    writeBuffer(commonInfo, context);
    return context;
}