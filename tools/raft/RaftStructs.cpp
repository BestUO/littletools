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

RaftCommandType::Vote::Vote(const char* buf, uint16_t size)
{
    deserialize(buf);
}

RaftCommandType::VoteResponse::VoteResponse(const char* buf, uint16_t size)
{
    deserialize(buf);
}

RaftCommandType::HeartBeat::HeartBeat(const char* buf, uint16_t size)
{
    deserialize(buf);
}

RaftCommandType::VoteResult::VoteResult(const char* buf, uint16_t size)
{
    deserialize(buf);
}