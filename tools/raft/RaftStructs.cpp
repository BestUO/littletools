#include "RaftStructs.h"
#include "service_registry/Templates.h"
namespace acfw
{
namespace cm
{
constexpr char RaftCommandType::CommonInfo::constname[15];
RaftCommandType::CommonInfo::CommonInfo(MessageType m)
    : messageType(m)
{ }

RaftCommandType::CommonInfo::CommonInfo(const char* buf, uint16_t size)
{
    uint16_t len;
    uint16_t offset = 0;

    memcpy(&len, buf + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy(&messageType, buf + offset, len);
    offset += len;

    memcpy(&len, buf + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    offset += len;

    memcpy(&len, buf + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy(&term, buf + offset, len);
    offset += len;

    memcpy(&len, buf + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy(&uuid, buf + offset, len);
    offset += len;

    memcpy(&len, buf + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy(&timestamp, buf + offset, len);
    offset += len;

    messageType = MessageType(EndianSwap<>::swap((uint16_t)messageType));
    term        = EndianSwap<>::swap(term);
    uuid        = {EndianSwap<>::swap(uuid.a), EndianSwap<>::swap(uuid.b)};
    timestamp   = EndianSwap<>::swap(timestamp);
    if (size != offset)
    { }
}

std::string RaftCommandType::CommonInfo::serialize()
{
    std::string context;
    uint16_t len;

    auto messageTypeBigEndian = EndianSwap<>::swap((uint16_t)messageType);
    auto termBigEndian        = EndianSwap<>::swap(term);
    auto uuidBigEndian
        = Id{EndianSwap<>::swap(uuid.a), EndianSwap<>::swap(uuid.b)};
    auto timestampBigEndian = EndianSwap<>::swap(timestamp);

    len = sizeof(messageTypeBigEndian);
    context.append((char*)&len, sizeof(uint16_t));
    context.append((char*)&messageTypeBigEndian, sizeof(messageTypeBigEndian));

    len = sizeof(constname);
    context.append((char*)&len, sizeof(uint16_t));
    context.append(constname, len);

    len = sizeof(termBigEndian);
    context.append((char*)&len, sizeof(uint16_t));
    context.append((char*)&termBigEndian, sizeof(termBigEndian));

    len = sizeof(uuidBigEndian);
    context.append((char*)&len, sizeof(uint16_t));
    context.append((char*)&uuidBigEndian, sizeof(uuidBigEndian));

    len = sizeof(timestampBigEndian);
    context.append((char*)&len, sizeof(uint16_t));
    context.append((char*)&timestampBigEndian, len);

    return context;
}

RaftCommandType::Vote::Vote(const char* buf, uint16_t size)
{
    uint16_t len;
    uint16_t offset = 0;

    memcpy(&len, buf + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    commonInfo = CommonInfo(buf + offset, len);
    offset += len;

    if (size != offset)
    { }
}

std::string RaftCommandType::Vote::serialize()
{
    std::string context, structcontext;
    uint16_t len;

    structcontext = commonInfo.serialize();
    len           = structcontext.size();
    context.append((char*)&len, sizeof(uint16_t));
    context.append(structcontext.data(), len);

    return context;
}

RaftCommandType::VoteResponse::VoteResponse(const char* buf, uint16_t size)
{
    uint16_t len;
    uint16_t offset = 0;

    memcpy(&len, buf + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    commonInfo = CommonInfo(buf + offset, len);
    offset += len;

    memcpy(&len, buf + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy(&voteGranted, buf + offset, len);
    offset += len;

    voteGranted = EndianSwap<>::swap(voteGranted);
    if (size != offset)
    { }
}

std::string RaftCommandType::VoteResponse::serialize()
{
    std::string context, structcontext;
    uint16_t len;
    auto voteGrantedBigEndian = EndianSwap<>::swap(voteGranted);

    structcontext = commonInfo.serialize();
    len           = structcontext.size();
    context.append((char*)&len, sizeof(uint16_t));
    context.append(structcontext.data(), len);

    len = sizeof(voteGrantedBigEndian);
    context.append((char*)&len, sizeof(uint16_t));
    context.append((char*)&voteGrantedBigEndian, sizeof(voteGrantedBigEndian));

    return context;
}

RaftCommandType::HeartBeat::HeartBeat(const char* buf, uint16_t size)
{
    uint16_t len;
    uint16_t offset = 0;

    memcpy(&len, buf + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    commonInfo = CommonInfo(buf + offset, len);
    offset += len;

    if (size != offset)
    { }
}

std::string RaftCommandType::HeartBeat::serialize()
{
    std::string context, structcontext;
    uint16_t len;

    structcontext = commonInfo.serialize();
    len           = structcontext.size();
    context.append((char*)&len, sizeof(uint16_t));
    context.append(structcontext.data(), len);

    return context;
}

RaftCommandType::VoteResult::VoteResult(const char* buf, uint16_t size)
{
    uint16_t len;
    uint16_t offset = 0;

    memcpy(&len, buf + offset, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    commonInfo = CommonInfo(buf + offset, len);
    offset += len;

    if (size != offset)
    { }
}

std::string RaftCommandType::VoteResult::serialize()
{
    std::string context, structcontext;
    uint16_t len;

    structcontext = commonInfo.serialize();
    len           = structcontext.size();
    context.append((char*)&len, sizeof(uint16_t));
    context.append(structcontext.data(), len);

    return context;
}
}  // namespace cm
}  // namespace acfw