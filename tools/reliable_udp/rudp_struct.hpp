#pragma once
#include "tools/uuid.hpp"
#include "tools/simple_serialize.hpp"
#include <cstdint>
#include <string>
#include <vector>

enum class ReliableUDPType : uint8_t
{
    CellSend,
    CellReceived,
    MessageFinished,
    MessageFinishedACK,
    CellTimeoutCheck,
    CellTimeoutResponse,
    Abnormal,
};

struct CellInfoHeader
{
    UUID cell_id;
    uint8_t cell_current_index;
    uint16_t cell_offset;
    uint16_t cell_segment_count;
    GEN_SERIALIZE(cell_id, cell_current_index, cell_offset, cell_segment_count)
};

struct CellInfo
{
    CellInfoHeader cell_header;
    std::string cell_payload;
    GEN_SERIALIZE(cell_header, cell_payload)
};

struct MessageInfo
{
    ReliableUDPType type = ReliableUDPType::CellSend;
    UUID message_id;
    uint64_t message_size;
    uint64_t message_offset;
    CellInfo cell_info;
    GEN_SERIALIZE(type, message_id, message_size, message_offset, cell_info)
};

struct CellReceived
{
    ReliableUDPType type = ReliableUDPType::CellReceived;
    UUID message_id;
    UUID cell_id;
    CellReceived(const char* data)
    {
        deserialize(data);
    }
    CellReceived(ReliableUDPType type,
        const UUID& message_id,
        const UUID& cell_id)
        : type(type)
        , message_id(message_id)
        , cell_id(cell_id)
    { }
    GEN_SERIALIZE(type, message_id, cell_id)
};

struct MessageFinished
{
    ReliableUDPType type = ReliableUDPType::MessageFinished;
    UUID message_id;
    MessageFinished(const char* data)
    {
        deserialize(data);
    }
    MessageFinished(ReliableUDPType type, const UUID& message_id)
        : type(type)
        , message_id(message_id)
    { }
    GEN_SERIALIZE(type, message_id)
};

struct CellTimeoutCheck
{
    ReliableUDPType type = ReliableUDPType::CellTimeoutCheck;
    UUID message_id;
    UUID cell_id;
    CellTimeoutCheck(const char* data)
    {
        deserialize(data);
    }
    CellTimeoutCheck(ReliableUDPType type,
        const UUID& message_id,
        const UUID& cell_id)
        : type(type)
        , message_id(message_id)
        , cell_id(cell_id)
    { }
    GEN_SERIALIZE(type, message_id, cell_id)
};

struct CellTimeoutResponse
{
    ReliableUDPType type = ReliableUDPType::CellTimeoutResponse;
    UUID message_id;
    UUID cell_id;
    std::vector<uint8_t> cell_loss_index;
    CellTimeoutResponse(const char* data)
    {
        deserialize(data);
    }
    CellTimeoutResponse(ReliableUDPType type,
        const UUID& message_id,
        const UUID& cell_id,
        std::vector<uint8_t>&& cell_loss_index)
        : type(type)
        , message_id(message_id)
        , cell_id(cell_id)
        , cell_loss_index(std::move(cell_loss_index))
    { }
    GEN_SERIALIZE(type, message_id, cell_id, cell_loss_index)
};