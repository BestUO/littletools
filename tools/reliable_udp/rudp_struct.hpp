#pragma once
#include "tools/uuid.hpp"
#include "tools/simple_serialize.hpp"
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

enum class ReliableUDPType : uint8_t
{
    CellSend,
    CellReceived,
    MessageFinished,
    CellTimeoutCheck,
    CellTimeoutResponse,
    Abnormal,
};

struct CellInfoHeader
{
    UUID cell_id;
    uint8_t cell_current_index;
    uint16_t cell_offset;
    uint16_t cell_total_count;

    std::string serialize() const
    {
        std::string context(CalculateSize(), '\0');
        serialize(context.data());
        return context;
    }

    char* serialize(char* buf) const
    {
        buf = writeBuffer(cell_id, buf);
        buf = writeBuffer(cell_current_index, buf);
        buf = writeBuffer(cell_offset, buf);
        buf = writeBuffer(cell_total_count, buf);

        return buf;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, cell_id);
        readBuffer(buf, offset, cell_current_index);
        readBuffer(buf, offset, cell_offset);
        readBuffer(buf, offset, cell_total_count);
        return offset;
    }

    uint16_t CalculateSize() const
    {
        return ::CalculateSize(cell_id) + ::CalculateSize(cell_current_index)
            + ::CalculateSize(cell_offset) + ::CalculateSize(cell_total_count);
    }
};

struct CellInfo
{
    CellInfoHeader cell_header;
    std::string_view cell_payload;

    std::string serialize() const
    {
        std::string context(CalculateSize(), '\0');
        serialize(context.data());
        return context;
    }

    char* serialize(char* buf) const
    {
        buf = writeBuffer(cell_header, buf);
        buf = writeBuffer(cell_payload, buf);

        return buf;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, cell_header);
        return offset;
    }

    uint16_t CalculateSize() const
    {
        return ::CalculateSize(cell_header) + ::CalculateSize(cell_payload);
    }
};

struct MessageInfo
{
    ReliableUDPType type = ReliableUDPType::CellSend;
    UUID message_id;
    uint64_t message_size;
    uint64_t message_offset;
    CellInfo cell_info;

    std::string serialize() const
    {
        std::string context(CalculateSize(), '\0');
        serialize(context.data());
        return context;
    }

    char* serialize(char* buf) const
    {
        buf = writeBuffer(type, buf);
        buf = writeBuffer(message_id, buf);
        buf = writeBuffer(message_size, buf);
        buf = writeBuffer(message_offset, buf);
        buf = writeBuffer(cell_info, buf);

        return buf;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
        readBuffer(buf, offset, message_size);
        readBuffer(buf, offset, message_offset);
        // readBuffer(buf, offset, cell_info);
        return offset + cell_info.deserialize(buf + offset);
    }

    uint16_t CalculateSize() const
    {
        return ::CalculateSize(type) + ::CalculateSize(message_id)
            + ::CalculateSize(message_size) + ::CalculateSize(message_offset)
            + ::CalculateSize(cell_info);
    }
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
    // GEN_SERIALIZE(type, message_id, cell_id)
    std::string serialize() const
    {
        std::string context(CalculateSize(), '\0');
        serialize(context.data());
        return context;
    }

    char* serialize(char* buf) const
    {
        buf = writeBuffer(type, buf);
        buf = writeBuffer(message_id, buf);
        buf = writeBuffer(cell_id, buf);

        return buf;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
        readBuffer(buf, offset, cell_id);
        return offset;
    }

    uint16_t CalculateSize() const
    {
        return ::CalculateSize(type) + ::CalculateSize(message_id)
            + ::CalculateSize(cell_id);
    }
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
    // GEN_SERIALIZE(type, message_id)
    std::string serialize() const
    {
        std::string context(CalculateSize(), '\0');
        serialize(context.data());
        return context;
    }

    char* serialize(char* buf) const
    {
        buf = writeBuffer(type, buf);
        buf = writeBuffer(message_id, buf);

        return buf;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
        return offset;
    }

    uint16_t CalculateSize() const
    {
        return ::CalculateSize(type) + ::CalculateSize(message_id);
    }
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
    // GEN_SERIALIZE(type, message_id, cell_id)
    std::string serialize() const
    {
        std::string context(CalculateSize(), '\0');
        serialize(context.data());
        return context;
    }

    char* serialize(char* buf) const
    {
        buf = writeBuffer(type, buf);
        buf = writeBuffer(message_id, buf);
        buf = writeBuffer(cell_id, buf);

        return buf;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
        readBuffer(buf, offset, cell_id);
        return offset;
    }

    uint16_t CalculateSize() const
    {
        return ::CalculateSize(type) + ::CalculateSize(message_id)
            + ::CalculateSize(cell_id);
    }
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
    // GEN_SERIALIZE(type, message_id, cell_id, cell_loss_index)
    std::string serialize() const
    {
        std::string context(CalculateSize(), '\0');
        serialize(context.data());
        return context;
    }

    char* serialize(char* buf) const
    {
        buf = writeBuffer(type, buf);
        buf = writeBuffer(message_id, buf);
        buf = writeBuffer(cell_id, buf);
        buf = writeBuffer(cell_loss_index, buf);
        return buf;
    }

    uint16_t deserialize(const char* buf)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
        readBuffer(buf, offset, cell_id);
        readBuffer(buf, offset, cell_loss_index);
        return offset;
    }

    uint16_t CalculateSize() const
    {
        return ::CalculateSize(type) + ::CalculateSize(message_id)
            + ::CalculateSize(cell_id) + ::CalculateSize(cell_loss_index);
    }
};