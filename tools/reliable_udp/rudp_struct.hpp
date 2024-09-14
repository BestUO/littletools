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
    MessageSendOK,
    CellTimeoutCheck,
    CellTimeoutResponse,
    Abnormal,
};

struct CellInfoHeader
{
    UUID cell_id;
    uint32_t cell_current_index;
    uint64_t cell_offset;
    uint32_t cell_total_count;
    uint32_t cell_payload_size;

    std::string serialize()
    {
        std::string context;
        writeBuffer(cell_id, sizeof(cell_id), context);
        writeBuffer(cell_current_index, sizeof(cell_current_index), context);
        writeBuffer(cell_offset, sizeof(cell_offset), context);
        writeBuffer(cell_total_count, sizeof(cell_total_count), context);
        writeBuffer(cell_payload_size, sizeof(cell_payload_size), context);

        return context;
    }

    void deserialize(const char* buf, uint16_t size)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, cell_id);
        readBuffer(buf, offset, cell_current_index);
        readBuffer(buf, offset, cell_offset);
        readBuffer(buf, offset, cell_total_count);
        readBuffer(buf, offset, cell_payload_size);
    }
};

struct CellInfo
{
    CellInfoHeader cell_header;
    std::string_view cell_payload;

    std::string serialize()
    {
        std::string context;
        writeBuffer(cell_header, sizeof(cell_header), context);
        writeBuffer(cell_payload, sizeof(cell_payload), context);

        return context;
    }

    char* deserialize(const char* buf, uint16_t size)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, cell_header);
        return const_cast<char*>(buf) + offset + 2;
    }
};

struct MessageInfo
{
    ReliableUDPType type = ReliableUDPType::CellSend;
    UUID message_id;
    uint64_t message_size;
    uint64_t message_offset;
    CellInfo cell_info;

    std::string serialize()
    {
        std::string context;
        writeBuffer(type, sizeof(type), context);
        writeBuffer(message_id, sizeof(message_id), context);
        writeBuffer(message_size, sizeof(message_size), context);
        writeBuffer(message_offset, sizeof(message_offset), context);
        writeBuffer(cell_info, sizeof(cell_info), context);

        return context;
    }

    char* deserialize(const char* buf, uint16_t size)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
        readBuffer(buf, offset, message_size);
        readBuffer(buf, offset, message_offset);
        // readBuffer(buf, offset, cell_info);
        return cell_info.deserialize(buf + offset + 2, 0);
    }
};

struct CellReceived
{
    ReliableUDPType type = ReliableUDPType::CellReceived;
    UUID message_id;
    UUID cell_id;

    std::string serialize()
    {
        std::string context;
        writeBuffer(type, sizeof(type), context);
        writeBuffer(message_id, sizeof(message_id), context);
        writeBuffer(cell_id, sizeof(cell_id), context);

        return context;
    }

    void deserialize(const char* buf, uint16_t size)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
        readBuffer(buf, offset, cell_id);
    }
};

struct MessageSendOK
{
    ReliableUDPType type = ReliableUDPType::MessageSendOK;
    UUID message_id;
    std::string serialize()
    {
        std::string context;
        writeBuffer(type, sizeof(type), context);
        writeBuffer(message_id, sizeof(message_id), context);

        return context;
    }

    void deserialize(const char* buf, uint16_t size)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
    }
};

struct CellTimeoutCheck
{
    ReliableUDPType type = ReliableUDPType::CellTimeoutCheck;
    UUID message_id;
    UUID cell_id;
    std::string serialize()
    {
        std::string context;
        writeBuffer(type, sizeof(type), context);
        writeBuffer(message_id, sizeof(message_id), context);
        writeBuffer(cell_id, sizeof(cell_id), context);

        return context;
    }

    void deserialize(const char* buf, uint16_t size)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
        readBuffer(buf, offset, cell_id);
    }
};

struct CellTimeoutResponse
{
    ReliableUDPType type = ReliableUDPType::CellTimeoutResponse;
    UUID message_id;
    UUID cell_id;
    std::vector<uint8_t> cell_loss_index;
    std::string serialize()
    {
        std::string context;
        writeBuffer(type, sizeof(type), context);
        writeBuffer(message_id, sizeof(message_id), context);
        writeBuffer(cell_id, sizeof(cell_id), context);
        writeBuffer(cell_loss_index, cell_loss_index.size(), context);

        return context;
    }

    void deserialize(const char* buf, uint16_t size)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
        readBuffer(buf, offset, cell_id);
        readBuffer(buf, offset, cell_loss_index);
    }
};

struct Abnormal
{
    ReliableUDPType type = ReliableUDPType::Abnormal;
    UUID message_id;
    std::string serialize()
    {
        std::string context;
        writeBuffer(type, sizeof(type), context);
        writeBuffer(message_id, sizeof(message_id), context);

        return context;
    }

    void deserialize(const char* buf, uint16_t size)
    {
        uint16_t offset = 0;
        readBuffer(buf, offset, type);
        readBuffer(buf, offset, message_id);
    }
};