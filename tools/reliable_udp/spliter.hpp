#pragma once
#include <cstdint>
#include <functional>
#include <netinet/in.h>
#include <set>
#include <algorithm>
#include "tools/uuid.hpp"
#include "rudp_struct.hpp"

template <uint8_t MAX_SPLIT_COUNT = 8, uint16_t MAX_PAYLOAD_SIZE = 1024>
class SplitCell
{
public:
    SplitCell(std::string_view payload_view,
        UUID message_id,
        uint64_t message_size,
        uint64_t offset_in_message,
        UUID cell_id)
        : __payload_view(payload_view)
        , __cell_id(cell_id)
    {
        auto cell_size            = __payload_view.size();
        uint32_t cell_total_count = cell_size / MAX_PAYLOAD_SIZE
            + (cell_size % MAX_PAYLOAD_SIZE == 0 ? 0 : 1);

        Split(message_id, message_size, offset_in_message, cell_total_count);
    }

    SplitCell(SplitCell&& split_cell)
        : __payload_view(split_cell.__payload_view)
        , __cell_id(split_cell.__cell_id)
        , __messages(std::move(split_cell.__messages))
    { }

    void DealWithMessage(std::function<void(MessageInfo&)> f)
    {
        for (auto& message : __messages)
            f(message);
    }

    void Resend(const std::vector<uint8_t>& loss_index,
        std::function<void(MessageInfo&)> f)
    {
        for (auto& i : loss_index)
        {
            if (i < __messages.size())
                f(__messages[i]);
        }
    }

    uint32_t GetCellSize()
    {
        return __payload_view.size();
    }

    ////////// only for test //////////
public:
    UUID GetMessageID()
    {
        return __cell_id;
    }

    std::vector<MessageInfo> GetMessagesForTest()
    {
        return __messages;
    }
    ///////////////////////////////////

private:
    std::string_view __payload_view;
    UUID __cell_id;
    std::vector<MessageInfo> __messages;

    void Split(UUID message_id,
        uint64_t message_size,
        uint64_t offset_in_message,
        uint32_t cell_total_count)
    {
        for (uint32_t i = 0; i < cell_total_count && i < MAX_SPLIT_COUNT; i++)
        {
            auto payload_view
                = __payload_view.substr(i * MAX_PAYLOAD_SIZE, MAX_PAYLOAD_SIZE);
            __messages.emplace_back(MessageInfo{ReliableUDPType::CellSend,
                message_id,
                message_size,
                offset_in_message,
                CellInfo{CellInfoHeader{__cell_id,
                             i,
                             i * MAX_PAYLOAD_SIZE,
                             cell_total_count,
                             (uint32_t)payload_view.size()},
                    payload_view}});
        }
    }
};

template <uint8_t MAX_SPLIT_COUNT = 8, uint16_t MAX_PAYLOAD_SIZE = 1024>
class MessageSpliter
{
    using SplitCellType = SplitCell<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE>;

public:
    MessageSpliter(std::string&& payload, UUID message_id, sockaddr_in addr)
        : __payload(std::move(payload))
        , __message_id(message_id)
        , __addr(addr)
    {
        __total_count = __payload.size() / (MAX_PAYLOAD_SIZE * MAX_SPLIT_COUNT)
            + (__payload.size() % (MAX_PAYLOAD_SIZE * MAX_SPLIT_COUNT) == 0
                    ? 0
                    : 1);
        Split();
    }

    MessageSpliter(MessageSpliter&& message_split)
        : __payload(std::move(message_split.__payload))
        , __total_count(message_split.__total_count)
        , __message_id(message_split.__message_id)
        , __addr(message_split.__addr)
    {
        message_split.__cells.clear();
        message_split.__loss_cells.clear();
        Split();
    }

    void Remove(UUID cell_id)
    {
        __loss_cells.erase(cell_id);
    }

    void DealWithSplitCell(std::function<void(MessageInfo&)> f,
        std::function<void(UUID, UUID, sockaddr_in)> timeout_fun)
    {
        for (auto& [cell_id, cell] : __cells)
        {
            cell.DealWithMessage(f);
            if (timeout_fun)
                timeout_fun(__message_id, cell_id, __addr);
        }
    }

    void Resend(UUID cell_id,
        const std::vector<uint8_t>& loss_index,
        std::function<void(MessageInfo&)> f)
    {
        auto iter = __cells.find(cell_id);
        if (iter != __cells.end())
        {
            iter->second.Resend(loss_index, f);
        }
    }

    bool IsAllRecved()
    {
        return __loss_cells.size() == 0;
    }

    sockaddr_in GetAddr()
    {
        return __addr;
    }

    uint32_t GetCellSize(UUID cell_id)
    {
        auto iter = __cells.find(cell_id);
        if (iter != __cells.end())
            return iter->second.GetCellSize();
        else
            return 0;
    }

    uint32_t GetLeftMsgSize()
    {
        return __payload.size();
    }

    ////////// only for test //////////
public:
    void AddForTest(SplitCellType&& split_cell)
    {
        auto emplace_result
            = __cells.emplace(split_cell.GetMessageID(), std::move(split_cell));
        __loss_cells.emplace(split_cell.GetMessageID());
    }
    size_t GetSplitCellSizeForTest()
    {
        return __loss_cells.size();
    }
    ///////////////////////////////////

private:
    std::string __payload;
    uint32_t __total_count;
    std::unordered_map<UUID, SplitCellType> __cells;
    std::set<UUID> __loss_cells;
    UUID __message_id;
    sockaddr_in __addr;

    void Split()
    {
        for (uint32_t i = 0; i < __total_count; i++)
        {
            auto seg_size
                = std::min((uint32_t)(MAX_PAYLOAD_SIZE * MAX_SPLIT_COUNT),
                    (uint32_t)(__payload.size()
                        - i * (MAX_PAYLOAD_SIZE * MAX_SPLIT_COUNT)));
            auto cell_id = UUID::gen();
            __cells.emplace(cell_id,
                SplitCellType(
                    {__payload.c_str()
                            + i * (MAX_PAYLOAD_SIZE * MAX_SPLIT_COUNT),
                        seg_size},
                    __message_id,
                    __payload.size(),
                    i * (MAX_PAYLOAD_SIZE * MAX_SPLIT_COUNT),
                    cell_id));
            __loss_cells.emplace(cell_id);
        }
    }
};