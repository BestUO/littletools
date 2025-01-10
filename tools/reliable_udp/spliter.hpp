#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <netinet/in.h>
#include <set>
#include <algorithm>
#include "tools/uuid.hpp"
#include "tools/objectpool.hpp"
#include "rudp_struct.hpp"
namespace RUDP
{
inline namespace v1
{
template <uint8_t MAX_SEGMENT_COUNT   = 8,
    uint16_t MAX_SEGMENT_PAYLOAD_SIZE = 1024>
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
        auto cell_payload_size = __payload_view.size();
        uint16_t cell_segment_count
            = cell_payload_size / MAX_SEGMENT_PAYLOAD_SIZE
            + (cell_payload_size % MAX_SEGMENT_PAYLOAD_SIZE == 0 ? 0 : 1);

        SplitStr(
            message_id, message_size, offset_in_message, cell_segment_count);
    }

    SplitCell(std::vector<std::string>::iterator payload_begin,
        std::vector<std::string>::iterator payload_end,
        UUID message_id,
        uint64_t message_size,
        uint64_t offset_in_message,
        UUID cell_id)
        : __cell_id(cell_id)
        , __payload_begin(payload_begin)
        , __payload_end(payload_end)
    {
        SplitVector(message_id,
            message_size,
            offset_in_message,
            payload_end - payload_begin);
    }

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
    std::vector<std::string>::iterator __payload_begin;
    std::vector<std::string>::iterator __payload_end;

    void SplitStr(UUID message_id,
        uint64_t message_size,
        uint64_t offset_in_message,
        uint16_t cell_segment_count)
    {
        for (uint8_t i = 0; i < cell_segment_count && i < MAX_SEGMENT_COUNT;
             i++)
        {
            auto payload_view = __payload_view.substr(
                i * MAX_SEGMENT_PAYLOAD_SIZE, MAX_SEGMENT_PAYLOAD_SIZE);
            __messages.emplace_back(MessageInfo{ReliableUDPType::CellSend,
                message_id,
                message_size,
                offset_in_message,
                CellInfo{CellInfoHeader{
                             __cell_id,
                             i,
                             (uint16_t)(i * MAX_SEGMENT_PAYLOAD_SIZE),
                             cell_segment_count,
                         },
                    payload_view}});
        }
    }

    void SplitVector(UUID message_id,
        uint64_t message_size,
        uint64_t offset_in_message,
        uint16_t cell_segment_count)
    {
        for (uint8_t i = 0; i < cell_segment_count && i < MAX_SEGMENT_COUNT;
             i++)
        {
            __messages.emplace_back(MessageInfo{ReliableUDPType::VectorCellSend,
                message_id,
                message_size,
                offset_in_message,
                CellInfo{CellInfoHeader{
                             __cell_id,
                             i,
                             0,
                             cell_segment_count,
                         },
                    *(__payload_begin + i)}});
        }
    }
};

template <uint8_t MAX_SEGMENT_COUNT   = 8,
    uint16_t MAX_SEGMENT_PAYLOAD_SIZE = 1024>
class MessageSpliter
{
    using SplitCellType
        = SplitCell<MAX_SEGMENT_COUNT, MAX_SEGMENT_PAYLOAD_SIZE>;

public:
    MessageSpliter(std::string&& payload, UUID message_id, sockaddr_in addr)
        : __payload(std::move(payload))
        , __message_id(message_id)
        , __addr(addr)
    {
        uint32_t cell_count
            = (__payload.size() + MAX_SEGMENT_PAYLOAD_SIZE * MAX_SEGMENT_COUNT
                  - 1)
            / (MAX_SEGMENT_PAYLOAD_SIZE * MAX_SEGMENT_COUNT);
        SplitStr(cell_count);
    }

    MessageSpliter(MessageSpliter&& other)
        : __payload(std::move(other.__payload))
        , __payloads(std::move(other.__payloads))
        , __message_id(other.message_id)
        , __addr(other.addr)
    { }

    MessageSpliter(std::vector<std::string>&& payload,
        UUID message_id,
        sockaddr_in addr)
        : __payloads(std::move(payload))
        , __message_id(message_id)
        , __addr(addr)
    {
        uint32_t cell_count
            = (__payloads.size() + MAX_SEGMENT_COUNT - 1) / MAX_SEGMENT_COUNT;
        SplitVector(cell_count);
    }

    ~MessageSpliter()
    {
        for (auto& cell : __cell_ptrs)
            ObjectPool<SplitCellType>::GetInstance()->PutObject(cell.second);
    }

    void Remove(UUID cell_id)
    {
        __loss_cells.erase(cell_id);
    }

    void DealWithSplitCell(std::function<void(MessageInfo&)> f,
        std::function<void(UUID, UUID, sockaddr_in)> timeout_fun)
    {
        for (auto& [cell_id, cell_ptr] : __cell_ptrs)
        {
            if (timeout_fun)
                timeout_fun(__message_id, cell_id, __addr);
            cell_ptr->DealWithMessage(f);
        }
    }

    void Resend(UUID cell_id,
        const std::vector<uint8_t>& loss_index,
        std::function<void(MessageInfo&)> f)
    {
        auto iter = __cell_ptrs.find(cell_id);
        if (iter != __cell_ptrs.end())
            iter->second->Resend(loss_index, f);
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
        auto iter = __cell_ptrs.find(cell_id);
        if (iter != __cell_ptrs.end())
            return iter->second->GetCellSize();
        else
            return 0;
    }

    uint32_t GetPayloadSize()
    {
        return __payload.size();
    }

    bool IsFinished()
    {
        return __is_finished.exchange(true);
    }

    ////////// only for test //////////
public:
    size_t GetSplitCellSizeForTest()
    {
        return __loss_cells.size();
    }

    UUID GetMessageID()
    {
        return __message_id;
    }
    ///////////////////////////////////

private:
    std::string __payload;
    std::unordered_map<UUID, SplitCellType*> __cell_ptrs;
    std::set<UUID> __loss_cells;
    UUID __message_id;
    sockaddr_in __addr;
    std::atomic_bool __is_finished{false};
    std::vector<std::string> __payloads;

    void SplitStr(uint32_t cell_count)
    {
        for (uint32_t i = 0; i < cell_count; i++)
        {
            auto seg_size = std::min(
                (uint32_t)(MAX_SEGMENT_PAYLOAD_SIZE * MAX_SEGMENT_COUNT),
                (uint32_t)(__payload.size()
                    - i * (MAX_SEGMENT_PAYLOAD_SIZE * MAX_SEGMENT_COUNT)));
            auto cell_id = UUID::gen();

            __cell_ptrs.emplace(cell_id,
                ObjectPool<SplitCellType>::GetInstance()->GetObject(
                    std::string_view{__payload.c_str()
                            + i
                                * (MAX_SEGMENT_PAYLOAD_SIZE
                                    * MAX_SEGMENT_COUNT),
                        seg_size},
                    __message_id,
                    __payload.size(),
                    i * (MAX_SEGMENT_PAYLOAD_SIZE * MAX_SEGMENT_COUNT),
                    cell_id));
            __loss_cells.emplace(cell_id);
        }
    }

    void SplitVector(uint32_t cell_count)
    {
        auto iter = __payloads.begin();
        for (uint32_t i = 0; i < cell_count - 1; i++)
        {
            auto cell_id = UUID::gen();
            iter += i;
            __cell_ptrs.emplace(cell_id,
                ObjectPool<SplitCellType>::GetInstance()->GetObject(
                    iter, iter + MAX_SEGMENT_COUNT, __message_id, cell_id));
            __loss_cells.emplace(cell_id);
        }
        auto cell_id = UUID::gen();
        __cell_ptrs.emplace(cell_id,
            ObjectPool<SplitCellType>::GetInstance()->GetObject(
                iter, __payloads.end(), __message_id, cell_id));
        __loss_cells.emplace(cell_id);
    }
};
}  // namespace v1

namespace v2
{
template <uint8_t MAX_SEGMENT_COUNT   = 8,
    uint16_t MAX_SEGMENT_PAYLOAD_SIZE = 1024>
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
        auto cell_payload_size = __payload_view.size();
        uint16_t cell_segment_count
            = cell_payload_size / MAX_SEGMENT_PAYLOAD_SIZE
            + (cell_payload_size % MAX_SEGMENT_PAYLOAD_SIZE == 0 ? 0 : 1);

        SplitStr(
            message_id, message_size, offset_in_message, cell_segment_count);
    }

    SplitCell(SplitCell&& other)
        : __payload_view(other.__payload_view)
        , __cell_id(other.__cell_id)
        , __payload_begin(other.__payload_begin)
        , __payload_end(other.__payload_end)
    { }

    SplitCell(std::vector<std::string>::iterator payload_begin,
        std::vector<std::string>::iterator payload_end,
        UUID message_id,
        uint64_t message_size,
        uint64_t offset_in_message,
        UUID cell_id)
        : __cell_id(cell_id)
        , __payload_begin(payload_begin)
        , __payload_end(payload_end)
    {
        SplitVector(message_id,
            message_size,
            offset_in_message,
            payload_end - payload_begin);
    }

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
    std::vector<std::string>::iterator __payload_begin;
    std::vector<std::string>::iterator __payload_end;

    void SplitStr(UUID message_id,
        uint64_t message_size,
        uint64_t offset_in_message,
        uint16_t cell_segment_count)
    {
        for (uint8_t i = 0; i < cell_segment_count && i < MAX_SEGMENT_COUNT;
             i++)
        {
            auto payload_view = __payload_view.substr(
                i * MAX_SEGMENT_PAYLOAD_SIZE, MAX_SEGMENT_PAYLOAD_SIZE);
            __messages.emplace_back(MessageInfo{ReliableUDPType::CellSend,
                message_id,
                message_size,
                offset_in_message,
                CellInfo{CellInfoHeader{
                             __cell_id,
                             i,
                             (uint16_t)(i * MAX_SEGMENT_PAYLOAD_SIZE),
                             cell_segment_count,
                         },
                    payload_view}});
        }
    }

    void SplitVector(UUID message_id,
        uint64_t message_size,
        uint64_t offset_in_message,
        uint16_t cell_segment_count)
    {
        for (uint8_t i = 0; i < cell_segment_count && i < MAX_SEGMENT_COUNT;
             i++)
        {
            __messages.emplace_back(MessageInfo{ReliableUDPType::VectorCellSend,
                message_id,
                message_size,
                offset_in_message,
                CellInfo{CellInfoHeader{
                             __cell_id,
                             i,
                             0,
                             cell_segment_count,
                         },
                    *(__payload_begin + i)}});
        }
    }
};

template <uint8_t MAX_SEGMENT_COUNT   = 8,
    uint16_t MAX_SEGMENT_PAYLOAD_SIZE = 1024>
class MessageSpliter
{
    using SplitCellType
        = SplitCell<MAX_SEGMENT_COUNT, MAX_SEGMENT_PAYLOAD_SIZE>;

public:
    MessageSpliter(std::string&& payload, UUID message_id, sockaddr_in addr)
        : __payload(std::move(payload))
        , __message_id(message_id)
        , __addr(addr)
    {
        uint32_t cell_count
            = (__payload.size() + MAX_SEGMENT_PAYLOAD_SIZE * MAX_SEGMENT_COUNT
                  - 1)
            / (MAX_SEGMENT_PAYLOAD_SIZE * MAX_SEGMENT_COUNT);
        SplitStr(cell_count);
    }

    MessageSpliter(MessageSpliter&& other)
        : __payload(std::move(other.__payload))
        , __payloads(std::move(other.__payloads))
        , __message_id(other.__message_id)
        , __addr(other.__addr)
    { }

    MessageSpliter(std::vector<std::string>&& payload,
        UUID message_id,
        sockaddr_in addr)
        : __payloads(std::move(payload))
        , __message_id(message_id)
        , __addr(addr)
    {
        uint32_t cell_count
            = (__payloads.size() + MAX_SEGMENT_COUNT - 1) / MAX_SEGMENT_COUNT;
        SplitVector(cell_count);
    }

    ~MessageSpliter() = default;

    void Remove(UUID cell_id)
    {
        __loss_cells.erase(cell_id);
    }

    void DealWithSplitCell(std::function<void(MessageInfo&)> f,
        std::function<void(UUID, UUID, sockaddr_in)> timeout_fun)
    {
        for (auto& [cell_id, cell_ptr] : __cells)
        {
            if (timeout_fun)
                timeout_fun(__message_id, cell_id, __addr);
            cell_ptr.DealWithMessage(f);
        }
    }

    void Resend(UUID cell_id,
        const std::vector<uint8_t>& loss_index,
        std::function<void(MessageInfo&)> f)
    {
        auto iter = __cells.find(cell_id);
        if (iter != __cells.end())
            iter->second.Resend(loss_index, f);
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
            return iter->second->GetCellSize();
        else
            return 0;
    }

    uint32_t GetPayloadSize()
    {
        return __payload.size();
    }

    bool IsFinished()
    {
        return __is_finished.exchange(true);
    }

    ////////// only for test //////////
public:
    size_t GetSplitCellSizeForTest()
    {
        return __loss_cells.size();
    }

    UUID GetMessageID()
    {
        return __message_id;
    }
    ///////////////////////////////////

private:
    std::string __payload;
    std::unordered_map<UUID, SplitCellType> __cells;
    std::set<UUID> __loss_cells;
    UUID __message_id;
    sockaddr_in __addr;
    std::atomic_bool __is_finished{false};
    std::vector<std::string> __payloads;

    void SplitStr(uint32_t cell_count)
    {
        for (uint32_t i = 0; i < cell_count; i++)
        {
            auto seg_size = std::min(
                (uint32_t)(MAX_SEGMENT_PAYLOAD_SIZE * MAX_SEGMENT_COUNT),
                (uint32_t)(__payload.size()
                    - i * (MAX_SEGMENT_PAYLOAD_SIZE * MAX_SEGMENT_COUNT)));
            auto cell_id = UUID::gen();

            __cells.emplace(cell_id,
                SplitCellType(std::string_view{__payload.c_str()
                                      + i
                                          * (MAX_SEGMENT_PAYLOAD_SIZE
                                              * MAX_SEGMENT_COUNT),
                                  seg_size},
                    __message_id,
                    __payload.size(),
                    i * (MAX_SEGMENT_PAYLOAD_SIZE * MAX_SEGMENT_COUNT),
                    cell_id));
            __loss_cells.emplace(cell_id);
        }
    }

    void SplitVector(uint32_t cell_count)
    {
        auto iter = __payloads.begin();
        for (uint32_t i = 0; i < cell_count - 1; i++)
        {
            auto cell_id = UUID::gen();
            iter += i;
            __cells.emplace(cell_id,
                SplitCellType{iter,
                    iter + MAX_SEGMENT_COUNT,
                    __message_id,
                    __payloads.size(),
                    i,
                    cell_id});
            __loss_cells.emplace(cell_id);
        }
        auto cell_id = UUID::gen();
        __cells.emplace(cell_id,
            SplitCellType{iter,
                __payloads.end(),
                __message_id,
                __payloads.size(),
                cell_count - 1,
                cell_id});
        __loss_cells.emplace(cell_id);
    }
};
}  // namespace v2

}  // namespace RUDP