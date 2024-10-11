#pragma once

#include <cstdint>
#include <functional>
#include <shared_mutex>
#include <unordered_map>
#include <array>
#include "tools/network/simple_epoll.hpp"
#include "tools/network/network_manager.hpp"
#include "tools/network/network_global.hpp"
#include "tools/uuid.hpp"
#include "tools/network/udp.hpp"
#include "tools/reliable_udp/assembler.hpp"
#include "tools/timermanager.hpp"
#include "rudp_struct.hpp"
#include "spliter.hpp"
#include "assembler.hpp"

#define TIMEOUT 1000

template <uint8_t MAX_SEGMENT_COUNT   = 8,
    uint16_t MAX_SEGMENT_PAYLOAD_SIZE = 1024,
    uint32_t BAND_WIDTH               = 1024 * 1024 * 50>
struct RUDPLargeMsgSend
{
    constexpr static uint8_t SPLIT_COUNT = MAX_SEGMENT_COUNT > 8 ? 16 : 8;

public:
    RUDPLargeMsgSend(const std::string& ip,
        uint16_t port,
        std::shared_ptr<FlowControl> flow_control)
        : __endpoint(std::make_shared<network::inet_udp::UDP<>>())
        , __flow_control(flow_control)
    {
        __endpoint->SetAddr(ip.c_str(), port);
        __endpoint->SetCallBack(std::bind(&RUDPLargeMsgSend::Recv,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3));
        network::NetWorkManager<network::SimpleEpoll>::GetInstance()
            ->AddListenSocket(__endpoint);
    };
    ~RUDPLargeMsgSend()
    {
        network::NetWorkManager<network::SimpleEpoll>::GetInstance()
            ->RemoveListenSocket(__endpoint);
    }

    bool Send(std::string_view message, const sockaddr_in& addr)
    {
        if (message.size() <= MAX_SEGMENT_PAYLOAD_SIZE * SPLIT_COUNT * 1024)
        {
            UUID msg_id = UUID::gen();
            typename std::unordered_map<UUID, MAP_VALUE_TYPE>::iterator iter;
            MessageSpliter<SPLIT_COUNT, MAX_SEGMENT_PAYLOAD_SIZE>* spliter_ptr
                = ObjectPool<MessageSpliter<SPLIT_COUNT,
                    MAX_SEGMENT_PAYLOAD_SIZE>>::GetInstance()
                      ->GetObject(message.data(), msg_id, addr);
            {
                std::unique_lock<std::shared_mutex> lock(__mutex);
                iter = __message_spliters.emplace(msg_id, spliter_ptr).first;
            }
            spliter_ptr->DealWithSplitCell(
                [this, addr](MessageInfo& message_info) {
                    std::string msg(message_info.CalculateSize(), '\0');
                    message_info.serialize(msg.data());
                    if (__endpoint->Send(msg, addr) == network::Result::SUCCESS)
                        __flow_control->Wait(msg.size());
                },
                [this](UUID message_id, UUID cell_id, sockaddr_in addr) {
                    timermanager::TimerManager<UUID>::GetInstance()->AddAlarm(
                        std::chrono::milliseconds(TIMEOUT),
                        cell_id,
                        std::string(),
                        [this, message_id, cell_id, addr]() {
                            __endpoint->Send(
                                CellTimeoutCheck{
                                    ReliableUDPType::CellTimeoutCheck,
                                    message_id,
                                    cell_id}
                                    .serialize(),
                                addr);
                        },
                        std::chrono::milliseconds(TIMEOUT));
                });

            if (spliter_ptr->IsFinished())
                EraseSpliter(iter);

            return true;
        }
        else
            return false;
    };

public:  // only for test
    bool SendLost(std::string_view message,
        const sockaddr_in& addr,
        uint32_t lost_index)
    {
        // if (message.size() <= MAX_SEGMENT_PAYLOAD_SIZE * SPLIT_COUNT * 1024)
        // {
        //     UUID msg_id = UUID::gen();
        //     auto iter   = __message_spliters.emplace(msg_id,
        //         MessageSpliter<SPLIT_COUNT, MAX_SEGMENT_PAYLOAD_SIZE>(
        //             message.data(), msg_id, addr));
        //     iter.first->second.DealWithSplitCell(
        //         [this, addr, lost_index](MessageInfo& message_info) {
        //             auto msg = std::move(message_info.serialize());
        //             if (message_info.cell_info.cell_header.cell_current_index
        //                     != lost_index
        //                 && __endpoint->Send(message_info.serialize(), addr)
        //                     == network::Result::SUCCESS)
        //                 __flow_control->Wait(msg.size());
        //         },
        //         [this](UUID message_id, UUID cell_id, sockaddr_in addr) {
        //             timermanager::TimerManager<UUID>::GetInstance()->AddAlarm(
        //                 std::chrono::milliseconds(TIMEOUT),
        //                 cell_id,
        //                 std::string(),
        //                 [this, message_id, cell_id, addr]() {
        //                     __endpoint->Send(
        //                         CellTimeoutCheck{
        //                             ReliableUDPType::CellTimeoutCheck,
        //                             message_id,
        //                             cell_id}
        //                             .serialize(),
        //                         addr);
        //                 },
        //                 std::chrono::milliseconds(TIMEOUT));
        //         });
        //     return true;
        // }
        // else
        return false;
    };

private:
    using MAP_VALUE_TYPE
        = MessageSpliter<SPLIT_COUNT, MAX_SEGMENT_PAYLOAD_SIZE>*;
    std::shared_ptr<network::inet_udp::UDP<>> __endpoint;
    std::shared_ptr<FlowControl> __flow_control;
    std::unordered_map<UUID, MAP_VALUE_TYPE> __message_spliters;
    std::shared_mutex __mutex;

    void EraseSpliter(std::unordered_map<UUID, MAP_VALUE_TYPE>::iterator iter)
    {

        // __flow_control->Increase(iter->second->GetPayloadSize());
        ObjectPool<MessageSpliter<SPLIT_COUNT,
            MAX_SEGMENT_PAYLOAD_SIZE>>::GetInstance()
            ->PutObject(iter->second);
        std::unique_lock<std::shared_mutex> lock(__mutex);
        __message_spliters.erase(iter);
    }

    std::string Recv(const char* data, size_t size, const sockaddr&)
    {
        ReliableUDPType message_type = *(ReliableUDPType*)data;
        if (message_type == ReliableUDPType::CellReceived)
        {
            // UUID* message_id = (UUID*)(data + 1);
            // auto amessage_id = EndianSwap<>::swap(*message_id);
            // return MessageFinished{
            //     ReliableUDPType::MessageFinished, amessage_id}
            //     .serialize();
            // UUID message_id = EndianSwap<>::swap(*(UUID*)(data + 1));
            // UUID cell_id    = EndianSwap<>::swap(*(UUID*)(data + 17));
            CellReceived cell_received(data);
            typename std::unordered_map<UUID, MAP_VALUE_TYPE>::iterator iter;
            {
                std::shared_lock<std::shared_mutex> lock(__mutex);
                iter = __message_spliters.find(cell_received.message_id);
            }
            if (iter != __message_spliters.end())
            {
                timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
                    cell_received.cell_id, std::string());
                iter->second->Remove(cell_received.cell_id);
                if (!iter->second->IsAllRecved())
                    return std::string();
                else
                {
                    if (iter->second->IsFinished())
                        EraseSpliter(iter);
                    return MessageFinished{ReliableUDPType::MessageFinished,
                        cell_received.message_id}
                        .serialize();
                }
            }
            else
                return MessageFinished{
                    ReliableUDPType::Abnormal, cell_received.message_id}
                    .serialize();
        }
        else if (message_type == ReliableUDPType::CellTimeoutResponse)
        {
            CellTimeoutResponse cell_timeout_response(data);
            typename std::unordered_map<UUID, MAP_VALUE_TYPE>::iterator iter;
            {
                std::shared_lock<std::shared_mutex> lock(__mutex);
                iter
                    = __message_spliters.find(cell_timeout_response.message_id);
            }
            if (iter != __message_spliters.end())
            {
                iter->second->Resend(cell_timeout_response.cell_id,
                    cell_timeout_response.cell_loss_index,
                    [this, addr = iter->second->GetAddr()](
                        MessageInfo& message_info) {
                        std::string msg(message_info.CalculateSize(), '\0');
                        message_info.serialize(msg.data());
                        __endpoint->Send(msg, addr);
                    });
                return std::string();
            }
            else
                return MessageFinished{
                    ReliableUDPType::Abnormal, cell_timeout_response.message_id}
                    .serialize();
        }
        else if (message_type == ReliableUDPType::Abnormal)
        {
            MessageFinished abnormal(data);
            return std::string();
        }
        else
            return std::string();
    }
};