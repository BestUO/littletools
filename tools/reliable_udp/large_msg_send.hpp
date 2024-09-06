#pragma once

#include <cstdint>
#include <functional>
#include <memory>
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

template <uint8_t MAX_SPLIT_COUNT = 8,
    uint16_t MAX_PAYLOAD_SIZE     = 1024,
    uint32_t BAND_WIDTH           = 1024 * 1024 * 50>
struct RUDPLargeMsgSend
{
    constexpr static uint8_t SPLIT_COUNT = MAX_SPLIT_COUNT > 8 ? 16 : 8;

public:
    RUDPLargeMsgSend(const std::string& ip,
        uint16_t port,
        std::shared_ptr<FlowControl<SPLIT_COUNT * MAX_PAYLOAD_SIZE, BAND_WIDTH>>
            flow_control)
        : __endpoint(std::make_shared<network::inet_udp::UDP>())
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
        if (message.size() <= MAX_PAYLOAD_SIZE * SPLIT_COUNT * 1024)
        {
            UUID msg_id = UUID::gen();
            typename std::unordered_map<UUID,
                MessageSpliter<SPLIT_COUNT, MAX_PAYLOAD_SIZE>>::iterator iter;
            {
                std::lock_guard<std::mutex> lock(__mutex);
                iter = __message_spliters
                           .emplace(msg_id,
                               MessageSpliter<SPLIT_COUNT, MAX_PAYLOAD_SIZE>(
                                   message.data(), msg_id, addr))
                           .first;
            }
            iter->second.DealWithSplitCell(
                [this, addr](MessageInfo& message_info) {
                    auto msg = std::move(message_info.serialize());
                    if (__endpoint->Send(message_info.serialize(), addr)
                        == network::Result::SUCCESS)
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
        if (message.size() <= MAX_PAYLOAD_SIZE * SPLIT_COUNT * 1024)
        {
            UUID msg_id = UUID::gen();
            auto iter   = __message_spliters.emplace(msg_id,
                MessageSpliter<SPLIT_COUNT, MAX_PAYLOAD_SIZE>(
                    message.data(), msg_id, addr));
            iter.first->second.DealWithSplitCell(
                [this, addr, lost_index](MessageInfo& message_info) {
                    auto msg = std::move(message_info.serialize());
                    if (message_info.cell_info.cell_header.cell_current_index
                            != lost_index
                        && __endpoint->Send(message_info.serialize(), addr)
                            == network::Result::SUCCESS)
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
            return true;
        }
        else
            return false;
    };

private:
    std::shared_ptr<network::inet_udp::UDP> __endpoint;
    std::shared_ptr<FlowControl<SPLIT_COUNT * MAX_PAYLOAD_SIZE, BAND_WIDTH>>
        __flow_control;
    std::unordered_map<UUID, MessageSpliter<SPLIT_COUNT, MAX_PAYLOAD_SIZE>>
        __message_spliters;
    std::mutex __mutex;

    std::string AbnormalMsg(UUID message_id)
    {
        return Abnormal{ReliableUDPType::Abnormal, message_id}.serialize();
    }

    void DealAbnormal(UUID message_id)
    {
        std::lock_guard<std::mutex> lock(__mutex);
        if (auto iter = __message_spliters.find(message_id);
            iter != __message_spliters.end())
        {
            __flow_control->Increase(iter->second.GetLeftMsgSize());
            __message_spliters.erase(message_id);
        }
    }

    void DealAbnormal(std::unordered_map<UUID,
        MessageSpliter<SPLIT_COUNT, MAX_PAYLOAD_SIZE>>::iterator iter)
    {
        iter->second.DealWithSplitCell(
            [](const MessageInfo& message_info) {
                timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
                    message_info.cell_info.cell_header.cell_id);
            },
            nullptr);
        std::lock_guard<std::mutex> lock(__mutex);
        __flow_control->Increase(iter->second.GetLeftMsgSize());
        __message_spliters.erase(iter);
    }

    std::string Recv(const char* data, size_t size, const sockaddr&)
    {
        ReliableUDPType message_type = *(ReliableUDPType*)(data + 2);
        if (message_type == ReliableUDPType::CellReceived)
        {
            CellReceived cell_ack;
            cell_ack.deserialize(data, size);
            typename std::unordered_map<UUID,
                MessageSpliter<SPLIT_COUNT, MAX_PAYLOAD_SIZE>>::iterator iter;
            {
                std::lock_guard<std::mutex> lock(__mutex);
                iter = __message_spliters.find(cell_ack.message_id);
            }
            if (iter != __message_spliters.end())
            {
                auto& spliter = iter->second;
                timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
                    cell_ack.cell_id);
                spliter.Remove(cell_ack.cell_id);
                __flow_control->Increase(spliter.GetCellSize(cell_ack.cell_id));
                if (!spliter.IsAllRecved())
                    return std::string();
                else
                {
                    std::lock_guard<std::mutex> lock(__mutex);
                    __message_spliters.erase(iter);
                    return MessageSendOK{
                        ReliableUDPType::MessageSendOK, cell_ack.message_id}
                        .serialize();
                }
            }
            else
                return AbnormalMsg(cell_ack.message_id);
        }
        else if (message_type == ReliableUDPType::CellTimeoutResponse)
        {
            CellTimeoutResponse cell_timeout_response;
            cell_timeout_response.deserialize(data, size);
            typename std::unordered_map<UUID,
                MessageSpliter<SPLIT_COUNT, MAX_PAYLOAD_SIZE>>::iterator iter;
            {
                std::lock_guard<std::mutex> lock(__mutex);
                iter
                    = __message_spliters.find(cell_timeout_response.message_id);
            }
            if (iter != __message_spliters.end())
            {
                auto& spliter = iter->second;
                bool flag     = false;
                spliter.Resend(cell_timeout_response.cell_id,
                    cell_timeout_response.cell_loss_index,
                    [this, &flag, addr = spliter.GetAddr()](
                        MessageInfo& message_info) {
                        flag = true;
                        __endpoint->Send(message_info.serialize(), addr);
                    });
                if (flag)
                    return std::string();
                else
                {
                    __flow_control->Increase(
                        spliter.GetCellSize(cell_timeout_response.cell_id));
                    DealAbnormal(iter);
                    return AbnormalMsg(cell_timeout_response.message_id);
                }
            }
            else
                return AbnormalMsg(cell_timeout_response.message_id);
        }
        else if (message_type == ReliableUDPType::Abnormal)
        {
            Abnormal abnormal;
            DealAbnormal(abnormal.message_id);
            return std::string();
        }
        else
            return std::string();
    }
};