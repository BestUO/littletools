#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <array>
#include "tools/network/simple_epoll.hpp"
#include "tools/network/network_manager.hpp"
#include "tools/uuid.hpp"
#include "tools/network/udp.hpp"
#include "tools/reliable_udp/assembler.hpp"
#include "tools/timermanager.hpp"
#include "tools/threadpool.hpp"
#include "rudp_allocate.hpp"
#include "rudp_struct.hpp"
#include "spliter.hpp"
#include "assembler.hpp"

#define TIMEOUT 1000

template <uint8_t MAX_SPLIT_COUNT = 8,
    uint16_t MAX_PAYLOAD_SIZE     = 1024,
    uint32_t BAND_WIDTH           = 1024 * 1024 * 50>
struct RUDPLargeMsgRecv
{
    constexpr static uint8_t SPLIT_COUNT = MAX_SPLIT_COUNT > 8 ? 16 : 8;
    constexpr static std::array<uint8_t, 16> All_Loss_index
        = MAX_SPLIT_COUNT > 8
        ? std::array<uint8_t,
            16>{0, 1, 2, 3, 4, 5, 6, 7, 255, 255, 255, 255, 255, 255, 255, 255}
        : std::array<uint8_t,
            16>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    using RUDPAllocate = RudpAllocate<1440>;

public:
    RUDPLargeMsgRecv(const std::string& ip,
        uint16_t port,
        std::function<void(std::unique_ptr<char[]>)> cb)
        : __endpoint(
            std::make_shared<network::inet_udp::UDP<RudpAllocate<1440>>>())
        , __cb(cb)
    {
        __thread_pool.Start([this](RudpAllocate<1440>::RudpAllocateTag*& msg) {
            Working(msg);
        });
        __endpoint->SetAddr(ip.c_str(), port);
        __endpoint->SetCallBack(std::bind(&RUDPLargeMsgRecv::Recv,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3));
        network::NetWorkManager<network::SimpleEpoll>::GetInstance()
            ->AddListenSocket(__endpoint);
    };
    ~RUDPLargeMsgRecv()
    {
        __thread_pool.Stop();
        network::NetWorkManager<network::SimpleEpoll>::GetInstance()
            ->RemoveListenSocket(__endpoint);
    }

private:
    std::shared_ptr<network::inet_udp::UDP<RudpAllocate<1440>>> __endpoint;
    std::function<void(std::unique_ptr<char[]>)> __cb;
    std::shared_mutex __rw_mutex;
    std::unordered_map<UUID, MessageAssembler<SPLIT_COUNT>>
        __message_assemblers;
    threadpool::v4::ThreadPoll<RudpAllocate<1440>::RudpAllocateTag*>
        __thread_pool;
    int __index = 0;

    void EraseAssembler(UUID message_id)
    {
        __message_assemblers.erase(message_id);
    }

    void Working(RudpAllocate<1440>::RudpAllocateTag*& msg)
    {
        auto data                    = msg->__buf;
        ReliableUDPType message_type = *(ReliableUDPType*)data;
        if (message_type == ReliableUDPType::CellSend)
        {
            // auto message_id = EndianSwap<>::swap(*(UUID*)(data + 1));
            // auto cell_id    = EndianSwap<>::swap(*(UUID*)(data + 33));
            // auto aaa        = CellReceived(
            //     ReliableUDPType::CellReceived, message_id, cell_id)
            //                .serialize();
            // CellReceived cell_received(aaa.data());
            // __endpoint->Send(
            //     CellReceived{ReliableUDPType::CellReceived, message_id,
            //     cell_id}
            //         .serialize(),
            //     *reinterpret_cast<sockaddr_in*>(&msg.addr));

            MessageInfo message_info;
            auto offset = message_info.deserialize(data);
            typename std::unordered_map<UUID,
                MessageAssembler<SPLIT_COUNT>>::iterator iter;
            {
                std::shared_lock<std::shared_mutex> lock(__rw_mutex);
                iter = __message_assemblers.find(message_info.message_id);
            }

            if (iter == __message_assemblers.end())
            {
                std::unique_lock<std::shared_mutex> lock(__rw_mutex);
                iter = __message_assemblers
                           .emplace(message_info.message_id,
                               MessageAssembler<SPLIT_COUNT>(
                                   message_info.message_size,
                                   message_info.message_id))
                           .first;
            }

            auto len = EndianSwap<>::swap(*(uint16_t*)(data + offset));
            auto is_cell_finish = iter->second.DealWithCellMessage(
                message_info.cell_info.cell_header,
                message_info.message_offset,
                data + offset + sizeof(uint16_t),
                len);
            if (is_cell_finish)
            {
                __endpoint->Send(CellReceived{ReliableUDPType::CellReceived,
                                     message_info.message_id,
                                     message_info.cell_info.cell_header.cell_id}
                                     .serialize(),
                    *reinterpret_cast<sockaddr_in*>(
                        const_cast<sockaddr*>(&msg->addr)));
            }
        }
        else if (message_type == ReliableUDPType::MessageFinished)
        {
            std::unique_ptr<char[]> payload = nullptr;
            MessageFinished send_ok(data);
            {
                std::unique_lock<std::shared_mutex> lock(__rw_mutex);
                auto iter = __message_assemblers.find(send_ok.message_id);
                if (iter != __message_assemblers.end())
                {
                    payload = iter->second.GetPalyload();
                    __message_assemblers.erase(iter);
                }
            }
            if (payload)
                __cb(std::move(payload));
        }
        else if (message_type == ReliableUDPType::CellTimeoutCheck)
        {
            CellTimeoutCheck cell_timeout_check(data);
            typename std::unordered_map<UUID,
                MessageAssembler<SPLIT_COUNT>>::iterator iter;
            {
                std::shared_lock<std::shared_mutex> lock(__rw_mutex);
                iter = __message_assemblers.find(cell_timeout_check.message_id);
            }
            if (iter != __message_assemblers.end())
                __endpoint->Send(
                    CellTimeoutResponse{ReliableUDPType::CellTimeoutResponse,
                        cell_timeout_check.message_id,
                        cell_timeout_check.cell_id,
                        std::move(ArrayToVector(iter->second.DealWithTimeout(
                            cell_timeout_check.cell_id)))}
                        .serialize(),
                    *reinterpret_cast<sockaddr_in*>(
                        const_cast<sockaddr*>(&msg->addr)));
            else
            {
                __endpoint->Send(
                    CellTimeoutResponse{ReliableUDPType::CellTimeoutResponse,
                        cell_timeout_check.message_id,
                        cell_timeout_check.cell_id,
                        ArrayToVector(All_Loss_index)}
                        .serialize(),
                    *reinterpret_cast<sockaddr_in*>(
                        const_cast<sockaddr*>(&msg->addr)));
            }
        }
        else if (message_type == ReliableUDPType::Abnormal)
        { }
        __endpoint->FreeBuf(const_cast<char*>(data));
    }

    std::string Recv(const char* data, size_t size, const sockaddr& addr)
    {
        ((RudpAllocate<1440>::RudpAllocateTag*)data)->addr = addr;
        auto message_id = EndianSwap<>::swap(*(UUID*)(data + 1));
        __thread_pool.Put(
            message_id.hash(), (RudpAllocate<1440>::RudpAllocateTag*)data);
        return std::string();
    }

    template <size_t num>
    std::vector<uint8_t> ArrayToVector(const std::array<uint8_t, num>& arr)
    {
        std::vector<uint8_t> vec;
        for (size_t i = 0; i < SPLIT_COUNT; i++)
        {
            if (arr[i] == 255)
                break;
            vec.push_back(i);
        }
        return vec;
    }
};