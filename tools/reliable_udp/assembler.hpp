#pragma once

#include <sys/types.h>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <functional>
#include <netinet/in.h>
#include "tools/reliable_udp/flow_control.hpp"
#include "tools/uuid.hpp"
#include "rudp_struct.hpp"

template <uint8_t MAX_SEGMENT_COUNT = 8>
class AssemblerCell
{
public:
    using collect_type
        = std::conditional<(MAX_SEGMENT_COUNT > 8), uint16_t, uint8_t>::type;

    static constexpr uint16_t row = MAX_SEGMENT_COUNT > 8 ? 65536 : 256;
    static constexpr uint8_t co   = MAX_SEGMENT_COUNT > 8 ? 16 : 8;

    AssemblerCell(uint32_t cell_segment_count, char* dst)
        : __dst(dst)
    {
        __segment_collect |= __table_total_count[cell_segment_count - 1];
    };

    AssemblerCell(uint32_t cell_segment_count,
        std::vector<std::string>::iterator iter_dst)
        : __iter_dst(iter_dst)
    {
        __segment_collect |= __table_total_count[cell_segment_count - 1];
    };

    bool DealWithCellMessage(const char* src,
        const CellInfoHeader& cell_info_header,
        uint16_t len)
    {
        __segment_collect |= 1 << cell_info_header.cell_current_index;
        memcpy(__dst + cell_info_header.cell_offset, src, len);
        return __segment_collect == std::numeric_limits<collect_type>::max();
    }

    bool DealWithVectorCellMessage(const char* src,
        const CellInfoHeader& cell_info_header,
        uint16_t len)
    {
        __segment_collect |= 1 << cell_info_header.cell_current_index;
        *(__iter_dst + cell_info_header.cell_current_index)
            = std::string(src, len);
        return __segment_collect == std::numeric_limits<collect_type>::max();
    }

    std::array<uint8_t, co> GetLossTable() const
    {
        return __table_loss[__segment_collect];
    }

    std::array<uint8_t, co> GetLossTable(collect_type collect_num) const
    {
        return __table_loss[collect_num];
    }

    ////////// only for test //////////
public:
    void PrintTable()
    {
        for (int i = 0; i < row; i++)
        {
            for (int j = 0; j < co; j++)
                std::cout << (int)__table_loss[i][j] << " ";
            std::cout << std::endl;
        }
        for (int i = 0; i < co; i++)
            std::cout << (int)__table_total_count[i] << " ";
        std::cout << std::endl;
    }

    char* GetPayload()
    {
        return __dst;
    }
    ///////////////////////////////////

private:
    collect_type __segment_collect = 0;
    char* __dst;
    std::vector<std::string>::iterator __iter_dst;
    static constexpr std::array<std::array<uint8_t, co>, row> __table_loss
        = [] {
              std::array<std::array<uint8_t, co>, row> temp{};
              for (uint16_t r = 0; r < row; ++r)
              {
                  uint8_t co_index = 0;
                  for (uint8_t c = 0; c < co; ++c)
                  {
                      if ((r & (1 << c)) == 0)
                      {
                          temp[r][co_index] = c;
                          co_index++;
                      }
                  }
                  for (uint8_t c = co_index; c < co; ++c)
                      temp[r][c] = -1;
              }
              return temp;
          }();
    static constexpr std::array<collect_type, co> __table_total_count = [] {
        std::array<collect_type, co> temp{};
        for (uint8_t c = 0; c < co; ++c)
            temp[c] = std::numeric_limits<collect_type>::max()
                - ((1 << (c + 1)) - 1);
        return temp;
    }();
};

template <uint8_t MAX_SEGMENT_COUNT = 8>
class MessageAssembler
{
public:
    MessageAssembler(uint64_t message_size, UUID message_id)
        : __payload(new char[message_size]())
        , __message_id(message_id){};

    MessageAssembler(std::vector<std::string>&& payloads, UUID message_id)
        : __payloads(std::move(payloads))
        , __message_id(message_id){};

    bool DealWithCellMessage(const CellInfoHeader& cell_info_header,
        uint64_t message_offset,
        const char* src,
        uint16_t len)
    {
        auto iter = __message_map.find(cell_info_header.cell_id);
        if (iter == __message_map.end())
        {
            iter = __message_map
                       .emplace(cell_info_header.cell_id,
                           AssemblerCell<MAX_SEGMENT_COUNT>(
                               cell_info_header.cell_segment_count,
                               __payload.get() + message_offset))
                       .first;
        }

        if (iter->second.DealWithCellMessage(src, cell_info_header, len))
            return true;
        else
            return false;
    }

    bool DealWithVectorCellMessage(const CellInfoHeader& cell_info_header,
        uint64_t message_offset,
        const char* src,
        uint16_t len)
    {
        auto iter = __message_map.find(cell_info_header.cell_id);
        if (iter == __message_map.end())
        {
            iter = __message_map
                       .emplace(cell_info_header.cell_id,
                           AssemblerCell<MAX_SEGMENT_COUNT>(
                               cell_info_header.cell_segment_count,
                               __payloads.begin() + message_offset))
                       .first;
        }

        if (iter->second.DealWithVectorCellMessage(src, cell_info_header, len))
            return true;
        else
            return false;
    }

    std::unique_ptr<char[]> GetPalyload()
    {
        return std::move(__payload);
    }

    std::vector<std::string> GetVectorPalyload()
    {
        return std::move(__payloads);
    }

    std::array<uint8_t, MAX_SEGMENT_COUNT> DealWithTimeout(UUID cell_id)
    {
        if (auto iter = __message_map.find(cell_id);
            iter != __message_map.end())
            return iter->second.GetLossTable();
        else
            return iter->second.GetLossTable(0);
    }

private:
    std::unordered_map<UUID, AssemblerCell<MAX_SEGMENT_COUNT>> __message_map;
    std::unique_ptr<char[]> __payload;
    UUID __message_id;
    std::vector<std::string> __payloads;
};