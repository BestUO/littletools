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

template <uint8_t MAX_SPLIT_COUNT = 8>
class AssemblerCell
{
public:
    using collect_type
        = std::conditional<(MAX_SPLIT_COUNT > 8), uint16_t, uint8_t>::type;

    static constexpr uint16_t row = MAX_SPLIT_COUNT > 8 ? 65536 : 256;
    static constexpr uint8_t co   = MAX_SPLIT_COUNT > 8 ? 16 : 8;

    AssemblerCell(uint32_t cell_total_count, char* dst)
        : __dst(dst)
    {
        __collect |= __table_total_count[cell_total_count - 1];
    };

    bool DealWithCellMessage(const char* src,
        const CellInfoHeader& cell_info_header)
    {
        __collect |= 1 << cell_info_header.cell_current_index;
        memcpy(__dst + cell_info_header.cell_offset,
            src,
            cell_info_header.cell_payload_size);
        return __collect == std::numeric_limits<collect_type>::max();
    }

    std::array<uint8_t, co> GetLossTable() const
    {
        return __table_loss[__collect];
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
    collect_type __collect = 0;
    char* __dst;
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

template <uint8_t MAX_SPLIT_COUNT = 8>
class MessageAssembler
{
public:
    MessageAssembler(uint64_t message_size, UUID message_id)
        : __payload(new char[message_size]())
        , __message_id(message_id){};

    bool DealWithCellMessage(const CellInfoHeader& cell_info_header,
        uint64_t message_offset,
        const char* src)
    {
        if (__message_map.find(cell_info_header.cell_id) == __message_map.end())
        {
            __message_map.emplace(cell_info_header.cell_id,
                AssemblerCell<MAX_SPLIT_COUNT>(
                    cell_info_header.cell_total_count,
                    __payload.get() + message_offset));
        }
        if (auto iter = __message_map.find(cell_info_header.cell_id);
            iter != __message_map.end())
        {
            if (iter->second.DealWithCellMessage(src, cell_info_header))
                return true;
        }
        return false;
    }

    std::unique_ptr<char[]> GetPalyload()
    {
        return std::move(__payload);
    }

    std::array<uint8_t, MAX_SPLIT_COUNT> DealWithTimeout(UUID cell_id)
    {
        if (auto iter = __message_map.find(cell_id);
            iter != __message_map.end())
            return iter->second.GetLossTable();
        else
            return iter->second.GetLossTable(0);
    }

private:
    std::unordered_map<UUID, AssemblerCell<MAX_SPLIT_COUNT>> __message_map;
    std::unique_ptr<char[]> __payload;
    UUID __message_id;
};