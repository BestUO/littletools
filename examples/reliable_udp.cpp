#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/reliable_udp/rudp_struct.hpp"
#include "tools/reliable_udp/spliter.hpp"
#include "tools/reliable_udp/assembler.hpp"
#include "tools/reliable_udp/flow_control.hpp"
#include "tools/network/socket.hpp"
#include "tools/reliable_udp/large_msg_send.hpp"
#include "tools/reliable_udp/large_msg_recv.hpp"
#include "tools/reliable_udp/multi_msg_send.hpp"
#include "tools/reliable_udp/multi_msg_recv.hpp"
#include "tools/network/network_global.hpp"
#include "tools/uuid.hpp"

TEST_CASE("ReliableUDP_SplitCell_MessageInfo")
{
    MessageInfo a(ReliableUDPType::CellSend,
        UUID::gen(),
        10,
        0,
        CellInfo{CellInfoHeader{UUID::gen(), 0, 0, 2}, "12345"});
    std::string s(a.CalculateSize(), '\0');
    a.serialize(s.data());
    UUID* message_id = (UUID*)(s.data() + 1);
    auto amessage_id = EndianSwap<>::swap(*message_id);
    UUID* cell_id    = (UUID*)(s.data() + 33);
    auto acell_id    = EndianSwap<>::swap(*cell_id);
    MessageInfo b;
    b.deserialize(s.data());
    CHECK(a.type == b.type);
    CHECK(a.message_id == b.message_id);
    CHECK(a.message_size == b.message_size);
    CHECK(a.message_offset == b.message_offset);
    CHECK(a.cell_info.cell_header.cell_current_index
        == b.cell_info.cell_header.cell_current_index);
    CHECK(a.cell_info.cell_header.cell_offset
        == b.cell_info.cell_header.cell_offset);
    CHECK(a.cell_info.cell_header.cell_segment_count
        == b.cell_info.cell_header.cell_segment_count);
    CHECK(a.cell_info.cell_header.cell_id == b.cell_info.cell_header.cell_id);
    CHECK(a.cell_info.cell_payload != b.cell_info.cell_payload);
}

TEST_CASE("ReliableUDP_SplitCell_CellReceived")
{
    CellReceived a(ReliableUDPType::CellReceived, UUID::gen(), UUID::gen());
    std::string s(a.CalculateSize(), '\0');
    a.serialize(s.data());
    auto type       = *(ReliableUDPType*)s.data();
    auto message_id = EndianSwap<>::swap(*(UUID*)(s.data() + 1));
    auto cell_id    = EndianSwap<>::swap(*(UUID*)(s.data() + 17));
    CHECK(type == ReliableUDPType::CellReceived);
    CHECK(a.message_id == message_id);
    CHECK(a.cell_id == cell_id);

    // std::string s2(a.CalculateSize(), '\0');
    // CellReceived b(s.data());
    CellReceived b(ReliableUDPType::CellReceived, UUID::gen(), UUID::gen());
    b.deserialize(s.data());
    CHECK(a.type == b.type);
    CHECK(a.message_id == b.message_id);
    CHECK(a.cell_id == b.cell_id);
}

TEST_CASE("ReliableUDP_SplitCell_221")
{
    std::string s = "12345";
    RUDP::SplitCell<8, 2> split_cell(
        std::move(s), UUID::gen(), 5, 0, UUID::gen());
    auto messages_infos = split_cell.GetMessagesForTest();

    CHECK(messages_infos.size() == 3);
    CHECK(messages_infos[0].cell_info.cell_header.cell_current_index == 0);
    CHECK(messages_infos[0].cell_info.cell_header.cell_segment_count == 3);
    CHECK(messages_infos[0].cell_info.cell_header.cell_offset == 0);
    CHECK(messages_infos[0].cell_info.cell_payload == "12");

    CHECK(messages_infos[1].cell_info.cell_header.cell_current_index == 1);
    CHECK(messages_infos[1].cell_info.cell_header.cell_segment_count == 3);
    CHECK(messages_infos[1].cell_info.cell_header.cell_offset == 2);
    CHECK(messages_infos[1].cell_info.cell_payload == "34");

    CHECK(messages_infos[2].cell_info.cell_header.cell_current_index == 2);
    CHECK(messages_infos[2].cell_info.cell_header.cell_segment_count == 3);
    CHECK(messages_infos[2].cell_info.cell_header.cell_offset == 4);
    CHECK(messages_infos[2].cell_info.cell_payload == "5");

    int a = 0;
    split_cell.DealWithMessage([&a](MessageInfo) {
        a++;
    });
    CHECK(a == 3);
}

TEST_CASE("ReliableUDP_SplitCell_5")
{
    std::string s = "12345";
    RUDP::SplitCell<8, 10> split_cell(
        std::move(s), UUID::gen(), 5, 0, UUID::gen());
    auto messages_infos = split_cell.GetMessagesForTest();

    CHECK(messages_infos.size() == 1);
    CHECK(messages_infos[0].message_size == 5);
    CHECK(messages_infos[0].cell_info.cell_header.cell_offset == 0);
    CHECK(messages_infos[0].cell_info.cell_header.cell_current_index == 0);
    CHECK(messages_infos[0].cell_info.cell_header.cell_segment_count == 1);
    CHECK(messages_infos[0].cell_info.cell_payload == "12345");
}

TEST_CASE("ReliableUDP_MessageSpliter")
{
    std::string message("123456789");
    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 1;
    RUDP::MessageSpliter<MAX_SEGMENT_COUNT, MAX_SEGMENT_PAYLOAD_SIZE>
        message_split(std::move(message),
            UUID::gen(),
            network::SocketBase::CreateAddr("127.0.0.1", 0));
    int a = 0;
    message_split.DealWithSplitCell(
        [&a](MessageInfo) {
            a++;
        },
        nullptr);
    CHECK(a == 9);
    CHECK(message_split.GetSplitCellSizeForTest() == 2);
}

TEST_CASE("ReliableUDP_FlowControl")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    FlowControl fc(1024);
    fc.Wait(1023);
    CHECK(true);
    fc.Wait(1023);
    CHECK(true);
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("ReliableUDP_AssemblerCell")
{
    constexpr int cell_total_size    = 10;
    constexpr int cell_segment_count = 2;
    std::string msg1                 = "0 23";
    std::string msg2                 = "56789";
    auto cell_message = std::unique_ptr<char>(new char[cell_total_size]);
    AssemblerCell<8> assembler_cell(cell_segment_count, cell_message.get());
    CellInfoHeader cell_info_header1 = {UUID::gen(), 0, 0, 2};
    CellInfoHeader cell_info_header2 = {UUID::gen(), 1, 5, 2};
    CHECK(assembler_cell.DealWithCellMessage(msg1.data(), cell_info_header1, 5)
        == false);
    CHECK(assembler_cell.DealWithCellMessage(msg2.data(), cell_info_header2, 5)
        == true);
    uint8_t b[] = {'0', ' ', '2', '3', '\0', '5', '6', '7', '8', '9'};
    CHECK(memcmp(assembler_cell.GetPayload(), b, 10) == 0);
}

TEST_CASE("ReliableUDP_Assembler")
{
    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 5;
    constexpr int total_size               = 10;
    constexpr int total_count              = 2;
    auto message_id                        = UUID::gen();
    MessageAssembler<MAX_SEGMENT_COUNT> assembler(total_size, message_id);
    std::string msg1 = "0 23";
    std::string msg2 = "56789";
    auto cell_id     = UUID::gen();
    CHECK(assembler.DealWithCellMessage(
              CellInfoHeader{cell_id, 0, 0, 2}, 0, msg1.data(), 5)
        == false);
    CHECK(assembler.DealWithCellMessage(
              CellInfoHeader{cell_id, 1, 5, 2}, 0, msg2.data(), 5)
        == true);

    uint8_t b[] = {'0', ' ', '2', '3', '\0', '5', '6', '7', '8', '9'};

    CHECK(memcmp(assembler.GetPalyload().get(), b, 10) == 0);
}

TEST_CASE("ReliableUDP_large_msg_send_1cell_1package")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();
    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 8;
    constexpr int BAND_WIDTH               = 1024 * 10;
    auto flow_control = std::make_shared<FlowControl>(BAND_WIDTH);
    RUDP::LargeMsg::RUDPLargeMsgSend<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    std::unique_ptr<char[]> recv_clone;
    RUDP::LargeMsg::RUDPLargeMsgRecv<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });

    std::string message = "12345678";
    auto message_clone  = message;
    rudp_large_msg_send.Send(std::move(message_clone),
        network::SocketBase::CreateAddr("127.0.0.1", 9988));
    while (recv_clone == nullptr)
        usleep(1000);
    CHECK(memcmp(recv_clone.get(), message.c_str(), message.size()) == 0);

    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("ReliableUDP_large_msg_send_1cell_7package")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();
    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 1;
    constexpr int BAND_WIDTH               = 1024 * 10;
    auto flow_control = std::make_shared<FlowControl>(BAND_WIDTH);
    RUDP::LargeMsg::RUDPLargeMsgSend<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    std::unique_ptr<char[]> recv_clone;
    RUDP::LargeMsg::RUDPLargeMsgRecv<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });

    std::string message = "1234567";
    auto message_clone  = message;
    rudp_large_msg_send.Send(std::move(message_clone),
        network::SocketBase::CreateAddr("127.0.0.1", 9988));
    while (recv_clone == nullptr)
        usleep(1000);
    CHECK(memcmp(recv_clone.get(), message.c_str(), message.size()) == 0);

    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("ReliableUDP_large_msg_send_1cell_8package")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();
    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 1;
    constexpr int BAND_WIDTH               = 1024 * 10;
    auto flow_control = std::make_shared<FlowControl>(BAND_WIDTH);
    RUDP::LargeMsg::RUDPLargeMsgSend<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    std::unique_ptr<char[]> recv_clone;
    RUDP::LargeMsg::RUDPLargeMsgRecv<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });

    std::string message = "12345678";
    auto message_clone  = message;
    rudp_large_msg_send.Send(std::move(message_clone),
        network::SocketBase::CreateAddr("127.0.0.1", 9988));
    while (recv_clone == nullptr)
        usleep(1000);
    CHECK(memcmp(recv_clone.get(), message.c_str(), message.size()) == 0);

    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("ReliableUDP_large_msg_send_2cell_9package")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();
    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 1;
    constexpr int BAND_WIDTH               = 1024 * 10;
    auto flow_control = std::make_shared<FlowControl>(BAND_WIDTH);
    RUDP::LargeMsg::RUDPLargeMsgSend<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    std::unique_ptr<char[]> recv_clone;
    RUDP::LargeMsg::RUDPLargeMsgRecv<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });

    std::string message = "123456789";
    auto message_clone  = message;
    rudp_large_msg_send.Send(std::move(message_clone),
        network::SocketBase::CreateAddr("127.0.0.1", 9988));
    while (recv_clone == nullptr)
        usleep(1000);
    CHECK(memcmp(recv_clone.get(), message.c_str(), message.size()) == 0);

    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("ReliableUDP_large_msg_send_lost_all")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();
    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 1;
    constexpr int BAND_WIDTH               = 1024 * 10;
    auto flow_control = std::make_shared<FlowControl>(BAND_WIDTH);
    RUDP::LargeMsg::RUDPLargeMsgSend<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    std::string message = "12345678";
    auto message_clone  = message;
    rudp_large_msg_send.Send(std::move(message_clone),
        network::SocketBase::CreateAddr("127.0.0.1", 9988));
    std::unique_ptr<char[]> recv_clone;
    RUDP::LargeMsg::RUDPLargeMsgRecv<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });
    while (recv_clone == nullptr)
        usleep(1000);
    CHECK(memcmp(recv_clone.get(), message.c_str(), message.size()) == 0);
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("ReliableUDP_large_msg_send_lost_partition")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();
    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 1;
    constexpr int BAND_WIDTH               = 1024 * 10;
    std::unique_ptr<char[]> recv_clone;
    RUDP::LargeMsg::RUDPLargeMsgRecv<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });
    auto flow_control = std::make_shared<FlowControl>(BAND_WIDTH);
    RUDP::LargeMsg::RUDPLargeMsgSend<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);

    std::string message       = "12345678";
    std::string message_clone = message;
    rudp_large_msg_send.SendLost(std::move(message_clone),
        network::SocketBase::CreateAddr("127.0.0.1", 9988),
        0);
    sleep(2);
    CHECK(memcmp(recv_clone.get(), message.c_str(), message.size()) == 0);
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("ReliableUDP_large_msg_send_100k")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();
    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 1024;
    constexpr int BAND_WIDTH               = 1024 * 1024;
    std::unique_ptr<char[]> recv_clone;
    RUDP::LargeMsg::RUDPLargeMsgRecv<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });
    int send_count    = 0;
    auto flow_control = std::make_shared<FlowControl>(BAND_WIDTH);
    RUDP::LargeMsg::RUDPLargeMsgSend<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    sleep(1);
    std::string message(1024 * 100, '\100');
    auto dst = network::SocketBase::CreateAddr("127.0.0.1", 9988);

    auto now = std::chrono::high_resolution_clock::now();
    rudp_large_msg_send.Send(std::move(message), dst);
    while (recv_clone == nullptr)
        usleep(10);
    std::cout << "100k msg use: "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     std::chrono::high_resolution_clock::now() - now)
                     .count()
              << "us" << std::endl;

    CHECK(memcmp(recv_clone.get(), message.c_str(), message.size()) == 0);
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}

TEST_CASE("ReliableUDP_multi_msg_send")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();
    constexpr int MAX_SEGMENT_COUNT        = 8;
    constexpr int MAX_SEGMENT_PAYLOAD_SIZE = 1024;
    constexpr int BAND_WIDTH               = 1024 * 1024;
    std::vector<std::string> recv_clone;
    RUDP::MultiMsg::RUDPLargeMsgRecv<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::vector<std::string>&& recv) {
                recv_clone = std::move(recv);
            });
    int send_count    = 0;
    auto flow_control = std::make_shared<FlowControl>(BAND_WIDTH);
    RUDP::MultiMsg::RUDPLargeMsgSend<MAX_SEGMENT_COUNT,
        MAX_SEGMENT_PAYLOAD_SIZE,
        BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    sleep(1);
    std::vector<std::string> v;
    for (int i = 0; i < 100; i++)
        v.push_back(std::to_string(i));

    auto dst = network::SocketBase::CreateAddr("127.0.0.1", 9988);

    auto now = std::chrono::high_resolution_clock::now();
    rudp_large_msg_send.Send(std::move(v), dst);
    while (recv_clone.empty())
        usleep(10);
    std::cout << "100 msg use: "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     std::chrono::high_resolution_clock::now() - now)
                     .count()
              << "us" << std::endl;

    CHECK(recv_clone.size() == 100);
    for (int i = 0; i < 100; i++)
        CHECK(recv_clone[i] == std::to_string(i));
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}