#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
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
    CHECK(a.cell_info.cell_header.cell_total_count
        == b.cell_info.cell_header.cell_total_count);
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
    SplitCell<8, 2> split_cell(std::move(s), UUID::gen(), 5, 0, UUID::gen());
    auto messages_infos = split_cell.GetMessagesForTest();

    CHECK(messages_infos.size() == 3);
    CHECK(messages_infos[0].cell_info.cell_header.cell_current_index == 0);
    CHECK(messages_infos[0].cell_info.cell_header.cell_total_count == 3);
    CHECK(messages_infos[0].cell_info.cell_header.cell_offset == 0);
    CHECK(messages_infos[0].cell_info.cell_payload == "12");

    CHECK(messages_infos[1].cell_info.cell_header.cell_current_index == 1);
    CHECK(messages_infos[1].cell_info.cell_header.cell_total_count == 3);
    CHECK(messages_infos[1].cell_info.cell_header.cell_offset == 2);
    CHECK(messages_infos[1].cell_info.cell_payload == "34");

    CHECK(messages_infos[2].cell_info.cell_header.cell_current_index == 2);
    CHECK(messages_infos[2].cell_info.cell_header.cell_total_count == 3);
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
    SplitCell<8, 10> split_cell(std::move(s), UUID::gen(), 5, 0, UUID::gen());
    auto messages_infos = split_cell.GetMessagesForTest();

    CHECK(messages_infos.size() == 1);
    CHECK(messages_infos[0].message_size == 5);
    CHECK(messages_infos[0].cell_info.cell_header.cell_offset == 0);
    CHECK(messages_infos[0].cell_info.cell_header.cell_current_index == 0);
    CHECK(messages_infos[0].cell_info.cell_header.cell_total_count == 1);
    CHECK(messages_infos[0].cell_info.cell_payload == "12345");
}

TEST_CASE("ReliableUDP_MessageSpliter")
{
    std::string message("123456789");
    constexpr int MAX_SPLIT_COUNT  = 8;
    constexpr int MAX_PAYLOAD_SIZE = 1;
    MessageSpliter<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE> message_split(
        std::move(message),
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
    FlowControl<1024, 1024 * 10> rsend;
    rsend.Decrease(1024 * 9);
    CHECK(rsend.Pass() == true);
    rsend.Decrease(1);
    CHECK(rsend.Pass() == false);
    rsend.Increase(1);
    CHECK(rsend.Pass() == true);
}

TEST_CASE("ReliableUDP_AssemblerCell")
{
    constexpr int cell_total_size  = 10;
    constexpr int cell_total_count = 2;
    std::string msg1               = "0 23";
    std::string msg2               = "56789";
    auto cell_message = std::unique_ptr<char>(new char[cell_total_size]);
    AssemblerCell<8> assembler_cell(cell_total_count, cell_message.get());
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
    constexpr int MAX_SPLIT_COUNT  = 8;
    constexpr int MAX_PAYLOAD_SIZE = 5;
    constexpr int total_size       = 10;
    constexpr int total_count      = 2;
    auto message_id                = UUID::gen();
    MessageAssembler<MAX_SPLIT_COUNT> assembler(total_size, message_id);
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
    constexpr int MAX_SPLIT_COUNT  = 8;
    constexpr int MAX_PAYLOAD_SIZE = 8;
    constexpr int BAND_WIDTH       = 1024 * 10;
    auto flow_control              = std::make_shared<
        FlowControl<MAX_SPLIT_COUNT * MAX_PAYLOAD_SIZE, BAND_WIDTH>>();
    RUDPLargeMsgSend<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    std::unique_ptr<char[]> recv_clone;
    RUDPLargeMsgRecv<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });

    std::string message = "12345678";
    rudp_large_msg_send.Send(
        message, network::SocketBase::CreateAddr("127.0.0.1", 9988));
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
    constexpr int MAX_SPLIT_COUNT  = 8;
    constexpr int MAX_PAYLOAD_SIZE = 1;
    constexpr int BAND_WIDTH       = 1024 * 10;
    auto flow_control              = std::make_shared<
        FlowControl<MAX_SPLIT_COUNT * MAX_PAYLOAD_SIZE, BAND_WIDTH>>();
    RUDPLargeMsgSend<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    std::unique_ptr<char[]> recv_clone;
    RUDPLargeMsgRecv<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });

    std::string message = "1234567";
    rudp_large_msg_send.Send(
        message, network::SocketBase::CreateAddr("127.0.0.1", 9988));
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
    constexpr int MAX_SPLIT_COUNT  = 8;
    constexpr int MAX_PAYLOAD_SIZE = 1;
    constexpr int BAND_WIDTH       = 1024 * 10;
    auto flow_control              = std::make_shared<
        FlowControl<MAX_SPLIT_COUNT * MAX_PAYLOAD_SIZE, BAND_WIDTH>>();
    RUDPLargeMsgSend<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    std::unique_ptr<char[]> recv_clone;
    RUDPLargeMsgRecv<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });

    std::string message = "12345678";
    rudp_large_msg_send.Send(
        message, network::SocketBase::CreateAddr("127.0.0.1", 9988));
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
    constexpr int MAX_SPLIT_COUNT  = 8;
    constexpr int MAX_PAYLOAD_SIZE = 1;
    constexpr int BAND_WIDTH       = 1024 * 10;
    auto flow_control              = std::make_shared<
        FlowControl<MAX_SPLIT_COUNT * MAX_PAYLOAD_SIZE, BAND_WIDTH>>();
    RUDPLargeMsgSend<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    std::unique_ptr<char[]> recv_clone;
    RUDPLargeMsgRecv<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
                recv_clone = std::move(recv);
            });

    std::string message = "123456789";
    rudp_large_msg_send.Send(
        message, network::SocketBase::CreateAddr("127.0.0.1", 9988));
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
    constexpr int MAX_SPLIT_COUNT  = 8;
    constexpr int MAX_PAYLOAD_SIZE = 1;
    constexpr int BAND_WIDTH       = 1024 * 10;
    auto flow_control              = std::make_shared<
        FlowControl<MAX_SPLIT_COUNT * MAX_PAYLOAD_SIZE, BAND_WIDTH>>();
    RUDPLargeMsgSend<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    std::string message = "12345678";
    rudp_large_msg_send.Send(
        message, network::SocketBase::CreateAddr("127.0.0.1", 9988));
    std::unique_ptr<char[]> recv_clone;
    RUDPLargeMsgRecv<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
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

// TEST_CASE("ReliableUDP_large_msg_send_lost_partition")
// {
//     timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
//     network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();
//     constexpr int MAX_SPLIT_COUNT  = 8;
//     constexpr int MAX_PAYLOAD_SIZE = 1;
//     constexpr int BAND_WIDTH       = 1024 * 10;
//     std::unique_ptr<char[]> recv_clone;
//     RUDPLargeMsgRecv<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
//         rudp_large_msg_recv(
//             "127.0.0.1", 9988, [&recv_clone](std::unique_ptr<char[]> recv) {
//                 recv_clone = std::move(recv);
//             });
//     auto flow_control = std::make_shared<
//         FlowControl<MAX_SPLIT_COUNT * MAX_PAYLOAD_SIZE, BAND_WIDTH>>();
//     RUDPLargeMsgSend<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
//         rudp_large_msg_send("127.0.0.1", 9987, flow_control);

//     std::string message = "12345678";
//     rudp_large_msg_send.SendLost(
//         message, network::SocketBase::CreateAddr("127.0.0.1", 9988), 0);
//     sleep(2);
//     CHECK(memcmp(recv_clone.get(), message.c_str(), message.size()) == 0);
//     network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
//     timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
// }

TEST_CASE("ReliableUDP_large_msg_send_bench")
{
    timermanager::TimerManager<UUID>::GetInstance()->StartTimerManager();
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Start();
    constexpr int MAX_SPLIT_COUNT  = 8;
    constexpr int MAX_PAYLOAD_SIZE = 1024;
    constexpr int BAND_WIDTH       = 1024 * 1024 * 300;
    int recv_count                 = 0;
    RUDPLargeMsgRecv<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_recv(
            "127.0.0.1", 9988, [&recv_count](std::unique_ptr<char[]> recv) {
                recv_count++;
            });
    int send_count    = 0;
    auto flow_control = std::make_shared<
        FlowControl<MAX_SPLIT_COUNT * MAX_PAYLOAD_SIZE, BAND_WIDTH>>();
    RUDPLargeMsgSend<MAX_SPLIT_COUNT, MAX_PAYLOAD_SIZE, BAND_WIDTH>
        rudp_large_msg_send("127.0.0.1", 9987, flow_control);
    sleep(1);
    // std::string message(1024 * 1024 * 8, '\0');
    std::string message("12345");
    auto now = std::chrono::high_resolution_clock::now();
    auto dst = network::SocketBase::CreateAddr("127.0.0.1", 9988);
    std::cout << "first send message: "
              << std::chrono::steady_clock::now().time_since_epoch().count()
              << std::endl;
    while (std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::high_resolution_clock::now() - now)
               .count()
        < 10)
    {
        rudp_large_msg_send.Send(message, dst);
        send_count++;
    }
    sleep(1);
    std::cout << "send 8M msg for 10s, send_num/ps: " << send_count / 10
              << " recv_num/ps: " << recv_count / 10 << std::endl;
    network::NetWorkManager<network::SimpleEpoll>::GetInstance()->Stop();
    timermanager::TimerManager<UUID>::GetInstance()->StopTimerManager();
}