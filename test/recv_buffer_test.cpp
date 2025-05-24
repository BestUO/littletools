#include <mutex>
#include <string>
#include <string_view>
#include <vector>
#include <thread>
#include <random>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "../tools/recv_buffer.hpp"

TEST_CASE("RecvBuffer_basic_functionality")
{
    BUFFER::V1::RecvBuffer<1024> buffer;

    SUBCASE("basic append and consumer")
    {
        const char* data = "Hello, World!";
        size_t len       = strlen(data);

        BUFFER::RecvBufferError result = buffer.Append(data, len);
        CHECK(result == BUFFER::RING_BUFFER_OK);

        std::string consumed = buffer.ConsumeAndGetString(len);
        CHECK(consumed == data);
        CHECK(consumed.length() == len);
    }

    SUBCASE("empty buffer consumer")
    {
        std::string result = buffer.ConsumeAndGetString(10);
        CHECK(result.empty());
    }

    SUBCASE("partial consumer")
    {
        std::string data = "abcdefghijklmnopqrstuvwxyz";
        buffer.Append(data.c_str(), data.length());

        std::string part1 = buffer.ConsumeAndGetString(10);
        std::string part2 = buffer.ConsumeAndGetString(10);
        std::string part3 = buffer.ConsumeAndGetString(6);

        CHECK(part1 == "abcdefghij");
        CHECK(part2 == "klmnopqrst");
        CHECK(part3 == "uvwxyz");

        std::string remaining = buffer.ConsumeAndGetString(1);
        CHECK(remaining.empty());
    }
}

TEST_CASE("RecvBuffer_overflow_and_edge_cases")
{
    SUBCASE("buffer overflow")
    {
        BUFFER::V1::RecvBuffer<100> buffer;

        std::string large_data(200, 'A');
        BUFFER::RecvBufferError result
            = buffer.Append(large_data.c_str(), large_data.length());
        CHECK(result == BUFFER::RING_BUFFER_INVALID_SIZE);
    }

    SUBCASE("buffer full")
    {
        BUFFER::V1::RecvBuffer<100> buffer;

        std::string data(90, 'X');
        BUFFER::RecvBufferError result1
            = buffer.Append(data.c_str(), data.length());
        CHECK(result1 == BUFFER::RING_BUFFER_OK);

        std::string more_data(20, 'Y');
        BUFFER::RecvBufferError result2
            = buffer.Append(more_data.c_str(), more_data.length());
        CHECK(result2 == BUFFER::RING_BUFFER_FULL);
    }

    SUBCASE("zero length data")
    {
        BUFFER::V1::RecvBuffer<100> buffer;
        BUFFER::RecvBufferError result = buffer.Append("", 0);
        CHECK(result == BUFFER::RING_BUFFER_INVALID_SIZE);
    }

    SUBCASE("consumer more than available")
    {
        BUFFER::V1::RecvBuffer<100> buffer;
        buffer.Append("test", 4);

        std::string consumed = buffer.ConsumeAndGetString(100);
        CHECK(consumed.length() == 4);
        CHECK(consumed == "test");
    }
}

TEST_CASE("RecvBuffer_data_management")
{
    SUBCASE("buffer reuse after consumption")
    {
        BUFFER::V1::RecvBuffer<512> buffer;

        for (int i = 0; i < 10; ++i)
        {
            std::string data = "Iteration " + std::to_string(i);
            BUFFER::RecvBufferError result
                = buffer.Append(data.c_str(), data.length());
            CHECK(result == BUFFER::RING_BUFFER_OK);

            std::string consumed = buffer.ConsumeAndGetString(data.length());
            CHECK(consumed == data);
        }
    }
    {
        BUFFER::V1::RecvBuffer<1024> buffer;

        std::vector<std::string> fragments
            = {"Hello", " ", "World", "!", " ", "Test", " ", "Data"};
        std::string expected;

        for (const auto& fragment : fragments)
        {
            BUFFER::RecvBufferError result
                = buffer.Append(fragment.c_str(), fragment.length());
            CHECK(result == BUFFER::RING_BUFFER_OK);
            expected += fragment;
        }

        std::string consumed = buffer.ConsumeAndGetString(expected.length());
        CHECK(consumed == expected);
    }

    SUBCASE("buffer compaction test")
    {
        BUFFER::V1::RecvBuffer<100> buffer;

        buffer.Append("12345", 5);
        std::string part1 = buffer.ConsumeAndGetString(3);
        CHECK(part1 == "123");

        buffer.Append("67890ABCDEF", 11);

        std::string remaining = buffer.ConsumeAndGetString(100);
        CHECK(remaining == "4567890ABCDEF");
    }
}

TEST_CASE("RecvBuffer_large_data_handling")
{
    BUFFER::V1::RecvBuffer<65536> buffer;

    SUBCASE("large data append and consumer")
    {
        std::string large_data(30000, 'X');
        BUFFER::RecvBufferError result
            = buffer.Append(large_data.c_str(), large_data.length());
        CHECK(result == BUFFER::RING_BUFFER_OK);

        std::string consumed1 = buffer.ConsumeAndGetString(10000);
        std::string consumed2 = buffer.ConsumeAndGetString(10000);
        std::string consumed3 = buffer.ConsumeAndGetString(10000);

        CHECK(consumed1.length() == 10000);
        CHECK(consumed2.length() == 10000);
        CHECK(consumed3.length() == 10000);
        CHECK(consumed1 == std::string(10000, 'X'));
        CHECK(consumed2 == std::string(10000, 'X'));
        CHECK(consumed3 == std::string(10000, 'X'));
    }
}

TEST_CASE("RecvBuffer_string_view_operator")
{
    BUFFER::V1::RecvBuffer<1024> buffer;

    SUBCASE("string_view functionality")
    {
        const char* data = "Test string view";
        buffer.Append(data, strlen(data));

        std::string_view view = buffer;
        CHECK(view.length() == strlen(data));

        buffer.ConsumeAndGetString(5);
        std::string_view view2 = buffer;
    }
}

TEST_CASE("RecvBuffer_boundary_conditions")
{
    SUBCASE("exact buffer size")
    {
        BUFFER::V1::RecvBuffer<10> buffer;

        std::string data(9, 'A');
        BUFFER::RecvBufferError result
            = buffer.Append(data.c_str(), data.length());
        CHECK(result == BUFFER::RING_BUFFER_OK);

        std::string consumed = buffer.ConsumeAndGetString(data.length());
        CHECK(consumed == data);
    }

    SUBCASE("multiple small appends")
    {
        BUFFER::V1::RecvBuffer<100> buffer;

        std::string total_data;
        for (int i = 0; i < 20; ++i)
        {
            std::string small_data = std::to_string(i);
            BUFFER::RecvBufferError result
                = buffer.Append(small_data.c_str(), small_data.length());
            CHECK(result == BUFFER::RING_BUFFER_OK);
            total_data += small_data;
        }

        std::string consumed = buffer.ConsumeAndGetString(total_data.length());
        CHECK(consumed == total_data);
    }
}

TEST_CASE("RecvBuffer_concurrent_access")
{
    BUFFER::V1::RecvBuffer<200> buffer;
    std::mutex mtx;
    int producer_bytes = 0;
    int consumer_bytes = 0;

    std::vector<std::string> data;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(50, 200);
    for (int i = 0; i < 100000; i++)
    {
        std::string str((size_t)distrib(gen), 'X');
        data.push_back(str);
        producer_bytes += str.length();
    }

    SUBCASE("simple concurrent test")
    {
        std::thread producer([&]() {
            auto cb = [&](const std::string& data) {
                std::lock_guard<std::mutex> lock(mtx);
                return buffer.Append(data.c_str(), data.length());
            };
            for (auto& d : data)
            {
                while (cb(d) != BUFFER::RING_BUFFER_OK)
                {
                    std::this_thread::yield();
                }
            }
        });

        std::thread consumer(
            [&buffer, &consumer_bytes, &mtx, &producer_bytes]() {
                while (true)
                {
                    auto cb = [&](size_t len) {
                        std::lock_guard<std::mutex> lock(mtx);
                        return buffer.ConsumeAndGetString(len);
                    };

                    if (auto s = cb(10); !s.empty())
                    {
                        consumer_bytes += s.length();
                    }
                    else
                    {
                        std::this_thread::yield();
                    }
                    if (consumer_bytes == producer_bytes)
                    {
                        break;
                    }
                }
            });

        producer.join();
        consumer.join();

        CHECK(consumer_bytes == producer_bytes);
    }
}

TEST_CASE("RecvBuffer_performance")
{
    std::vector<std::string> data;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(50, 200);
    for (int i = 0; i < 100000; i++)
    {
        std::string str((size_t)distrib(gen), 'X');
        data.push_back(str);
    }

    SUBCASE("string single thread performance")
    {
        std::string buffer;
        ankerl::nanobench::Bench().epochs(3).run(
            "std::string append/substr", [&]() {
                for (auto& d : data)
                {
                    buffer.append(d.c_str(), d.length());
                    while (buffer.length() > 30)
                    {
                        auto s = buffer.substr(0, 30);
                        buffer = buffer.substr(30);
                    }
                }
            });
    }

    SUBCASE("recvbuffer single thread performance")
    {
        constexpr size_t max_size = 65536;
        auto buffers = std::make_tuple(std::make_pair(std::string("V1"),
                                           BUFFER::V1::RecvBuffer<max_size>()),
            std::make_pair(
                std::string("V2"), BUFFER::V2::RecvBuffer<max_size>()));

        std::apply(
            [&](auto&... buffer_pairs) {
                ((ankerl::nanobench::Bench()
                         .epochs(3)
                         .minEpochIterations(100)
                         .run(
                             buffer_pairs.first + " RecvBuffer append/consumer",
                             [&]() {
                                 auto& buffer = buffer_pairs.second;
                                 for (auto& d : data)
                                 {
                                     buffer.Append(d.c_str(), d.length());
                                     while (buffer.Size() > 30)
                                     {
                                         auto s
                                             = buffer.ConsumeAndGetString(30);
                                     }
                                 }
                             })),
                    ...);
            },
            buffers);
    }

    SUBCASE("fragmented operations performance")
    {
        BUFFER::V1::RecvBuffer<65536> buffer;

        ankerl::nanobench::Bench().epochs(3).run(
            "RecvBuffer fragmented operations", [&]() {
                for (int i = 0; i < 1000; ++i)
                {
                    buffer.Append("test", 4);
                    buffer.Append("data", 4);
                    buffer.Append("frag", 4);

                    buffer.ConsumeAndGetString(12);
                }
            });
    }
}
