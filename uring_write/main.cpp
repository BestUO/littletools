#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <cstring>
#include <string_view>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <unistd.h>
#include <fcntl.h>
#include <ext/stdio_filebuf.h>
#include "uring_write_file.h"

class TimeCost
{
public:
    TimeCost(std::string_view log_prefix)
        : start_time_(std::chrono::high_resolution_clock::now())
        , log_prefix_(log_prefix)
    { }
    ~TimeCost()
    {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time_)
                            .count();
        printf("%s Time taken: %ld ms\n", log_prefix_.data(), duration);
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    std::string_view log_prefix_;
};

class WriteFile
{
public:
    WriteFile() = default;
    ~WriteFile()
    {
        UnInit();
    }

    bool Init(const std::string& file_name)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::filesystem::path path(file_name);
        std::filesystem::path dir = path.parent_path();
        if (!dir.empty())
        {
            std::error_code ec;
            if (!std::filesystem::create_directories(dir, ec))
            {
                if (ec && !std::filesystem::exists(dir))
                {
                    fprintf(stderr,
                        "create_directories failed: %s\n",
                        ec.message().c_str());
                    return false;
                }
            }
        }

        file_.open(file_name, std::ios::out | std::ios::app | std::ios::binary);
        if (!file_.is_open())
        {
            fprintf(stderr,
                "open(%s) failed: %s\n",
                file_name.c_str(),
                strerror(errno));
            return false;
        }

        return true;
    }

    bool UnInit()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_.is_open())
        {
            file_.close();
        }
        return true;
    }

    void WriteMsg(std::string_view msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_.is_open())
        {
            file_.write(msg.data(), msg.size());
        }
    }

    void Flush()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_.is_open())
        {
            file_.flush();
            // force the data to be written to disk
            // sync();
        }
    }

private:
    mutable std::mutex mutex_;
    std::ofstream file_;
};

void OneThreadBench()
{
    size_t count     = 500000;
    auto uring_write = std::make_shared<UringWriteFile>();
    uring_write->Init("./log/test_uring.log");
    {
        TimeCost time_cost("one thread log with uring");
        for (auto i = 0; i < count; i++)
        {
            uring_write->WriteMsg(
                "12345678901234567890123456789012345678901234567890123456789012"
                "34567890123456789012345678901234567890\n");
        }
    }
    uring_write->Flush();
    uring_write->UnInit();

    auto write_file = std::make_shared<WriteFile>();
    write_file->Init("./log/test_write.log");
    {
        TimeCost time_cost("one thread log with write");
        for (auto i = 0; i < count; i++)
        {
            write_file->WriteMsg(
                "12345678901234567890123456789012345678901234567890123456789012"
                "34567890123456789012345678901234567890\n");
        }
    }
    write_file->Flush();
    write_file->UnInit();
}

void FiveThreadsBench()
{
    size_t count = 100000;
    std::atomic<size_t> uring_cost(0);
    std::atomic<size_t> write_cost(0);

    {
        std::vector<std::thread> threads;
        auto uring_write = std::make_shared<UringWriteFile>();
        uring_write->Init("./log/test_uring.log");
        for (size_t i = 0; i < 5; i++)
        {
            threads.emplace_back([&]() {
                auto start_time = std::chrono::steady_clock::now();
                for (auto j = 0; j < count; j++)
                {
                    uring_write->WriteMsg(
                        "123456789012345678901234567890123456789012345678901234"
                        "56789012"
                        "34567890123456789012345678901234567890\n");
                }
                uring_cost.fetch_add(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - start_time)
                        .count());
            });
        }
        for (auto& t : threads)
        {
            t.join();
        }
        uring_write->Flush();
        uring_write->UnInit();
    }

    {
        auto write_file = std::make_shared<WriteFile>();
        write_file->Init("./log/test_write.log");
        std::vector<std::thread> threads;
        for (size_t i = 0; i < 5; i++)
        {
            threads.emplace_back([&]() {
                auto start_time = std::chrono::steady_clock::now();
                for (auto j = 0; j < count; j++)
                {
                    write_file->WriteMsg(
                        "123456789012345678901234567890123456789012345678901234"
                        "56789012"
                        "34567890123456789012345678901234567890\n");
                }
                write_cost.fetch_add(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - start_time)
                        .count());
            });
        }
        for (auto& t : threads)
        {
            t.join();
        }
        write_file->Flush();
        write_file->UnInit();
    }
    printf("uring write cost: %ld us, write cost: %ld us\n",
        uring_cost.load(),
        write_cost.load());
}

int main(int argc, char* argv[])
{
    // OneThreadBench();
    FiveThreadsBench();
    return 0;
}