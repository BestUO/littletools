/*
 * Copyright (c) 2023, Alibaba Group Holding Limited;
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <exception>
#include <system_error>
#ifdef HAVE_GLOG
    #include <glog/logging.h>
#endif

#ifdef HAVE_SPDLOG
    #include <spdlog/sinks/base_sink.h>
    #include <spdlog/sinks/basic_file_sink.h>
    #include <spdlog/spdlog.h>
    #include "spdlog/async.h"
#endif

#include <filesystem>
#include "easylog.hpp"

using namespace easylog;

class ScopedTimer
{
public:
    ScopedTimer(const char* name)
        : m_name(name)
        , m_beg(std::chrono::high_resolution_clock::now())
    { }
    ScopedTimer(const char* name, uint64_t& ns)
        : ScopedTimer(name)
    {
        m_ns = &ns;
    }
    ~ScopedTimer()
    {
        auto end = std::chrono::high_resolution_clock::now();
        auto dur
            = std::chrono::duration_cast<std::chrono::nanoseconds>(end - m_beg);
        if (m_ns)
            *m_ns = dur.count();

        std::cout << m_name << " : " << dur.count() << " ns\n";
    }

private:
    const char* m_name;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_beg;
    uint64_t* m_ns = nullptr;
};

void test_easylog(std::string filename,
    int count,
    bool async,
    size_t thread_count,
    bool use_uring)
{
    std::error_code ec;
    if (ec)
    {
        std::cout << ec.message() << "\n";
    }
    std::vector<std::thread> threads;
    threads.reserve(thread_count);
    easylog::init_log(Severity::DEBUG, filename, async, false, use_uring);
    {
        ScopedTimer timer("easylog");
        for (size_t t = 0; t < thread_count; ++t)
        {
            threads.emplace_back([&]() {
                for (int i = 0; i < count; i++)
                    ELOGFMT(INFO, "Hello logger: msg number {}", i);
            });
        }

        for (auto& t : threads)
        {
            t.join();
        }
    }
}

#ifdef HAVE_SPDLOG

void bench_mt(int howmany,
    std::shared_ptr<spdlog::logger> log,
    size_t thread_count)
{
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    spdlog::drop(log->name());

    std::vector<std::thread> threads;
    threads.reserve(thread_count);
    {
        ScopedTimer timer("spdlog ");
        for (size_t t = 0; t < thread_count; ++t)
        {
            threads.emplace_back([&]() {
                for (int j = 0; j < howmany; j++)
                {
                    SPDLOG_LOGGER_INFO(log, "Hello logger: msg number {}", j);
                }
            });
        }

        for (auto& t : threads)
        {
            t.join();
        };
    }
}
#endif

int main()
{
    std::cout << "========write 2,500,000 msgs with 5 threads, format: '[time] "
                 "[level] "
                 "[tid] [position] "
                 "[msg]', 80 Bytes every msg, total about 210MB===========\n";
    int single_thread_count = 500000;
#ifdef HAVE_SPDLOG
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%t] [%s:%#] %v");

    std::cout << "========spdlog sync with 5 threads system write===========\n";
    std::filesystem::remove("log/spdlog_5t_sync_system_write.txt");
    auto sync_basic_mt = spdlog::basic_logger_mt(
        "basic_mt", "log/spdlog_5t_sync_system_write.txt", true);
    bench_mt(single_thread_count, std::move(sync_basic_mt), 5);

    // std::cout
    //     << "========spdlog async with 5 threads system write===========\n";
    // std::filesystem::remove("log/spdlog_5t_async_system_write.txt");
    // auto async_basic_mt = spdlog::basic_logger_mt<spdlog::async_factory>(
    //     "basic_mt", "log/spdlog_5t_async_system_write.txt", true);
    // bench_mt(single_thread_count, std::move(async_basic_mt), 5);
#endif

    std::cout << "========easylog sync with 5 thread system write===========\n";
    std::filesystem::remove("log/easylog_5t_sync_system_write.txt");
    test_easylog("log/easylog_5t_sync_system_write.txt",
        single_thread_count,
        /*async =*/false,
        5,
        false);

    std::cout
        << "========easylog async with 5 thread system write===========\n";
    std::filesystem::remove("log/easylog_5t_async_system_write.txt");
    test_easylog("log/easylog_5t_async_system_write.txt",
        single_thread_count,
        /*async =*/true,
        5,
        false);

    std::cout << "========easylog sync with 5 thread uring write===========\n";
    std::filesystem::remove("log/easylog_5t_sync_uring_write.txt");
    test_easylog("log/easylog_5t_sync_uring_write.txt",
        single_thread_count,
        /*async =*/false,
        5,
        true);

    std::cout << "========easylog async with 5 thread uring write===========\n";
    std::filesystem::remove("log/easylog_5t_async_uring_write.txt");
    test_easylog("log/easylog_5t_async_uring_write.txt",
        single_thread_count,
        /*async =*/true,
        5,
        true);
}