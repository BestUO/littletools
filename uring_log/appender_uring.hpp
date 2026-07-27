#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <fcntl.h>
#include <array>
#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <filesystem>
#include <condition_variable>

#include "record.hpp"
#include "tools/concurrentqueue/blockingconcurrentqueue.h"
#include "liburing/liburing.h"

namespace easylog
{
class AppenderUring
{
public:
    AppenderUring()
        : free_page_list_(NUM_BUFFERS)
    { }

    AppenderUring(const std::string& filename,
        bool async,
        bool enable_console,
        size_t max_file_size,
        size_t max_files,
        bool flush_every_time)
        : free_page_list_(NUM_BUFFERS)
    {
        init(filename);
    }

    ~AppenderUring()
    {
        stop();

        for (int i = 0; i < NUM_BUFFERS; i++)
            free(pages_[i].data);
        close(fd_);
    }

    bool init(const std::string& file_name)
    {
        if (!OpenFile(file_name))
        {
            return false;
        }

        for (int i = 0; i < NUM_BUFFERS; i++)
        {
            pages_[i].index = i;
            if (auto ret
                = posix_memalign(reinterpret_cast<void**>(&pages_[i].data),
                    PAGE_CACHE,
                    PAGE_CACHE);
                ret != 0)
            {
                perror("posix_memalign");
                throw std::runtime_error("Failed to allocate aligned memory");
            }
            memset(pages_[i].data,
                ' ',
                PAGE_CACHE);  // 用空格占位，方便看到 padding 效果
            iovs_[i].iov_base = pages_[i].data;
            iovs_[i].iov_len  = PAGE_CACHE;
            free_page_list_.enqueue(&pages_[i]);
        }
        base_info_.current_page = AcquireFreePage();
        if (!ResumeFromExistingFile())
        {
            return false;
        }
        // flags: IORING_SETUP_IOPOLL | IORING_SETUP_COOP_TASKRUN |
        // IORING_SETUP_SQPOLL
        if (auto ret = io_uring_queue_init(QUEUE_DEPTH, &ring_, 0); ret < 0)
        {
            fprintf(stderr, "queue_init: %s\n", strerror(-ret));
            return false;
        }

        if (auto ret
            = io_uring_register_buffers(&ring_, iovs_.data(), NUM_BUFFERS);
            ret < 0)
        {
            fprintf(stderr, "register_buffers: %s\n", strerror(-ret));
            return false;
        }

        work_thread_ = std::thread([this]() {
            Run();
        });

        return true;
    }

    void write(record_t& r)
    {
        thread_local moodycamel::ProducerToken token{record_list_};
        record_list_.enqueue(token, std::move(r));
        cnd_.notify_one();
    }

    void flush()
    {
        record_list_.enqueue(record_t{});
        cnd_.notify_one();
    }

    void stop()
    {
        stop_flag_ = true;
        flush();

        if (work_thread_.joinable())
            work_thread_.join();
    }

    void enable_console(bool b)
    { }

    template <bool sync = false, bool enable_console = false>
    void write_record(record_t& record)
    {
        std::lock_guard<std::mutex> guard(que_mtx_);
        WriteRecord(record);
    }

private:
    struct PageInfo
    {
        int index          = 0;
        char* data         = nullptr;
        size_t current_pos = 0;
        size_t file_offset = 0;
    };

    struct BaseInfo
    {
        PageInfo* current_page = nullptr;
        size_t file_offset     = 0;
    };
    static constexpr inline size_t PAGE_CACHE  = 1024 * 1024;
    static constexpr inline size_t NUM_BUFFERS = 4;
    // sqe must be enough for all dirty pages
    static constexpr inline size_t QUEUE_DEPTH       = NUM_BUFFERS * 2;
    static constexpr inline std::string_view BOM_STR = "\xEF\xBB\xBF";
    constexpr static char digits[10]
        = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    std::array<PageInfo, NUM_BUFFERS> pages_{};
    std::array<struct iovec, NUM_BUFFERS> iovs_{};
    moodycamel::ConcurrentQueue<PageInfo*> free_page_list_;
    // moodycamel::BlockingConcurrentQueue<record_t> record_list_;
    moodycamel::ConcurrentQueue<record_t> record_list_;
    struct io_uring_cqe* cqes_[QUEUE_DEPTH] = {nullptr};
    struct io_uring ring_;
    int fd_                               = -1;
    std::atomic<uint64_t> submit_count_   = 0;
    std::atomic<uint64_t> complete_count_ = 0;

    std::atomic<bool> stop_flag_ = false;
    BaseInfo base_info_;
    std::thread work_thread_;
    std::mutex que_mtx_;
    std::condition_variable cnd_;

    template <size_t N, char c>
    void to_int(int num, char* p, int& size)
    {
        for (size_t i = 0; i < N; i++)
        {
            p[--size] = digits[num % 10];
            num       = num / 10;
        }

        if constexpr (N != 4)
            p[--size] = c;
    }

    std::tm localtime_safe(std::time_t timer)
    {
        std::tm bt{};
#if defined(__unix__)
        localtime_r(&timer, &bt);
#elif defined(_MSC_VER)
        localtime_s(&bt, &timer);
#else
        static std::mutex mtx;
        std::lock_guard<std::mutex> lock(mtx);
        bt = *std::localtime(&timer);
#endif
        return bt;
    }

    char* get_time_str(const auto& now)
    {
        static thread_local char buf[36];
        static thread_local std::chrono::seconds last_sec_{};

        std::chrono::system_clock::duration d = now.time_since_epoch();
        std::chrono::seconds s
            = std::chrono::duration_cast<std::chrono::seconds>(d);
        auto usec = std::chrono::duration_cast<std::chrono::microseconds>(d - s)
                        .count();
        int size = 26;
        if (last_sec_ == s)
        {
            to_int<6, '.'>(usec, buf, size);
            return buf;
        }

        last_sec_    = s;
        auto tm      = std::chrono::system_clock::to_time_t(now);
        auto ltm     = localtime_safe(tm);
        std::tm* gmt = &ltm;

        to_int<6, '.'>(usec, buf, size);
        to_int<2, ':'>(gmt->tm_sec, buf, size);
        to_int<2, ':'>(gmt->tm_min, buf, size);
        to_int<2, ' '>(gmt->tm_hour, buf, size);

        to_int<2, '-'>(gmt->tm_mday, buf, size);
        to_int<2, '-'>(gmt->tm_mon + 1, buf, size);
        to_int<4, ' '>(gmt->tm_year + 1900, buf, size);
        return buf;
    }

    std::string_view get_tid_buf(unsigned int tid)
    {
        static thread_local char buf[24];
        static thread_local unsigned int last_tid;
        static thread_local size_t last_len;
        if (tid == last_tid)
        {
            return {buf, last_len};
        }

        buf[0]          = '[';
        auto [ptr, ec]  = std::to_chars(buf + 1, buf + 21, tid);
        last_tid        = tid;
        last_len        = ptr - buf;
        buf[last_len++] = ']';
        buf[last_len++] = ' ';
        return {buf, last_len};
    }

    void WriteRecord(record_t& record)
    {
        auto buf = get_time_str(record.get_time_point());

        buf[26] = ' ';
        memcpy(buf + 27, severity_str(record.get_severity()).data(), 8);
        buf[35] = ' ';

        auto time_str = std::string_view(buf, 36);
        auto tid_str  = get_tid_buf(record.get_tid());
        auto file_str = record.get_file_str();
        auto msg      = record.get_message_inner();

        Append(time_str);
        Append(tid_str);
        Append(file_str);
        Append(msg);
    }

    void Append(std::string_view msg)
    {
        size_t remaining = msg.size();
        size_t offset    = 0;
        while (remaining > 0)
        {
            size_t space = PAGE_CACHE - base_info_.current_page->current_pos;
            size_t chunk = std::min(remaining, space);
            memcpy(base_info_.current_page->data
                    + base_info_.current_page->current_pos,
                msg.data() + offset,
                chunk);
            base_info_.current_page->current_pos += chunk;
            remaining -= chunk;
            offset += chunk;

            if (chunk == space)
            {
                SubmitPage(base_info_.current_page);
                base_info_.file_offset += base_info_.current_page->current_pos;
                base_info_.current_page = AcquireFreePage();
            }
        }
    }

    void SubmitPage(PageInfo* page_info)
    {
        // sqe is enough for all dirty pages, so we
        // can submit one by one
        struct io_uring_sqe* sqe = io_uring_get_sqe(&ring_);
        if (page_info)
        {
            io_uring_prep_write_fixed(sqe,
                fd_,
                page_info->data,
                page_info->current_pos,
                page_info->file_offset,
                page_info->index);
        }
        else
        {
            // couldn`t happen
            io_uring_prep_nop(sqe);
        }
        io_uring_sqe_set_data(sqe, page_info);
        submit_count_ += io_uring_submit(&ring_);
    }

    void Run()
    {
        moodycamel::ConsumerToken ctok{record_list_};
        struct record_t record_list_tmp[QUEUE_DEPTH];
        while (!stop_flag_)
        {
            if (record_list_.size_approx() == 0)
            {
                std::unique_lock lock(que_mtx_);
                cnd_.wait(lock, [&]() {
                    return record_list_.size_approx() > 0 || stop_flag_;
                });
            }
            else
            {
                if (auto record_count = record_list_.try_dequeue_bulk(
                        ctok, record_list_tmp, NUM_BUFFERS);
                    record_count > 0)
                {
                    for (unsigned i = 0; i < record_count; i++)
                    {
                        WriteRecordAndSetPage(record_list_tmp[i]);
                    }
                }
            }
        }

        while (record_list_.try_dequeue(ctok, record_list_tmp[0]))
        {
            WriteRecordAndSetPage(record_list_tmp[0]);
        }

        if (base_info_.current_page && base_info_.current_page->current_pos > 0)
        {
            SubmitPage(base_info_.current_page);
            base_info_.file_offset += base_info_.current_page->current_pos;
            base_info_.current_page = AcquireFreePage();
        }

        WaitForAllSubmissions();

        io_uring_unregister_buffers(&ring_);
        io_uring_queue_exit(&ring_);
    }

    void DealCQEOnce(io_uring_cqe* cqes[])
    {
        unsigned cqe_count = io_uring_peek_batch_cqe(&ring_, cqes, QUEUE_DEPTH);
        complete_count_ += cqe_count;
        for (int i = 0; i < cqe_count; i++)
        {
            auto page_info = (PageInfo*)io_uring_cqe_get_data(cqes[i]);
            if (page_info)
            {
                if (cqes[i]->res < 0)
                {
                    fprintf(stderr,
                        "write failed err: %s\n",
                        strerror(-cqes[i]->res));
                }
                free_page_list_.enqueue(page_info);
            }
        }
        io_uring_cq_advance(&ring_, cqe_count);
    }

    bool OpenFile(const std::string& file_name)
    {
        std::filesystem::path path(file_name);
        std::filesystem::path dir = path.parent_path();

        if (!dir.empty())
        {
            std::error_code ec;
            if (!std::filesystem::exists(dir, ec))
            {
                if (!std::filesystem::create_directories(dir, ec))
                {
                    if (ec)
                    {
                        fprintf(stderr,
                            "create_directories failed: %s\n",
                            ec.message().c_str());
                        return false;
                    }
                }
            }
            else if (ec)
            {
                fprintf(
                    stderr, "exists check failed: %s\n", ec.message().c_str());
                return false;
            }
        }

        // fd_ = open(file_name.c_str(), O_RDWR | O_CREAT | O_DIRECT, 0644);
        fd_ = open(file_name.c_str(), O_RDWR | O_CREAT, 0644);
        if (fd_ < 0)
        {
            fprintf(stderr,
                "open(%s) failed: %s\n",
                file_name.c_str(),
                strerror(errno));
            return false;
        }
        return true;
    }

    bool ResumeFromExistingFile()
    {
        struct stat st;
        if (fstat(fd_, &st) < 0)
        {
            perror("fstat");
            return false;
        }

        if (st.st_size == 0)
        {
            memcpy(
                base_info_.current_page->data, BOM_STR.data(), BOM_STR.size());
            base_info_.current_page->current_pos = BOM_STR.size();
            return true;
        }
        else
        {
            off_t full_blocks                    = st.st_size / PAGE_CACHE;
            off_t remainder                      = st.st_size % PAGE_CACHE;
            base_info_.current_page->file_offset = base_info_.file_offset
                = full_blocks * PAGE_CACHE;

            ssize_t n = pread(fd_,
                base_info_.current_page->data,
                PAGE_CACHE,
                base_info_.current_page->file_offset);
            if (n < 0)
            {
                perror("pread last block");
                return false;
            }
            else if (n != remainder)
            {
                fprintf(stderr,
                    "unexpected short read: got %zd, expected remainder %ld\n",
                    n,
                    static_cast<long>(remainder));
                return false;
            }
            base_info_.current_page->current_pos = remainder;
            return true;
        }
    }

    PageInfo* AcquireFreePage()
    {
        PageInfo* page_info;
        while (!free_page_list_.try_dequeue(page_info))
        {
            DealCQEOnce(cqes_);
        }
        page_info->current_pos = 0;
        page_info->file_offset = base_info_.file_offset;
        return page_info;
    }

    void WriteRecordAndSetPage(record_t& record)
    {
        if (!record.file_str_.empty())
        {
            WriteRecord(record);
        }
        else
        {
            // empty record, flush current pages
            if (base_info_.current_page->current_pos > 0)
            {
                SubmitPage(base_info_.current_page);
                base_info_.file_offset += base_info_.current_page->current_pos;
                base_info_.current_page = AcquireFreePage();
            }
        }
    }

    void WaitForAllSubmissions()
    {
        while (complete_count_ != submit_count_)
        {
            DealCQEOnce(cqes_);
            std::this_thread::yield();
        }
    }
};
}  // namespace easylog