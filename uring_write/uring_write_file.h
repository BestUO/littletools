#pragma once
#include "tools/concurrentqueue/blockingconcurrentqueue.h"
#include <cstddef>
#include <mutex>
#include <string>
#include <fcntl.h>
#include "liburing/liburing.h"
#include <array>

class UringWriteFile
{
public:
    UringWriteFile();
    ~UringWriteFile() = default;

    void WriteMsg(std::string_view msg);
    bool Init(const std::string& file_name);
    bool UnInit();
    void Flush();

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
    static constexpr inline size_t NUM_BUFFERS = 8;
    // sqe must be enough for all dirty pages
    static constexpr inline size_t QUEUE_DEPTH       = NUM_BUFFERS * 2;
    static constexpr inline std::string_view BOM_STR = "\xEF\xBB\xBF";
    static constexpr inline size_t BATCH_SIZE        = NUM_BUFFERS / 2;

    std::array<PageInfo, NUM_BUFFERS> pages_{};
    std::array<struct iovec, NUM_BUFFERS> iovs_{};
    moodycamel::BlockingConcurrentQueue<PageInfo*> dirty_page_list_;
    moodycamel::BlockingConcurrentQueue<PageInfo*> free_page_list_;
    struct io_uring ring_;
    int fd_ = -1;

    std::atomic<bool> stop_flag_ = false;
    std::mutex write_mutex_;
    BaseInfo base_info_;
    std::thread cq_thread_;
    std::array<PageInfo*, NUM_BUFFERS> tmp_slots_{};

    void DealWithCQ();
    bool OpenFile(const std::string& file_name);
    bool ResumeFromExistingFile();
    bool RotateFile(const std::string& new_file_name);
    void WaitAllComplete();
    UringWriteFile::PageInfo* AcquireFreeBufSlot();
    void FlushWithoutLock();
    void SubmitDirtyPages();
    void WakeUpCqThread(struct io_uring* ring);
};