#pragma once
#include "tools/concurrentqueue/blockingconcurrentqueue.h"
#include <cstddef>
#include <mutex>
#include <string>
#include <fcntl.h>
#include "liburing/liburing.h"
#include <list>
#include <array>
#include <condition_variable>

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
    struct BufSlot
    {
        int index          = 0;
        char* data         = nullptr;
        size_t current_pos = 0;
        size_t file_offset = 0;
    };
    // struct BlockFooter
    // {
    //     uint32_t magic;
    //     uint32_t real_len;
    // };

    struct PageInfo
    {
        BufSlot* current_buf_slot = nullptr;
        size_t file_offset        = 0;
    };
    int count                                        = 0;
    static constexpr inline size_t PAGE_CACHE        = 1024 * 4;
    static constexpr inline size_t NUM_BUFFERS       = 1024 * 4;
    static constexpr inline size_t QUEUE_DEPTH       = 128;
    static constexpr inline std::string_view BOM_STR = "\xEF\xBB\xBF";
    // static constexpr inline size_t FOOTER_SIZE       = sizeof(BlockFooter);
    // static constexpr inline size_t USABLE_SIZE = PAGE_CACHE - FOOTER_SIZE;

    std::array<BufSlot, NUM_BUFFERS> pages_{};
    std::array<struct iovec, NUM_BUFFERS> iovs_{};
    // std::list<BufSlot*> free_page_list_;
    // std::list<BufSlot*> dirty_page_list_;
    moodycamel::BlockingConcurrentQueue<BufSlot*> dirty_page_list_;
    moodycamel::BlockingConcurrentQueue<BufSlot*> free_page_list_;
    struct io_uring ring_;
    int fd_ = -1;

    std::atomic<bool> stop_flag_ = false;
    std::mutex write_mutex_;
    PageInfo base_info_;
    std::thread sq_thread_;
    std::thread cq_thread_;

    void DealWithSQ();
    void DealWithCQ();
    bool OpenFile(const std::string& file_name);
    bool ResumeFromExistingFile();
    bool RotateFile(const std::string& new_file_name);
    void WaitAllComplete();
    UringWriteFile::BufSlot* AcquireFreeBufSlot();
    void FlushWithoutLock();
};