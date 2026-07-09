#pragma once

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
    UringWriteFile()  = default;
    ~UringWriteFile() = default;

    void WriteMsg(std::string_view msg);
    bool Init(const std::string& file_name);
    bool UnInit();
    bool Flush();

private:
    struct BufSlot
    {
        int index          = 0;
        char* data         = nullptr;
        size_t current_pos = 0;
    };
    struct BlockFooter
    {
        uint32_t magic;
        uint32_t real_len;
    };

    struct BaseInfo
    {
        BufSlot* current_buf_slot;
        size_t file_offset;
    };

    static constexpr inline size_t BUFFER_SIZE       = 1024 * 1024 * 4;
    static constexpr inline size_t NUM_BUFFERS       = 4;
    static constexpr inline size_t QUEUE_DEPTH       = 32;
    static constexpr inline std::string_view BOM_STR = "\xEF\xBB\xBF";
    static constexpr inline size_t FOOTER_SIZE       = sizeof(BlockFooter);
    static constexpr inline size_t USABLE_SIZE = BUFFER_SIZE - FOOTER_SIZE;

    std::array<BufSlot, NUM_BUFFERS> bufs_{};
    std::array<struct iovec, NUM_BUFFERS> iovs_{};
    std::list<BufSlot*> free_bufs_list_;
    struct io_uring ring_;
    int fd_ = -1;

    std::atomic<bool> stop_flag_ = false;
    std::mutex write_mutex_;
    std::mutex buf_list_mutex_;
    std::condition_variable buf_list_mutex_cv_;
    BaseInfo base_info_;
    std::thread work_thread_;

    UringWriteFile::BufSlot* AcquireFreeBufSlot();
    void SubmitBufSlot(BufSlot* slot);
    void DealWithCQ();
    bool OpenFile(const std::string& file_name);
    bool ResumeFromExistingFile();
    bool RotateFile(const std::string& new_file_name);
    void WaitAllComplete();
    void PopFromFreeBufList(BufSlot* slot);
};