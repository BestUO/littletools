#include "uring_write_file.h"
#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <cstring>
#include <thread>
#include <filesystem>

UringWriteFile::UringWriteFile()
    : dirty_page_list_(NUM_BUFFERS)
    , free_page_list_(NUM_BUFFERS)
{ }

bool UringWriteFile::Init(const std::string& file_name)
{
    if (!OpenFile(file_name))
    {
        return false;
    }

    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        pages_[i].index = i;
        if (auto ret = posix_memalign(reinterpret_cast<void**>(&pages_[i].data),
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
    free_page_list_.wait_dequeue(base_info_.current_page);
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

    if (auto ret = io_uring_register_buffers(&ring_, iovs_.data(), NUM_BUFFERS);
        ret < 0)
    {
        fprintf(stderr, "register_buffers: %s\n", strerror(-ret));
        return false;
    }

    cq_thread_ = std::thread([this]() {
        DealWithCQ();
    });
    return true;
}

bool UringWriteFile::UnInit()
{
    std::lock_guard<std::mutex> lock(write_mutex_);
    FlushWithoutLock();

    stop_flag_ = true;
    WakeUpCqThread(&ring_);
    if (cq_thread_.joinable())
        cq_thread_.join();

    io_uring_unregister_buffers(&ring_);
    io_uring_queue_exit(&ring_);

    for (int i = 0; i < NUM_BUFFERS; i++)
        free(pages_[i].data);
    close(fd_);

    return true;
}

bool UringWriteFile::OpenFile(const std::string& file_name)
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
            fprintf(stderr, "exists check failed: %s\n", ec.message().c_str());
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

bool UringWriteFile::ResumeFromExistingFile()
{
    struct stat st;
    if (fstat(fd_, &st) < 0)
    {
        perror("fstat");
        return false;
    }

    if (st.st_size == 0)
    {
        memcpy(base_info_.current_page->data, BOM_STR.data(), BOM_STR.size());
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

void UringWriteFile::WriteMsg(std::string_view msg)
{
    std::lock_guard<std::mutex> lock(write_mutex_);
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
            dirty_page_list_.enqueue(base_info_.current_page);
            if (dirty_page_list_.size_approx()
                >= static_cast<size_t>(BATCH_SIZE))
            {
                SubmitDirtyPages();
            }
            base_info_.file_offset += PAGE_CACHE;
            base_info_.current_page = AcquireFreeBufSlot();
        }
    }
}

void UringWriteFile::SubmitDirtyPages()
{
    unsigned available_count = dirty_page_list_.try_dequeue_bulk(
        tmp_slots_.data(), io_uring_sq_space_left(&ring_));
    for (auto i = 0; i < available_count; i++)
    {
        struct io_uring_sqe* sqe = io_uring_get_sqe(&ring_);
        auto slot                = tmp_slots_[i];
        if (slot)
        {
            io_uring_prep_write_fixed(sqe,
                fd_,
                slot->data,
                slot->current_pos,
                slot->file_offset,
                slot->index);
        }
        else
        {
            io_uring_prep_nop(sqe);
        }
        io_uring_sqe_set_data(sqe, slot);
    }
    io_uring_submit(&ring_);
}

UringWriteFile::PageInfo* UringWriteFile::AcquireFreeBufSlot()
{
    PageInfo* slot;
    free_page_list_.wait_dequeue(slot);
    slot->current_pos = 0;
    slot->file_offset = base_info_.file_offset;
    return slot;
}

void UringWriteFile::DealWithCQ()
{
    struct __kernel_timespec ts                  = {15, 0};
    struct io_uring_cqe* cqes[QUEUE_DEPTH]       = {nullptr};
    struct PageInfo* free_bufs_list[QUEUE_DEPTH] = {nullptr};

    while (!stop_flag_)
    {
        unsigned count = io_uring_peek_batch_cqe(&ring_, cqes, QUEUE_DEPTH);
        if (count == 0)
        {
            int ret = io_uring_wait_cqe_timeout(&ring_, &cqes[0], &ts);
            // int ret = io_uring_wait_cqe(&ring_, &cqes[0]);
            if (ret < 0)
            {
                if (ret == -ETIME || ret == -EINTR)
                {
                    continue;
                }
                // 其他错误(如 -EINVAL 代表内核太老不支持该特性)是致命的，
                fprintf(stderr, "wait_cqes_min_timeout: %s\n", strerror(-ret));
                break;
            }
            count = 1;
        }
        int valid_count = 0;
        for (unsigned i = 0; i < count; i++)
        {
            auto buf_slot = (PageInfo*)io_uring_cqe_get_data(cqes[i]);
            if (buf_slot)
            {
                if (cqes[i]->res < 0)
                {
                    fprintf(stderr,
                        "write failed err: %s\n",
                        strerror(-cqes[i]->res));
                }
                free_bufs_list[valid_count++] = buf_slot;
            }
        }
        // printf("Completed %d write requests from io_uring\n", count);
        io_uring_cq_advance(&ring_, count);

        if (valid_count > 0)
        {
            free_page_list_.enqueue_bulk(free_bufs_list, valid_count);
        }
    }
}

void UringWriteFile::WaitAllComplete()
{
    while (free_page_list_.size_approx() != static_cast<size_t>(NUM_BUFFERS))
    {
        std::this_thread::yield();
    }
}

void UringWriteFile::FlushWithoutLock()
{
    dirty_page_list_.enqueue(base_info_.current_page);
    SubmitDirtyPages();
    WaitAllComplete();
    base_info_.current_page = nullptr;
}

// void UringWriteFile::Flush()
// {
//     std::lock_guard<std::mutex> lock(write_mutex_);
//     FlushWithoutLock();
// }

void UringWriteFile::WakeUpCqThread(struct io_uring* ring)
{
    struct io_uring_sqe* sqe = io_uring_get_sqe(ring);
    if (sqe)
    {
        io_uring_prep_nop(sqe);
    }
    io_uring_submit(ring);
}