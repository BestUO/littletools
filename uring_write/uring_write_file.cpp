#include "uring_write_file.h"
#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <cstring>
#include <thread>
#include <filesystem>

bool UringWriteFile::Init(const std::string& file_name)
{
    if (!OpenFile(file_name))
    {
        return false;
    }

    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        bufs_[i].index = i;
        if (auto ret = posix_memalign(reinterpret_cast<void**>(&bufs_[i].data),
                BUFFER_SIZE,
                BUFFER_SIZE);
            ret != 0)
        {
            perror("posix_memalign");
            throw std::runtime_error("Failed to allocate aligned memory");
        }
        memset(bufs_[i].data,
            ' ',
            BUFFER_SIZE);  // 用空格占位，方便看到 padding 效果
        iovs_[i].iov_base = bufs_[i].data;
        iovs_[i].iov_len  = BUFFER_SIZE;

        free_bufs_list_.push_back(&bufs_[i]);
    }
    base_info_.current_buf_slot = AcquireFreeBufSlot();
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
    work_thread_ = std::thread([this]() {
        DealWithCQ();
    });
    return true;
}

bool UringWriteFile::UnInit()
{
    std::lock_guard<std::mutex> lock(write_mutex_);
    SubmitBufSlot(base_info_.current_buf_slot);
    WaitAllComplete();

    stop_flag_               = true;
    struct io_uring_sqe* sqe = io_uring_get_sqe(&ring_);
    io_uring_prep_nop(sqe);
    io_uring_sqe_set_data(sqe, nullptr);
    io_uring_submit(&ring_);

    if (work_thread_.joinable())
        work_thread_.join();

    io_uring_unregister_buffers(&ring_);
    io_uring_queue_exit(&ring_);

    if (base_info_.current_buf_slot->current_pos > 0
        && base_info_.current_buf_slot->current_pos < BUFFER_SIZE)
    {
        off_t true_size = base_info_.file_offset
            + static_cast<off_t>(base_info_.current_buf_slot->current_pos)
            - BUFFER_SIZE;
        if (ftruncate(fd_, true_size) != 0)
            perror("ftruncate (strip trailing padding)");
    }

    for (int i = 0; i < NUM_BUFFERS; i++)
        free(bufs_[i].data);
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

    fd_ = open(file_name.c_str(), O_RDWR | O_CREAT | O_DIRECT, 0644);
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
        auto* slot = base_info_.current_buf_slot;
        memcpy(slot->data, BOM_STR.data(), BOM_STR.size());
        slot->current_pos      = BOM_STR.size();
        base_info_.file_offset = 0;
        return true;
    }
    else
    {
        off_t full_blocks = st.st_size / BUFFER_SIZE;
        off_t remainder   = st.st_size % BUFFER_SIZE;

        base_info_.file_offset = full_blocks * BUFFER_SIZE;
        ssize_t n              = pread(fd_,
            base_info_.current_buf_slot->data,
            BUFFER_SIZE,
            base_info_.file_offset);
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
        base_info_.current_buf_slot->current_pos = remainder;
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
        size_t space = BUFFER_SIZE - base_info_.current_buf_slot->current_pos;
        size_t chunk = std::min(remaining, space);
        memcpy(base_info_.current_buf_slot->data
                + base_info_.current_buf_slot->current_pos,
            msg.data() + offset,
            chunk);
        base_info_.current_buf_slot->current_pos += chunk;
        remaining -= chunk;
        offset += chunk;

        if (chunk == space)
        {
            SubmitBufSlot(base_info_.current_buf_slot);
            base_info_.current_buf_slot = AcquireFreeBufSlot();
        }
    }
}

UringWriteFile::BufSlot* UringWriteFile::AcquireFreeBufSlot()
{
    std::unique_lock<std::mutex> lock(buf_list_mutex_);
    buf_list_mutex_cv_.wait(lock, [this] {
        return !free_bufs_list_.empty();
    });
    auto buf_slot         = free_bufs_list_.front();
    buf_slot->current_pos = 0;
    memset(buf_slot->data, ' ', BUFFER_SIZE);
    free_bufs_list_.pop_front();

    return buf_slot;
}

void UringWriteFile::SubmitBufSlot(BufSlot* slot)
{
    struct io_uring_sqe* sqe = io_uring_get_sqe(&ring_);
    io_uring_prep_write_fixed(
        sqe, fd_, slot->data, BUFFER_SIZE, base_info_.file_offset, slot->index);
    base_info_.file_offset += BUFFER_SIZE;
    io_uring_sqe_set_data(sqe, slot);
    io_uring_submit(&ring_);
}

void UringWriteFile::DealWithCQ()
{
    while (!stop_flag_)
    {
        struct io_uring_cqe* cqe;
        int ret = io_uring_wait_cqe(&ring_, &cqe);
        if (ret < 0)
        {
            if (ret == -EINTR)
                continue;
            fprintf(stderr, "wait_cqe: %s\n", strerror(-ret));
            break;
        }

        auto buf_slot = (BufSlot*)io_uring_cqe_get_data(cqe);
        if (buf_slot)
        {
            if (cqe->res < 0 || cqe->res != BUFFER_SIZE)
            {
                fprintf(stderr, "write failed err: %s\n", strerror(-cqe->res));
            }
            io_uring_cqe_seen(&ring_, cqe);
            {
                std::lock_guard<std::mutex> lock(buf_list_mutex_);
                free_bufs_list_.push_back(buf_slot);
            }
            buf_list_mutex_cv_.notify_one();
        }
        else
        {
            io_uring_cqe_seen(&ring_, cqe);
            continue;
        }
    }
}

void UringWriteFile::WaitAllComplete()
{
    std::unique_lock<std::mutex> free_lock(buf_list_mutex_);
    buf_list_mutex_cv_.wait(free_lock, [this] {
        return free_bufs_list_.size() == static_cast<size_t>(NUM_BUFFERS);
    });
}

bool UringWriteFile::Flush()
{
    std::lock_guard<std::mutex> lock(write_mutex_);
    SubmitBufSlot(base_info_.current_buf_slot);
    WaitAllComplete();
    PopFromFreeBufList(base_info_.current_buf_slot);
    base_info_.file_offset -= BUFFER_SIZE;
    return true;
}

void UringWriteFile::PopFromFreeBufList(BufSlot* slot)
{
    std::lock_guard<std::mutex> lock(buf_list_mutex_);
    free_bufs_list_.remove(slot);
}
