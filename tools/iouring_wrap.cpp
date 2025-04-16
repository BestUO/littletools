#pragma once

#include <liburing.h>
#include <fcntl.h>
#include <string>
#include <vector>
#include <functional>
#include <unistd.h>
#include <memory>
#include <unordered_map>
#include <atomic>
#include <iostream>
#include <cstring>
#include <sys/stat.h>
#include <sys/mman.h>

class IoUringWrap
{
public:
    struct IoContext
    {
        enum class OpType
        {
            READ,
            WRITE
        };

        OpType type;
        void* buffer;
        size_t size;
        off_t offset;
        std::function<void(int result)> callback;
        int fd;

        IoContext(OpType t,
            void* buf,
            size_t s,
            off_t off,
            std::function<void(int)> cb,
            int file_desc)
            : type(t)
            , buffer(buf)
            , size(s)
            , offset(off)
            , callback(cb)
            , fd(file_desc)
        { }
    };

    IoUringWrap()
        : __file_fd(-1)
        , __initialized(false)
    { }

    ~IoUringWrap()
    {
        if (__initialized)
            io_uring_queue_exit(&__ring);
        if (__file_fd != -1)
            ::close(__file_fd);
    }

    bool init(unsigned int queue_depth = 32,
        unsigned int sq_thread_idle_ms = 2000)
    {
        if (__initialized)
            return true;

        struct io_uring_params params;
        memset(&params, 0, sizeof(params));

        params.flags          = IORING_SETUP_SQPOLL;
        params.sq_thread_idle = sq_thread_idle_ms;

        // 锁定内存以避免页面交换
        if (mlockall(MCL_CURRENT | MCL_FUTURE))
        {
            std::cerr << "Warning: mlockall failed: " << strerror(errno)
                      << std::endl;
        }

        if (io_uring_queue_init_params(queue_depth, &__ring, &params) < 0)
        {
            std::cerr << "io_uring_queue_init_params failed: "
                      << strerror(errno) << std::endl;
            std::cerr << "Note: IORING_SETUP_SQPOLL requires CAP_SYS_NICE "
                         "capability or root privileges"
                      << std::endl;
            return false;
        }

        __initialized = true;
        return true;
    }

    bool open(const std::string& path, int flags, mode_t mode = 0644)
    {
        if (__file_fd != -1)
        {
            ::close(__file_fd);
        }

        __file_fd = ::open(path.c_str(), flags, mode);
        if (__file_fd == -1)
        {
            std::cerr << "Failed to open file " << path << ": "
                      << strerror(errno) << std::endl;
            return false;
        }

        return true;
    }

    void close()
    {
        if (__file_fd != -1)
        {
            ::close(__file_fd);
            __file_fd = -1;
        }
    }

    bool read_async(void* buffer,
        size_t size,
        off_t offset,
        std::function<void(int)> callback)
    {
        if (!__initialized || __file_fd == -1)
        {
            return false;
        }

        struct io_uring_sqe* sqe = io_uring_get_sqe(&__ring);
        if (!sqe)
        {
            std::cerr << "io_uring_get_sqe failed" << std::endl;
            return false;
        }

        auto context = new IoContext(
            IoContext::OpType::READ, buffer, size, offset, callback, __file_fd);

        io_uring_prep_read(sqe, __file_fd, buffer, size, offset);
        io_uring_sqe_set_data(sqe, (void*)context);

        // 提交请求到队列
        int ret = io_uring_submit(&__ring);
        if (ret <= 0)
        {
            std::cerr << "io_uring_submit failed: " << strerror(-ret)
                      << std::endl;
            delete context;
            return false;
        }

        return true;
    }

    bool write_async(const void* buffer,
        size_t size,
        off_t offset,
        std::function<void(int)> callback)
    {
        if (!__initialized || __file_fd == -1)
        {
            return false;
        }

        struct io_uring_sqe* sqe = io_uring_get_sqe(&__ring);
        if (!sqe)
        {
            std::cerr << "io_uring_get_sqe failed" << std::endl;
            return false;
        }

        auto context = new IoContext(IoContext::OpType::WRITE,
            const_cast<void*>(buffer),
            size,
            offset,
            callback,
            __file_fd);

        io_uring_prep_write(sqe, __file_fd, buffer, size, offset);
        io_uring_sqe_set_data(sqe, (void*)context);

        int ret = io_uring_submit(&__ring);
        if (ret <= 0)
        {
            std::cerr << "io_uring_submit failed: " << strerror(-ret)
                      << std::endl;
            delete context;
            return false;
        }

        return true;
    }

    int process_completions(bool wait = true)
    {
        struct io_uring_cqe* cqe;
        int count = 0;

        int ret = io_uring_wait_cqe_nr(&__ring, &cqe, 1);
        if (ret < 0)
        {
            if (ret != -EAGAIN)
            {
                std::cerr << "io_uring_wait_cqe_nr failed: " << strerror(-ret)
                          << std::endl;
            }
            return 0;
        }

        unsigned head;
        io_uring_for_each_cqe(&__ring, head, cqe)
        {
            IoContext* context = (IoContext*)io_uring_cqe_get_data(cqe);
            if (context->callback)
            {
                context->callback(cqe->res);
            }
            count++;
            delete context;
        }

        io_uring_cq_advance(&__ring, count);
        return count;
    }

    struct io_uring __ring;
    int __file_fd;
    bool __initialized;
};