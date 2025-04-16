#pragma once
#include <liburing.h>
#include <fcntl.h>
#include <memory>
#include <string>
#include <cstring>
#include <vector>
#include <functional>
#include <unistd.h>

class IoUringFile
{
public:
    explicit IoUringFile(size_t queue_depth = 256)
        : queue_depth_(queue_depth)
    {
        struct io_uring_params params;
        memset(&params, 0, sizeof(params));
        params.flags          = IORING_SETUP_SQPOLL;
        params.sq_thread_cpu  = 0;
        params.sq_thread_idle = 2000;

        if (io_uring_queue_init_params(queue_depth_, &ring_, &params) < 0)
        {
            throw std::runtime_error("Failed to initialize io_uring");
        }
    }

    ~IoUringFile()
    {
        io_uring_queue_exit(&ring_);
    }

    void async_read(const std::string& filename,
        size_t size,
        std::function<void(const char*, size_t)> callback)
    {
        int fd = open(filename.c_str(), O_RDONLY);
        if (fd < 0)
        {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        auto buffer = std::make_shared<std::vector<char>>(size);
        auto cb = std::make_shared<std::function<void(const char*, size_t)>>(
            callback);

        struct io_uring_sqe* sqe = io_uring_get_sqe(&ring_);
        if (!sqe)
        {
            close(fd);
            throw std::runtime_error("Failed to get SQE");
        }

        io_uring_prep_read(sqe, fd, buffer->data(), size, 0);

        auto* data = new UserData{.fd = fd, .buffer = buffer, .callback = cb};

        io_uring_sqe_set_data(sqe, data);
        io_uring_submit(&ring_);

        // Start completion processing
        process_completion();
    }

    void async_write(const std::string& filename,
        const char* data,
        size_t size,
        std::function<void(size_t)> callback)
    {
        int fd = open(filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0)
        {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        auto buffer = std::make_shared<std::vector<char>>(data, data + size);
        auto cb     = std::make_shared<std::function<void(size_t)>>(callback);

        struct io_uring_sqe* sqe = io_uring_get_sqe(&ring_);
        if (!sqe)
        {
            close(fd);
            throw std::runtime_error("Failed to get SQE");
        }

        io_uring_prep_write(sqe, fd, buffer->data(), size, 0);

        auto* user_data
            = new UserData{.fd = fd, .buffer = buffer, .write_callback = cb};

        io_uring_sqe_set_data(sqe, user_data);
        io_uring_submit(&ring_);

        // Start completion processing
        process_completion();
    }

private:
    struct UserData
    {
        int fd;
        std::shared_ptr<std::vector<char>> buffer;
        std::shared_ptr<std::function<void(const char*, size_t)>> callback;
        std::shared_ptr<std::function<void(size_t)>> write_callback;
    };

    void process_completion()
    {
        struct io_uring_cqe* cqe;

        int ret = io_uring_wait_cqe(&ring_, &cqe);
        if (ret < 0)
        {
            throw std::runtime_error("Error waiting for completion");
        }

        auto* data = static_cast<UserData*>(io_uring_cqe_get_data(cqe));
        if (cqe->res >= 0)
        {
            if (data->callback)
            {
                (*data->callback)(data->buffer->data(), cqe->res);
            }
            if (data->write_callback)
            {
                (*data->write_callback)(cqe->res);
            }
        }

        close(data->fd);
        delete data;

        io_uring_cqe_seen(&ring_, cqe);
    }

    struct io_uring ring_;
    const size_t queue_depth_;
};