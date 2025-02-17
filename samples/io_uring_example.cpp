#include <liburing.h>
#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#define QUEUE_DEPTH 1
#define BUFFER_SIZE 1024

void setup_io_uring(struct io_uring& ring)
{
    if (io_uring_queue_init(256, &ring, 0) < 0)
    {
        throw std::runtime_error("Failed to initialize io_uring");
    }
}

void submit_read_request(struct io_uring& ring,
    int fd,
    char* buffer,
    size_t size)
{
    struct io_uring_sqe* sqe = io_uring_get_sqe(&ring);
    if (!sqe)
    {
        throw std::runtime_error("Failed to get SQE");
    }

    io_uring_prep_read(sqe, fd, buffer, size, 0);
    io_uring_sqe_set_data(sqe, buffer);
    io_uring_submit(&ring);
}

void wait_for_completion(struct io_uring& ring)
{
    struct io_uring_cqe* cqe;

    while (true)
    {
        io_uring_wait_cqe(&ring, &cqe);
        if (cqe->res < 0)
        {
            std::cerr << "Error in completion: " << strerror(-cqe->res)
                      << std::endl;
        }
        else
        {
            std::cout << "Completed request with result: " << cqe->res
                      << std::endl;
            break;  // Exit after the first completion
        }
        io_uring_cqe_seen(&ring, cqe);
    }
}

int DirectSubmissionMode()
{
    struct io_uring ring;
    setup_io_uring(ring);

    const char* filename = "samples/io_uring_example.txt";
    int fd               = open(filename, O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "Failed to open file: " << strerror(errno) << std::endl;
        return 1;
    }

    const size_t buffer_size = 1024;
    char buffer[buffer_size];
    submit_read_request(ring, fd, buffer, buffer_size);

    wait_for_completion(ring);

    close(fd);
    io_uring_queue_exit(&ring);

    return 0;
}

int KernelPollingMode()
{
    struct io_uring ring;
    struct io_uring_params params;
    memset(&params, 0, sizeof(params));
    params.flags         = IORING_SETUP_SQPOLL;
    params.sq_thread_cpu = 2;  // 绑定到CPU 2运行，CPU可以根据需要调整
    params.sq_thread_idle = 1000;  // 线程空闲时间，单位为毫秒

    if (io_uring_queue_init_params(QUEUE_DEPTH, &ring, &params) < 0)
    {
        perror("io_uring_queue_init_params");
        return 1;
    }

    int fd = open("samples/io_uring_example.txt", O_RDONLY);
    if (fd < 0)
    {
        perror("open");
        return 1;
    }

    char* buf = (char*)malloc(BUFFER_SIZE);
    if (!buf)
    {
        perror("malloc");
        close(fd);
        return 1;
    }

    struct io_uring_sqe* sqe = io_uring_get_sqe(&ring);
    if (!sqe)
    {
        fprintf(stderr, "io_uring_get_sqe failed\n");
        free(buf);
        close(fd);
        return 1;
    }

    io_uring_prep_read(sqe, fd, buf, BUFFER_SIZE, 0);

    if (io_uring_submit(&ring) < 0)
    {
        perror("io_uring_submit");
        free(buf);
        close(fd);
        return 1;
    }

    struct io_uring_cqe* cqe;
    if (io_uring_wait_cqe(&ring, &cqe) < 0)
    {
        perror("io_uring_wait_cqe");
        free(buf);
        close(fd);
        return 1;
    }

    if (cqe->res < 0)
    {
        fprintf(stderr, "io_uring_read failed: %s\n", strerror(-cqe->res));
    }
    else
    {
        printf("Read %d bytes: %.*s\n", cqe->res, cqe->res, buf);
    }

    io_uring_cqe_seen(&ring, cqe);
    free(buf);
    close(fd);
    io_uring_queue_exit(&ring);
    return 0;
}

int main()
{
    DirectSubmissionMode();
    KernelPollingMode();
    return 0;
}