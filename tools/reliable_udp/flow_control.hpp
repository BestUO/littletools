#pragma once
#include <atomic>
#include <condition_variable>
#include <mutex>

template <int MIN_SIZE, int BAND_WIDTH>
struct FlowControl
{
public:
    FlowControl()
        : __flow_size(BAND_WIDTH){};

    void Wait(uint32_t wait_size)
    {
        std::unique_lock<std::mutex> lock(__mutex);
        __cv.wait(lock, [this, wait_size = wait_size] {
            return __flow_size >= wait_size;
        });
        __flow_size -= wait_size;
    }

public:  // only for test
    void Increase(uint32_t flow_size)
    {
        __flow_size += flow_size;
        __cv.notify_one();
    }

    void Decrease(uint32_t flow_size)
    {
        __flow_size -= flow_size;
    }

    bool Pass()
    {
        return __flow_size >= MIN_SIZE;
    }

private:
    std::atomic<uint32_t> __flow_size;
    std::condition_variable __cv;
    std::mutex __mutex;
};