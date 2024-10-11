#pragma once
#include <atomic>
#include <condition_variable>
#include <mutex>
#include "tools/uuid.hpp"
#include "tools/timermanager.hpp"

struct FlowControl
{
public:
    FlowControl(uint32_t band_width)
        : __flow_size(band_width)
    {
        timermanager::TimerManager<UUID>::GetInstance()->AddAlarm(
            std::chrono::milliseconds(1000),
            __id,
            "FlowControl",
            [this, band_width]() {
                __flow_size = band_width;
                __cv.notify_one();
            },
            std::chrono::milliseconds(1000));
    }

    ~FlowControl()
    {
        timermanager::TimerManager<UUID>::GetInstance()->DeleteAlarm(
            __id, "FlowControl");
    }

    void Wait(uint32_t wait_size)
    {
        std::unique_lock<std::mutex> lock(__mutex);
        __cv.wait(lock, [this, wait_size = wait_size] {
            return __flow_size >= wait_size;
        });
        __flow_size -= wait_size;
    }

private:
    UUID __id = UUID::gen();
    std::atomic<uint32_t> __flow_size;
    std::condition_variable __cv;
    std::mutex __mutex;
};