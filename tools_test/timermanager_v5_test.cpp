#include "tools/timermanager.hpp"

#include <atomic>
#include <chrono>
#include <thread>

int main()
{
    using namespace std::chrono_literals;

    timermanager::v5::TimerManager timer_manager;
    std::atomic<int> one_shot_count{0};
    std::atomic<int> cancelled_count{0};
    std::atomic<int> repeating_count{0};
    std::atomic<int> cascaded_count{0};
    std::atomic<timermanager::v5::TimerManager::TimerId> repeating_id{0};

    timer_manager.AddTimer(2ms, [&] { ++one_shot_count; });
    const auto cancelled_id = timer_manager.AddTimer(20ms, [&] { ++cancelled_count; });
    if (!timer_manager.CancelTimer(cancelled_id))
        return 1;

    const auto timer_id = timer_manager.AddTimer(1ms,
        [&] {
            if (++repeating_count == 3)
                timer_manager.CancelTimer(repeating_id.load());
        },
        1ms);
    repeating_id.store(timer_id);
    timer_manager.AddTimer(70ms, [&] { ++cascaded_count; });

    std::this_thread::sleep_for(100ms);
    timer_manager.StopTimerManager();

    return one_shot_count == 1 && cancelled_count == 0 && repeating_count == 3
            && cascaded_count == 1
        ? 0
        : 1;
}
