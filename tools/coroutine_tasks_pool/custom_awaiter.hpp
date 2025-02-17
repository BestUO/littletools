#pragma once
#include <coroutine>
#include <memory>
#include <variant>
#include <chrono>
#include <iostream>
#include "tools/timermanager.hpp"
#include "any_wrapper.hpp"

struct SleepAwaiter
{
    bool await_ready() const noexcept
    {
        return false;
    }
    auto await_suspend(std::coroutine_handle<> h) noexcept
    {
        timermanager::v3::TimerManager<uint8_t>::GetInstance()->AddAlarm(
            sleep_time, 0, "", [h = h]() -> void {
                h.resume();
            });
    }
    void await_resume() noexcept
    { }
    std::chrono::milliseconds sleep_time = std::chrono::milliseconds(0);
};

class EventCount
{
public:
    EventCount(size_t all)
        : __all(all)
    { }
    bool AnyDone()
    {
        return __done.fetch_add(1) == 0;
    }

    bool AllDone()
    {
        return __done.fetch_add(1) == __all - 1;
    }

private:
    std::atomic<size_t> __all  = 0;
    std::atomic<size_t> __done = 0;
};

template <template <typename> typename LazyTaskType, typename... Ts>
struct CollectAnyAwaiter
{
    using InputType = std::tuple<LazyTaskType<Ts>...>;
    CollectAnyAwaiter(LazyTaskType<Ts>&&... inputs)
        : __input(std::make_unique<InputType>(std::move(inputs)...))
    { }
    bool await_ready() const noexcept
    {
        return false;
    }
    void await_suspend(std::coroutine_handle<> h) noexcept
    {
        await_suspend_impl(
            std::make_index_sequence<sizeof...(Ts)>{}, std::move(h));
    }
    auto await_resume() noexcept
    {
        return *__result;
    }

    template <size_t... index>
    void await_suspend_impl(std::index_sequence<index...>,
        std::coroutine_handle<> continuation)
    {
        auto event_count = std::make_shared<EventCount>(sizeof...(Ts));
        __result         = std::make_shared<std::variant<AnyWrapper<Ts>...>>();
        auto input       = std::move(__input);
        (
            [&]() {
                if (continuation.done())
                    return;
                std::move(std::get<index>(*input))
                    .AsyncStart2([result          = __result,
                                     event_count  = event_count,
                                     continuation = continuation](auto i) {
                        if (event_count->AnyDone())
                        {
                            *result = i;
                            continuation.resume();
                        }
                    });
            }(),
            ...);
    }

private:
    std::unique_ptr<InputType> __input = {};
    std::shared_ptr<std::variant<AnyWrapper<Ts>...>> __result;
};

template <template <typename> typename LazyTaskType, typename... Ts>
struct CollectAllAwaiter
{
    using InputType = std::tuple<LazyTaskType<Ts>...>;
    CollectAllAwaiter(LazyTaskType<Ts>&&... inputs)
        : __input(std::move(inputs)...)
    { }
    bool await_ready() const noexcept
    {
        return false;
    }

    void await_suspend(std::coroutine_handle<> h) noexcept
    {
        await_suspend_impl(
            std::make_index_sequence<sizeof...(Ts)>{}, std::move(h));
    }
    auto await_resume() noexcept
    {
        return __results;
    }

    template <size_t... index>
    void await_suspend_impl(std::index_sequence<index...>,
        std::coroutine_handle<> continuation)
    {
        auto event_count = std::make_shared<EventCount>(sizeof...(Ts));
        (
            [&]() {
                if (continuation.done())
                    return;
                std::move(std::get<index>(__input))
                    .AsyncStart2([&result         = std::get<index>(__results),
                                     event_count  = event_count,
                                     continuation = continuation](auto i) {
                        result = i;
                        if (event_count->AllDone())
                        {
                            continuation.resume();
                        }
                    });
            }(),
            ...);
    }

private:
    InputType __input;
    std::tuple<AnyWrapper<Ts>...> __results;
};

// template <template <typename> typename LazyTaskType, typename... Ts>
// struct CollectAllAwaiter
// {
//     using InputType = std::tuple<LazyTaskType<Ts>...>;
//     CollectAllAwaiter(LazyTaskType<Ts>&&... inputs)
//         : __input(std::make_unique<InputType>(std::move(inputs)...))
//     { }
//     bool await_ready() const noexcept
//     {
//         return false;
//     }
//     void await_suspend(std::coroutine_handle<> h) noexcept
//     {
//         await_suspend_impl(
//             std::make_index_sequence<sizeof...(Ts)>{}, std::move(h));
//     }
//     auto await_resume() noexcept
//     {
//         return *__result;
//     }

//     template <size_t... index>
//     void await_suspend_impl(std::index_sequence<index...>,
//         std::coroutine_handle<> continuation)
//     {
//         auto event_count = std::make_shared<EventCount>(sizeof...(Ts));
//         __result         = std::make_shared<std::any>();
//         auto input       = std::move(__input);
//         (
//             [&]() {
//                 if (continuation.done())
//                     return;
//                 std::move(std::get<index>(*input))
//                     .AsyncStart2([event_count     = event_count,
//                                      result       = __result,
//                                      continuation = continuation](auto i) {
//                         if (event_count->AnyDone())
//                         {
//                             *result = i;
//                             continuation.resume();
//                         }
//                     });
//             }(),
//             ...);
//     }

// private:
//     std::unique_ptr<InputType> __input = {};
//     std::shared_ptr<std::any> __result;
// };