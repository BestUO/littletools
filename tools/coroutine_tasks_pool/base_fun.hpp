#pragma once
#include <coroutine>
#include <variant>
#include <chrono>
#include "custom_awaiter.hpp"
#include "lazy_task.hpp"

namespace coroutine_task
{
inline LazyTask<void> CoroutineSleep(std::chrono::milliseconds sleep_time)
{
    co_await SleepAwaiter{sleep_time};
    co_return;
};

template <template <typename> typename LazyTaskType, typename... Ts>
inline LazyTask<std::variant<AnyWrapper<Ts>...>> CoroutineCollectAny(
    LazyTaskType<Ts>&&... tasks)
{
    co_return co_await CollectAnyAwaiter(std::move(tasks)...);
};

template <template <typename> typename LazyTaskType, typename... Ts>
inline LazyTask<std::tuple<AnyWrapper<Ts>...>> CoroutineCollectAll(
    LazyTaskType<Ts>&&... tasks)
{
    co_return co_await CollectAllAwaiter(std::move(tasks)...);
};

}  // namespace coroutine_task