#pragma once
#include <coroutine>
#include <type_traits>
#include <utility>
#include <variant>
#include <chrono>
#include "tools/timermanager.hpp"
#include "custom_awaiter.hpp"

namespace coroutine_task
{
struct DetachedCoroutine
{
    struct promise_type
    {
        std::suspend_never initial_suspend() noexcept
        {
            return {};
        }
        std::suspend_never final_suspend() noexcept
        {
            return {};
        }
        void return_void() noexcept
        { }
        void unhandled_exception()
        {
            try
            {
                std::rethrow_exception(std::current_exception());
            } catch (const std::exception& e)
            {
                fprintf(stderr, "find exception %s\n", e.what());
                fflush(stderr);
                std::rethrow_exception(std::current_exception());
            }
        }
        DetachedCoroutine get_return_object() noexcept
        {
            return DetachedCoroutine();
        }
    };
};

template <typename T>
class LazyTask;

class PromiseTypeBase
{
public:
    struct FinalAwaiter
    {
        bool await_ready() const noexcept
        {
            return false;
        }
        template <typename PromiseType>
        auto await_suspend(std::coroutine_handle<PromiseType> h) noexcept
        {
            return h.promise()._continuation;
        }
        void await_resume() noexcept
        { }
    };

    template <typename T>
    struct ValueAwaiter
    {
        bool await_ready() const noexcept
        {
            return false;
        }
        auto await_suspend(std::coroutine_handle<> h) noexcept
        {
            _coro.promise()._continuation = h;
            timermanager::v3::TimerManager<uint8_t>::GetInstance()->AddAlarm(
                std::chrono::seconds(0),
                0,
                "",
                [this, core = std::move(_coro)]() {
                    if (!core.done())
                        core.resume();
                });
        }

        auto await_resume() noexcept
        {
            if constexpr (std::is_same_v<T, void>)
            {
                _coro.destroy();
                _coro = nullptr;
                return AnyWrapper<void>();
            }
            else
            {
                auto tmp = std::get<T>(_coro.promise()._value);
                _coro.destroy();
                _coro = nullptr;
                return AnyWrapper<T>(tmp);
            }
        }

        std::coroutine_handle<typename LazyTask<T>::promise_type> _coro;
    };

    std::suspend_always initial_suspend() noexcept
    {
        return {};
    }
    FinalAwaiter final_suspend() noexcept
    {
        return {};
    }
    void unhandled_exception()
    { }

    // template <typename T>
    // auto await_transform(LazyTask<T>&& task)
    // {
    //     return ValueAwaiter<T>{std::exchange(task._coro, nullptr)};
    // }

    // template <typename Awaiter>
    // auto await_transform(Awaiter&& awaiter)
    // {
    //     return awaiter;
    // }

public:
    std::coroutine_handle<> _continuation = std::noop_coroutine();
};

template <typename T>
class PromiseType : public PromiseTypeBase
{
public:
    LazyTask<T> get_return_object() noexcept;
    void return_value(const T& value)
    {
        _value = value;
    }

    std::variant<std::monostate, T> _value;
};

template <>
class PromiseType<void> : public PromiseTypeBase
{
public:
    LazyTask<void> get_return_object() noexcept;
    void return_void() noexcept
    { }
};

class VoidType
{ };

template <typename T>
class LazyTask
{
public:
    using TYPE = T;
    // using TYPE = std::conditional<std::is_same_v<T, void>, VoidType, T>;
    using promise_type = PromiseType<TYPE>;
    explicit LazyTask(std::coroutine_handle<promise_type> coro)
        : _coro(coro)
    { }
    LazyTask(LazyTask&& other)
        : _coro(std::move(other._coro))
    {
        other._coro = nullptr;
    }
    LazyTask(const LazyTask&)            = delete;
    LazyTask& operator=(const LazyTask&) = delete;
    ~LazyTask()
    {
        if (_coro)
        {
            _coro.destroy();
            _coro = nullptr;
        }
    }
    void Resume()
    {
        _coro.resume();
    }
    auto GetValue()
    {
        if constexpr (std::is_same_v<T, void>)
            return;
        else
            return std::get<TYPE>(_coro.promise()._value);
    }

    auto operator co_await()
    {
        return typename promise_type::template ValueAwaiter<T>{
            std::exchange(_coro, nullptr)};
    }
    template <typename F>
    void AsyncStart2(F&& fun)
    {
        auto launchCoro
            = [](LazyTask lazy, std::decay_t<F> cb) -> DetachedCoroutine {
            cb(co_await lazy);
        };
        launchCoro(std::move(*this), std::forward<F>(fun));
    }

    bool IsDone()
    {
        return !_coro || _coro.done();
    }

public:
    std::coroutine_handle<promise_type> _coro;
};

template <typename T>
inline LazyTask<T> coroutine_task::PromiseType<T>::get_return_object() noexcept
{
    return LazyTask<T>{
        std::coroutine_handle<typename LazyTask<T>::promise_type>::from_promise(
            *this)};
}

inline LazyTask<void>
coroutine_task::PromiseType<void>::get_return_object() noexcept
{
    return LazyTask<void>{std::coroutine_handle<
        typename LazyTask<void>::promise_type>::from_promise(*this)};
}
};  // namespace coroutine_task