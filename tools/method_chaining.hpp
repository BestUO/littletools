#include <iostream>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include "tools/timermanager.hpp"
#include "tools/function_traits.hpp"
#include "tools/uuid.hpp"
#include "tools/coroutine_tasks_pool/any_wrapper.hpp"

template <typename T>
class Thenable : public std::enable_shared_from_this<Thenable<T>>
{
public:
    Thenable(std::future<T>&& future)
        : future_(std::move(future))
    { }

    template <typename F>
    auto ThenOrderInWorkPool(F&& func,
        std::shared_ptr<std::promise<typename function_traits<F>::return_type>>
            promiseptr
        = nullptr)
        -> std::shared_ptr<Thenable<typename function_traits<F>::return_type>>
    {
        using ReturnType = typename function_traits<F>::return_type;
        auto promise     = promiseptr
                ? promiseptr
                : std::make_shared<std::promise<ReturnType>>();
        if (future_.wait_for(std::chrono::seconds(0))
            == std::future_status::ready)
        {
            timermanager::v4::TimerManager<UUID>::GetInstance()->AddAlarm(
                std::chrono::milliseconds(0),
                UUID::gen(),
                "",
                [self = this->shared_from_this(), func, promise]() mutable {
                    if constexpr (std::is_same_v<ReturnType, void>)
                    {
                        if constexpr (function_traits<F>::arity == 0)
                            func();
                        else
                            func(self->future_.get());

                        promise->set_value();
                    }
                    else
                    {
                        if constexpr (function_traits<F>::arity == 0)
                            promise->set_value(func());
                        else
                            promise->set_value(func(self->future_.get()));
                    }
                });
        }
        else
        {
            timermanager::v4::TimerManager<UUID>::GetInstance()->AddAlarm(
                std::chrono::milliseconds(50),
                UUID::gen(),
                "",
                [self = this->shared_from_this(), func, promise]() mutable {
                    self->ThenOrderInWorkPool(std::forward<F>(func), promise);
                });
        }

        if (!promiseptr)
        {
            auto future = promise->get_future();
            return std::make_shared<Thenable<ReturnType>>(std::move(future));
        }
        else
            return nullptr;
    }

    template <typename F>
    auto ThenOrder(F&& func,
        std::shared_ptr<std::promise<typename function_traits<F>::return_type>>
            promiseptr
        = nullptr)
        -> std::shared_ptr<Thenable<typename function_traits<F>::return_type>>
    {
        using ReturnType = typename function_traits<F>::return_type;
        auto promise     = promiseptr
                ? promiseptr
                : std::make_shared<std::promise<ReturnType>>();
        if (future_.wait_for(std::chrono::seconds(0))
            == std::future_status::ready)
        {
            if constexpr (std::is_same_v<ReturnType, void>)
            {
                if constexpr (function_traits<F>::arity == 0)
                    func();
                else
                    func(future_.get());

                promise->set_value();
            }
            else
            {
                if constexpr (function_traits<F>::arity == 0)
                    promise->set_value(func());
                else
                    promise->set_value(func(future_.get()));
            }
        }
        else
        {
            timermanager::v4::TimerManager<UUID>::GetInstance()->AddAlarm(
                std::chrono::milliseconds(50),
                UUID::gen(),
                "",
                [self = this->shared_from_this(), func, promise]() mutable {
                    self->ThenOrder(std::forward<F>(func), promise);
                });
        }

        if (!promiseptr)
        {
            auto future = promise->get_future();
            return std::make_shared<Thenable<ReturnType>>(std::move(future));
        }
        else
            return nullptr;
    }

    template <typename F>
    [[deprecated]] auto ThenTryBest(F&& func,
        std::shared_ptr<std::promise<typename function_traits<F>::return_type>>
            promiseptr
        = nullptr)
        -> std::shared_ptr<Thenable<typename function_traits<F>::return_type>>
    {
        using ReturnType = typename function_traits<F>::return_type;
        auto promise     = promiseptr
                ? promiseptr
                : std::make_shared<std::promise<ReturnType>>();
        if constexpr (function_traits<F>::arity == 0)
        {
            if constexpr (std::is_same_v<ReturnType, void>)
            {
                func();
                promise->set_value();
            }
            else
                promise->set_value(func());
        }
        else
        {
            if (future_.wait_for(std::chrono::seconds(0))
                == std::future_status::ready)
            {
                if constexpr (std::is_same_v<ReturnType, void>)
                {
                    func(future_.get());
                    promise->set_value();
                }
                else
                    promise->set_value(func(future_.get()));
            }
            else
            {
                timermanager::v4::TimerManager<UUID>::GetInstance()->AddAlarm(
                    std::chrono::milliseconds(50),
                    UUID::gen(),
                    "",
                    [self = this->shared_from_this(), func, promise]() mutable {
                        self->ThenTryBest(std::forward<F>(func), promise);
                    });
            }
        }

        if (!promiseptr)
        {
            auto future = promise->get_future();
            return std::make_shared<Thenable<ReturnType>>(std::move(future));
        }
        else
            return nullptr;
    }

private:
    std::future<T> future_;
};

template <typename F, typename... Args>
auto MakeThenOrder(F&& func, Args&&... args)
{
    std::promise<decltype(func(args...))> promise;
    std::future<decltype(func(args...))> future = promise.get_future();
    promise.set_value(func(args...));
    return std::make_shared<Thenable<decltype(func(args...))>>(
        std::move(future));
}