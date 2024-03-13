#include <iostream>
#include <coroutine>

template <class T>
struct generator
{
    struct promise_type
    {
        T _current_value;
        static auto get_return_object_on_allocation_failure()
        {
            return generator<T>{nullptr};
        }
        auto get_return_object()
        {
            return generator<T>{handle::from_promise(*this)};
        }
        auto initial_suspend()
        {
            return std::suspend_always{};
        }
        auto final_suspend() noexcept
        {
            std::cout << "final_suspend" << std::endl;
            return std::suspend_always{};
        }
        void unhandled_exception()
        {
            std::terminate();
        }
        // void return_void() {}
        void return_value(T value)
        {
            _current_value = value;
        }
        auto yield_value(T value)
        {
            _current_value = value;
            return std::suspend_always{};
        }
    };
    using handle                = std::coroutine_handle<promise_type>;
    generator(generator const&) = delete;
    generator(generator&& rhs)
        : coro(rhs.coro)
    {
        rhs.coro = nullptr;
    }
    ~generator()
    {
        if (coro)
            coro.destroy();
    }
    bool move_next()
    {
        return coro ? (coro.resume(), !coro.done()) : false;
    }
    bool resume()
    {
        coro.resume();
        return false;
    }
    T current_value()
    {
        return coro.promise()._current_value;
    }

private:
    generator(handle h)
        : coro(h)
    { }
    handle coro;
};

generator<int> f()
{
    co_yield 1;
    co_yield 5;
    co_return 11;
}
using namespace std;

struct lazy
{
    struct promise_type;
    using handle = std::coroutine_handle<promise_type>;
    struct promise_type
    {
        int _return_value;
        static auto get_return_object_on_allocation_failure()
        {
            return lazy{nullptr};
        }
        auto get_return_object()
        {
            return lazy{handle::from_promise(*this)};
        }
        auto initial_suspend()
        {
            return std::suspend_always{};
        }
        auto final_suspend() noexcept
        {
            return std::suspend_always{};
        }
        void unhandled_exception()
        {
            std::terminate();
        }
        void return_value(int value)
        {
            cout << "before return_value" << std::endl;
            _return_value = value;
            cout << "after return_value" << std::endl;
        }
    };
    bool calculate()
    {
        if (calculated)
            return true;
        if (!coro)
            return false;
        cout << "before resume" << std::endl;
        coro.resume();
        cout << "after resume" << std::endl;
        if (coro.done())
            calculated = true;
        return calculated;
    }
    int get()
    {
        return coro.promise()._return_value;
    }
    lazy(lazy const&) = delete;
    lazy(lazy&& rhs)
        : coro(rhs.coro)
    {
        rhs.coro = nullptr;
    }
    ~lazy()
    {
        if (coro)
            coro.destroy();
    }

private:
    lazy(handle h)
        : coro(h)
    { }
    handle coro;
    bool calculated{false};
};

lazy f2(int n = 0)
{
    cout << "in f" << std::endl;
    co_return n + 1;
}

#include <chrono>
#include <coroutine>
#include <thread>
using namespace std;

class tt
{
public:
    constexpr bool await_ready() const noexcept
    {
        return false;
    }

    constexpr auto await_suspend(coroutine_handle<> coro) const noexcept
    {
        // return coro;
        return;
    }

    void await_resume() const noexcept
    {
        std::cout << "aa" << std::endl;
    }
};

struct my_future
{
    struct promise_type;
    using handle = std::coroutine_handle<promise_type>;

    struct promise_type
    {
        int current_value;
        static auto get_return_object_on_allocation_failure()
        {
            return my_future{nullptr};
        }
        auto get_return_object()
        {
            return my_future{handle::from_promise(*this)};
        }
        // auto initial_suspend() { return std::suspend_always{}; }
        auto initial_suspend()
        {
            return std::suspend_always{};
        }
        auto final_suspend() noexcept
        {
            return std::suspend_always{};
        }
        void unhandled_exception()
        {
            std::terminate();
        }

        void return_value(int value)
        {
            current_value = value;
        }
        auto yield_value(int value)
        {
            current_value = value;
            return std::suspend_always{};
        }
    };
    bool move_next()
    {
        return coro ? (coro.resume(), !coro.done()) : false;
    }
    int current_value()
    {
        return coro.promise().current_value;
    }
    void resume()
    {
        coro.resume();
    };
    void destroy()
    {
        coro.destroy();
    };
    my_future(my_future const&) = delete;
    my_future(my_future&& rhs)
        : coro(rhs.coro)
    {
        rhs.coro = nullptr;
    }
    ~my_future()
    {
        if (coro)
            coro.destroy();
    }

private:
    my_future(handle h)
        : coro(h)
    { }
    handle coro;
};

template <class Rep, class Period>
auto operator co_await(std::chrono::duration<Rep, Period> d)
{
    struct awaiter
    {
        std::chrono::system_clock::duration duration;
        awaiter(std::chrono::system_clock::duration d)
            : duration(d)
        { }
        bool await_ready() const
        {
            cout << duration.count() << std::endl;
            return duration.count() <= 0;
        }
        int await_resume()
        {
            std::cout << "await_resume" << std::endl;
            return 1;
        }
        void await_suspend(std::coroutine_handle<> h)
        {
            std::this_thread::sleep_for(duration);
            std::cout << "leave await_suspend" << std::endl;
            //   h.resume();
        }
    };
    return awaiter{d};
}

my_future te()
{
    co_return 1;
}

my_future sleep_coro()
{
    // auto aaa = te();
    // aaa.resume();
    co_await tt();
    printf("Start sleeping\n");
    //   int ans = co_await - 1s;
    //   printf("before seconde co_await value %d\n", ans);
    int ans = co_await 2s;
    std::cout << "leave sleep_coro " << ans << std::endl;
}

// Example code for blog post 'Understanding Awaitables'
//
// Copyright (c) Lewis Baker

#include <atomic>

class async_manual_reset_event
{
public:
    async_manual_reset_event(bool initiallySet = false) noexcept;

    // No copying/moving
    async_manual_reset_event(const async_manual_reset_event&) = delete;
    async_manual_reset_event(async_manual_reset_event&&)      = delete;
    async_manual_reset_event& operator=(const async_manual_reset_event&)
        = delete;
    async_manual_reset_event& operator=(async_manual_reset_event&&) = delete;

    bool is_set() const noexcept;

    struct awaiter;
    awaiter operator co_await() const noexcept;

    void set() noexcept;
    void reset() noexcept;

private:
    friend struct awaiter;

    // - 'this' => set state
    // - otherwise => not set, head of linked list of awaiter*.
    mutable std::atomic<void*> m_state;
};

struct async_manual_reset_event::awaiter
{
    awaiter(const async_manual_reset_event& event) noexcept
        : m_event(event)
    { }

    bool await_ready() const noexcept;
    bool await_suspend(std::coroutine_handle<> awaitingCoroutine) noexcept;
    void await_resume() noexcept
    { }

private:
    friend struct async_manual_reset_event;

    const async_manual_reset_event& m_event;
    std::coroutine_handle<> m_awaitingCoroutine;
    awaiter* m_next;
};

bool async_manual_reset_event::awaiter::await_ready() const noexcept
{
    return m_event.is_set();
}

bool async_manual_reset_event::awaiter::await_suspend(
    std::coroutine_handle<> awaitingCoroutine) noexcept
{
    // Special m_state value that indicates the event is in the 'set' state.
    const void* const setState = &m_event;

    // Stash the handle of the awaiting coroutine.
    m_awaitingCoroutine = awaitingCoroutine;

    // Try to atomically push this awaiter onto the front of the list.
    void* oldValue = m_event.m_state.load(std::memory_order_acquire);
    do
    {
        // Resume immediately if already in 'set' state.
        if (oldValue == setState)
            return false;

        // Update linked list to point at current head.
        m_next = static_cast<awaiter*>(oldValue);

        // Finally, try to swap the old list head, inserting this awaiter
        // as the new list head.
    } while (!m_event.m_state.compare_exchange_weak(
        oldValue, this, std::memory_order_release, std::memory_order_acquire));

    // Successfully enqueued. Remain suspended.
    return true;
}

async_manual_reset_event::async_manual_reset_event(bool initiallySet) noexcept
    : m_state(initiallySet ? this : nullptr)
{ }

bool async_manual_reset_event::is_set() const noexcept
{
    return m_state.load(std::memory_order_acquire) == this;
}

void async_manual_reset_event::reset() noexcept
{
    void* oldValue = this;
    m_state.compare_exchange_strong(
        oldValue, nullptr, std::memory_order_acquire);
}

void async_manual_reset_event::set() noexcept
{
    // Needs to be 'release' so that subsequent 'co_await' has
    // visibility of our prior writes.
    // Needs to be 'acquire' so that we have visibility of prior
    // writes by awaiting coroutines.
    void* oldValue = m_state.exchange(this, std::memory_order_acq_rel);
    if (oldValue != this)
    {
        // Wasn't already in 'set' state.
        // Treat old value as head of a linked-list of waiters
        // which we have now acquired and need to resume.
        auto* waiters = static_cast<awaiter*>(oldValue);
        while (waiters != nullptr)
        {
            // Read m_next before resuming the coroutine as resuming
            // the coroutine will likely destroy the awaiter object.
            auto* next = waiters->m_next;
            waiters->m_awaitingCoroutine.resume();
            waiters = next;
        }
    }
}

async_manual_reset_event::awaiter async_manual_reset_event::operator co_await()
    const noexcept
{
    return awaiter{*this};
}

// A simple task-class for void-returning coroutines.
struct task
{
    struct promise_type
    {
        task get_return_object()
        {
            return {};
        }
        std::suspend_never initial_suspend()
        {
            return {};
        }
        std::suspend_never final_suspend() noexcept
        {
            return {};
        }
        void return_void()
        { }
        void unhandled_exception()
        { }
    };
};

task example(async_manual_reset_event& event)
{
    co_await event;
    std::cout << "end"
              << std::endl;  //协程暂停了，没有运行到这里，主线程就结束了
}

#include <atomic>
#include <condition_variable>
#include <coroutine>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <stdexcept>
#include <thread>

template <typename T>
struct threadsafe_queue_t
{
    threadsafe_queue_t()
    { }

    void put(T task)
    {
        std::unique_lock<std::mutex> lk(m_m);
        m_queue.emplace(task);
        m_cv.notify_one();
    }

    std::optional<T> take()
    {
        std::unique_lock<std::mutex> lk(m_m);
        m_cv.wait(lk, [q = this] {
            return q->m_must_return_nullptr.test() || !q->m_queue.empty();
        });

        if (m_must_return_nullptr.test())
            return {};

        T ret = m_queue.front();
        m_queue.pop();

        return ret;
    }

    void destroy()
    {
        m_must_return_nullptr.test_and_set();
        m_cv.notify_all();
    }

private:
    std::queue<T> m_queue;
    std::mutex m_m;
    std::condition_variable m_cv;

    std::atomic_flag m_must_return_nullptr = false;
};

struct thread_pool_t
{
    struct promise_type;
    struct awaitable
    {
        bool await_ready()
        {
            return false;
        }
        void await_suspend(std::coroutine_handle<> h)
        {
            m_thread_pool->m_queue.put(h);
        }
        void await_resume()
        { }

        thread_pool_t* m_thread_pool;
    };

    struct wrapper
    {
        struct promise_type
        {
            wrapper get_return_object()
            {
                return {};
            }
            std::suspend_never initial_suspend()
            {
                return {};
            }
            std::suspend_never final_suspend() noexcept
            {
                return {};
            }
            void return_void()
            { }
            void unhandled_exception()
            { }

            promise_type()
            { }
        };
    };

    thread_pool_t(int thread_number)
    {
        for (int i = 0; i < thread_number; ++i)
        {
            m_threads.emplace(std::jthread([this] {
                this->worker();
            }));
        }
    }

    template <typename T>
    wrapper submit(T task)
    {
        co_await awaitable(this);

        task();
    }

    void stop()
    {
        m_queue.destroy();
        while (!m_threads.empty())
        {
            m_threads.pop();
        }
    }

    threadsafe_queue_t<std::coroutine_handle<>> m_queue;

private:
    void worker()
    {
        while (auto task = m_queue.take())
        {
            task.value().resume();
        }
    }

    std::queue<std::jthread> m_threads;
};

#include <coroutine>
#include <iostream>
#include <utility>

template <class T>
struct task2
{
    struct promise_type;
    using handle = std::coroutine_handle<promise_type>;

    struct promise_type
    {
        auto get_return_object()
        {
            return task2(handle::from_promise(*this));
        }
        auto initial_suspend()
        {
            return std::suspend_always{};
        }
        struct final_awaiter
        {
            bool await_ready() noexcept
            {
                return false;
            }
            void await_resume() noexcept
            { }
            std::coroutine_handle<> await_suspend(handle h) noexcept
            {
                auto previous = h.promise().previous;
                if (previous)
                {
                    return previous;
                }
                else
                {
                    return std::noop_coroutine();
                }
            }
        };
        auto final_suspend() noexcept
        {
            return final_awaiter{};
        }
        void unhandled_exception()
        {
            throw;
        }
        void return_value(T value)
        {
            result = std::move(value);
        }
        T result;
        std::coroutine_handle<> previous;
    };

    task2(handle h)
        : coro(h)
    { }
    task2(task2&& t) = delete;
    ~task2()
    {
        coro.destroy();
    }

    struct awaiter
    {
        bool await_ready()
        {
            return false;
        }
        T await_resume()
        {
            return std::move(coro.promise().result);
        }
        auto await_suspend(std::coroutine_handle<> h)
        {
            coro.promise().previous = h;
            return coro;
        }
        handle coro;
    };
    awaiter operator co_await()
    {
        return awaiter{coro};
    }
    T operator()()
    {
        coro.resume();
        return std::move(coro.promise().result);
    }

private:
    handle coro;
};

task2<int> get_random()
{
    std::cout << "in get_random()\n";
    co_return 3;
}

task2<int> test()
{
    task2<int> v = get_random();
    task2<int> u = get_random();
    std::cout << "in test()\n";
    int x = (co_await v + co_await u);
    co_return x;
}

void testcoroutine()
{
    {
        task2<int> t = test();
        int result   = t();
        std::cout << result << '\n';
    }
    {
        auto g = f();
        while (g.move_next())
            std::cout << g.current_value() << std::endl;
        std::cout << g.current_value() << std::endl;
    }
    cout << "second" << std::endl;
    {
        auto g = f2();
        g.calculate();  // 这时才从 initial_suspend 之中恢复, 所以就叫 lazy 了
        cout << g.get() << std::endl;
    }
    cout << "third" << std::endl;
    {
        auto h = sleep_coro();
        h.resume();
        cout << "first resume" << std::endl;
        h.resume();
        cout << "second resume" << std::endl;
    }
    {
        async_manual_reset_event e;
        example(e);
    }
    {
        using namespace std::chrono_literals;
        auto tpool = thread_pool_t(1);
        for (int i = 0; i < 1; ++i)
        {
            tpool.submit([i] {
                std::this_thread::sleep_for(5s);
                std::cout << "* Task " << i << std::endl;
            });
        }
        std::this_thread::sleep_for(10s);
        tpool.stop();
    }
}

int main()
{
    testcoroutine();
}