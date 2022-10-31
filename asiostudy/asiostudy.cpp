#include <asio/co_spawn.hpp>
#include <asio/detached.hpp>
#include <asio/io_context.hpp>
#include <asio/ip/tcp.hpp>
#include <asio/signal_set.hpp>
#include <asio/write.hpp>
#include <cstdio>

using asio::ip::tcp;
using asio::awaitable;
using asio::co_spawn;
using asio::detached;
using asio::use_awaitable;
namespace this_coro = asio::this_coro;

#if defined(ASIO_ENABLE_HANDLER_TRACKING)
# define use_awaitable \
  asio::use_awaitable_t(__FILE__, __LINE__, __PRETTY_FUNCTION__)
#endif

awaitable<void> echo(tcp::socket socket)
{
  try
  {
    char data[1024];
    for (;;)
    {
      std::size_t n = co_await socket.async_read_some(asio::buffer(data), use_awaitable);
      co_await async_write(socket, asio::buffer(data, n), use_awaitable);
    }
  }
  catch (std::exception& e)
  {
    std::printf("echo Exception: %s\n", e.what());
  }
}

awaitable<void> listener()
{
  auto executor = co_await this_coro::executor;
  tcp::acceptor acceptor(executor, {tcp::v4(), 55555});
  for (;;)
  {
    tcp::socket socket = co_await acceptor.async_accept(use_awaitable);
    co_spawn(executor, echo(std::move(socket)), detached);
  }
}

int coroutinuec20()
{
  try
  {
    asio::io_context io_context(1);

    asio::signal_set signals(io_context, SIGINT, SIGTERM);
    signals.async_wait([&](auto, auto){ io_context.stop(); });

    co_spawn(io_context, listener(), detached);

    io_context.run();
  }
  catch (std::exception& e)
  {
    std::printf("Exception: %s\n", e.what());
  }
  return 0;
}

#include <asio/any_io_executor.hpp>
#include <asio/defer.hpp>
#include <asio/post.hpp>
#include <asio/strand.hpp>
#include <asio/system_executor.hpp>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <typeinfo>
#include <vector>

using asio::any_io_executor;
using asio::defer;
using asio::post;
using asio::strand;
using asio::system_executor;

//------------------------------------------------------------------------------
// A tiny actor framework
// ~~~~~~~~~~~~~~~~~~~~~~

class actor;

// Used to identify the sender and recipient of messages.
typedef actor* actor_address;

// Base class for all registered message handlers.
class message_handler_base
{
public:
  virtual ~message_handler_base() {}

  // Used to determine which message handlers receive an incoming message.
  virtual const std::type_info& message_id() const = 0;
};

// Base class for a handler for a specific message type.
template <class Message>
class message_handler : public message_handler_base
{
public:
  // Handle an incoming message.
  virtual void handle_message(Message msg, actor_address from) = 0;
};

// Concrete message handler for a specific message type.
template <class Actor, class Message>
class mf_message_handler : public message_handler<Message>
{
public:
  // Construct a message handler to invoke the specified member function.
  mf_message_handler(void (Actor::* mf)(Message, actor_address), Actor* a)
    : function_(mf), actor_(a)
  {
  }

  // Used to determine which message handlers receive an incoming message.
  virtual const std::type_info& message_id() const
  {
    return typeid(Message);
  }

  // Handle an incoming message.
  virtual void handle_message(Message msg, actor_address from)
  {
    (actor_->*function_)(std::move(msg), from);
  }

  // Determine whether the message handler represents the specified function.
  bool is_function(void (Actor::* mf)(Message, actor_address)) const
  {
    return mf == function_;
  }

private:
  void (Actor::* function_)(Message, actor_address);
  Actor* actor_;
};

// Base class for all actors.
class actor
{
public:
  virtual ~actor()
  {
  }

  // Obtain the actor's address for use as a message sender or recipient.
  actor_address address()
  {
    return this;
  }

  // Send a message from one actor to another.
  template <class Message>
  friend void send(Message msg, actor_address from, actor_address to)
  {
    // Execute the message handler in the context of the target's executor.
    post(to->executor_,
      [=, msg=std::move(msg)]() mutable
      {
        to->call_handler(std::move(msg), from);
      });
  }

protected:
  // Construct the actor to use the specified executor for all message handlers.
  actor(any_io_executor e)
    : executor_(std::move(e))
  {
  }

  // Register a handler for a specific message type. Duplicates are permitted.
  template <class Actor, class Message>
  void register_handler(void (Actor::* mf)(Message, actor_address))
  {
    handlers_.push_back(
      std::make_shared<mf_message_handler<Actor, Message>>(
        mf, static_cast<Actor*>(this)));
  }

  // Deregister a handler. Removes only the first matching handler.
  template <class Actor, class Message>
  void deregister_handler(void (Actor::* mf)(Message, actor_address))
  {
    const std::type_info& id = typeid(Message);
    for (auto iter = handlers_.begin(); iter != handlers_.end(); ++iter)
    {
      if ((*iter)->message_id() == id)
      {
        auto mh = static_cast<mf_message_handler<Actor, Message>*>(iter->get());
        if (mh->is_function(mf))
        {
          handlers_.erase(iter);
          return;
        }
      }
    }
  }

  // Send a message from within a message handler.
  template <class Message>
  void tail_send(Message msg, actor_address to)
  {
    // Execute the message handler in the context of the target's executor.
    defer(to->executor_,
      [=, msg=std::move(msg), from=this]
      {
        to->call_handler(std::move(msg), from);
      });
  }

private:
  // Find the matching message handlers, if any, and call them.
  template <class Message>
  void call_handler(Message msg, actor_address from)
  {
    const std::type_info& message_id = typeid(Message);
    for (auto& h: handlers_)
    {
      if (h->message_id() == message_id)
      {
        auto mh = static_cast<message_handler<Message>*>(h.get());
        mh->handle_message(msg, from);
      }
    }
  }

  // All messages associated with a single actor object should be processed
  // non-concurrently. We use a strand to ensure non-concurrent execution even
  // if the underlying executor may use multiple threads.
  strand<any_io_executor> executor_;

  std::vector<std::shared_ptr<message_handler_base>> handlers_;
};

// A concrete actor that allows synchronous message retrieval.
template <class Message>
class receiver : public actor
{
public:
  receiver()
    : actor(system_executor())
  {
    register_handler(&receiver::message_handler);
  }

  // Block until a message has been received.
  Message wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this]{ return !message_queue_.empty(); });
    Message msg(std::move(message_queue_.front()));
    message_queue_.pop_front();
    return msg;
  }

private:
  // Handle a new message by adding it to the queue and waking a waiter.
  void message_handler(Message msg, actor_address /* from */)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    message_queue_.push_back(std::move(msg));
    condition_.notify_one();
  }

  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Message> message_queue_;
};

//------------------------------------------------------------------------------

#include <asio/thread_pool.hpp>
#include <iostream>

using asio::thread_pool;

class member : public actor
{
public:
  explicit member(any_io_executor e)
    : actor(std::move(e))
  {
    register_handler(&member::init_handler);
  }

private:
  void init_handler(actor_address next, actor_address from)
  {
    next_ = next;
    caller_ = from;

    register_handler(&member::token_handler);
    deregister_handler(&member::init_handler);
  }

  void token_handler(int token, actor_address /*from*/)
  {
    int msg(token);
    actor_address to(caller_);

    if (token > 0)
    {
      msg = token - 1;
      to = next_;
    }

    tail_send(msg, to);
  }

  actor_address next_;
  actor_address caller_;
};

int actortest()
{
  const std::size_t num_threads = 16;
  const int num_hops = 50000000;
  const std::size_t num_actors = 503;
  const int token_value = (num_hops + num_actors - 1) / num_actors;
  const std::size_t actors_per_thread = num_actors / num_threads;

  struct single_thread_pool : thread_pool { single_thread_pool() : thread_pool(1) {} };
  single_thread_pool pools[num_threads];
  std::vector<std::shared_ptr<member>> members(num_actors);
  receiver<int> rcvr;

  // Create the member actors.
  for (std::size_t i = 0; i < num_actors; ++i)
    members[i] = std::make_shared<member>(pools[(i / actors_per_thread) % num_threads].get_executor());

  // Initialise the actors by passing each one the address of the next actor in the ring.
  for (std::size_t i = num_actors, next_i = 0; i > 0; next_i = --i)
    send(members[next_i]->address(), rcvr.address(), members[i - 1]->address());

  // Send exactly one token to each actor, all with the same initial value, rounding up if required.
  for (std::size_t i = 0; i < num_actors; ++i)
    send(token_value, rcvr.address(), members[i]->address());

  // Wait for all signal messages, indicating the tokens have all reached zero.
  for (std::size_t i = 0; i < num_actors; ++i)
    rcvr.wait();
    return 0;
}


#include <iostream>
#include <functional>
#include <thread>
#include <mutex>
#include <chrono>
#include <stdint.h>
#include "asio.hpp"

using namespace std;

// 单线程，为了主循环不阻塞，可以调用io_context::poll
void restartpoll()
{
    asio::io_context ios;

    uint16_t s = 0;
    while (++s)
    {
        ios.post([s]()
        {   cout<<s; cout.flush();});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ios.restart(); // 注意调restart，设置stopped_=false否则后面的回调都不会执行
        ios.poll();  // poll不会阻塞，可以在自己的主循环里面调用
    }
}

void postdispatchtest()
{
    asio::io_context io_context;
    uint16_t s = 0;
    //使用make_work_guard保证io_context不退出
    auto worker = asio::make_work_guard(io_context);

    std::thread t2([&io_context](){ io_context.run(); });
    std::thread t([&io_context,&s]()
    {
        while (++s)
        {
            //post到t2中执行
            io_context.post([&io_context]()
            {
                //继续post回调函数
                io_context.post([]()
                {
                    cout<<"post will run after all dispatch, threadid:"<<std::this_thread::get_id()<<"\n";
                });
                //dispatch方法和run方法在同一线程，直接执行回调函数
                io_context.dispatch([]()
                {
                    cout<<"dispatch1 will run first, threadid:"<<std::this_thread::get_id()<<"\n";
                });
                io_context.dispatch([]()
                {
                    cout<<"dispatch2 will run first, threadid:"<<std::this_thread::get_id()<<"\n";
                });
            });
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    t2.join();
    t.join();  
}

void threadpooltest()
{
    asio::thread_pool io(1);
    //io.join(); need call in Destructor

    uint16_t s = 0;
    std::thread t([&s](){
    while (++s)
    {
        //post使用方法
        asio::post([s]()                     
        {   
            cout<<s; cout.flush();
        });
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }});


    t.join();
}


// 在单独一个线程中调用run，在其它线程中post
void MultiThreadRun()
{
    asio::io_context io; // 无论是post还是dispatch，其待调用的函数都是在io_context::run的线程里面运行
    asio::strand<asio::io_context::executor_type> str(io.get_executor()); // strand的用法，这里用来防止cout输出错乱

    thread t1([&io, &str]()
    {
        (void)str; // unused warning
            //asio::executor_work_guard<asio::io_context::executor_type> wg
            auto wg = asio::make_work_guard(io);// executor_work_guard可以防止io.run()在事件处理完以后退出
            io.run();// 从任何线程发起的post/dispatch的函数都在调用run/poll的线程中执行
        });

    thread t2([&io, &str]()
    {
//		cout<<"thread2 id:"<<std::this_thread::get_id()<<"\n";

            for(int i = 0; i < 10; ++i)
            {
                // 如果是调用dispatch的线程和调用io_context::run的所有线程都不是同一个线程(因为可以在多个线程中调用run)，则dispatch和post的效果是一样的。
                // 此处就是一样, dispatch就等于post，都会被post到另一个线程中去执行
                io.dispatch(asio::bind_executor(str, []()
                                {
                                    cout<<"dispatch, threadid:"<<std::this_thread::get_id()<<"\n";
                                }));
                io.post(asio::bind_executor(str, []()
                                {
                                    cout<<"post, threadid:"<<std::this_thread::get_id()<<"\n";
                                }));

                //下面就不一样了，首先是回调函数被post到t1中去执行
                //后来再执行内部的dispatch和post时，已经是在t1中了，所以会立即执行dispatch，而post会最后执行
                //得到的效果就是所有的disatch都执行完了，才执行post
                io.post([&io, &str]()
                        {
                            io.dispatch(asio::bind_executor(str, []()
                                            {
                                                cout<<"dispatch will run first, threadid:"<<std::this_thread::get_id()<<"\n";
                                            }));
                            io.post(asio::bind_executor(str, []()
                                            {
                                                cout<<"post will run after all dispatch, threadid:"<<std::this_thread::get_id()<<"\n";
                                            }));
                        });
            }
        });

    t2.join();
    t1.join();
}

void strandtest()
{
    asio::io_context io_context;
    auto worker = asio::make_work_guard(io_context);

    std::vector<std::thread> runthreads;
    for(int i=0;i<3;i++)
        runthreads.emplace_back(std::thread([&io_context](){ io_context.run(); }));
    auto fun1 = []()
    {
        std::cout<< "fun1 " << std::this_thread::get_id()<<std::endl;
    };
    auto fun2 = []()
    {
        std::cout<< "fun2 " << std::this_thread::get_id()<<std::endl;
    };
    {
        //乱序
        for(int i = 0;i<10;i++)
        {
            asio::post(io_context,fun1);
            asio::post(io_context,fun2);
        }
    }
    sleep(5);
    std::cout << "use strand" << std::endl;
    {
        //顺序绑定strand
        asio::strand<asio::io_context::executor_type> fun1strand(asio::make_strand(io_context));
        asio::strand<asio::io_context::executor_type> fun2strand(asio::make_strand(io_context));

        for(int i = 0;i<10;i++)
        {
            asio::post(asio::bind_executor(fun1strand,fun1));
            asio::post(asio::bind_executor(fun2strand,fun2));
        }
    }

    for(auto &tmp:runthreads)
        tmp.join();
}

std::string make_string(asio::streambuf& streambuf)
{
 return {buffers_begin(streambuf.data()), 
         buffers_end(streambuf.data())};
}

void streambuffertest()
{
    asio::streambuf write_buffer;
    std::ostream output(&write_buffer);
    output << "a@"
            "b@";
    std::cout << "Wrote: " << make_string(write_buffer) << std::endl;

    write_buffer.consume(2);
    std::cout << "Consumed write buffer, it now contains: " <<
                    make_string(write_buffer) << std::endl;
}


void Print(std::error_code ec, asio::steady_timer* timer, int *count) 
{
    if (*count < 3)
    {
        std::cout << *count << std::endl;
        ++(*count);       
        timer->expires_at(timer->expires_at() + std::chrono::seconds(1));
        timer->async_wait(std::bind(&Print, std::placeholders::_1, timer, count));
    }
}

void steadytimertest()
{
    asio::io_context ioc;
    asio::steady_timer timer(ioc, std::chrono::seconds(3));
    int count = 0;
    timer.async_wait(std::bind(&Print,std::placeholders::_1, &timer, &count));
    ioc.run();
}

int main()
{
    // main1();
    // coroutinuec20();
    // actortest();
    // restartpoll();
    // postdispatchtest();
    // threadpooltest();
    // MultiThreadRun();
    // strandtest();
    steadytimertest();
    return 0;
}