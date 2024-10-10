#pragma once

#include <chrono>
#include <thread>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <map>
#include <tuple>
#include <unordered_map>
#include "queue/threadsafecontainer.hpp"
#include "tools/simple_list.hpp"
#include "rbtree/rbtreewrap.hpp"
#include "objectpool.hpp"
#include "global.hpp"

namespace timermanager
{
namespace v1
{
template <typename T>
class TimerManager
{
public:
    static TimerManager* GetInstance()
    {
        static TimerManager instance;
        return &instance;
    }

    void AddAlarm(std::chrono::milliseconds alarm,
        T key,
        std::function<void()> fun,
        std::chrono::milliseconds interval = std::chrono::milliseconds(0),
        bool runinmainthread               = true)
    {
        TIMETYPE::time_point timepoint = TIMETYPE::now() + alarm;
        __timer_queue.AddObj(
            {{}, timepoint, fun, key, interval, runinmainthread});
        __cv.notify_one();
    }

    void DeleteAlarm(T&& key)
    {
        __timer_queue.DeleteObj([key = key](const TimerElement& timerelement) {
            return timerelement.key == key;
        });
        __cv.notify_one();
    }

    void DeleteAlarmCondition(std::function<bool(const T& key)> fun)
    {
        __timer_queue.DeleteObj([fun = fun](const TimerElement& timerelement) {
            return fun(timerelement.key);
        });
        __cv.notify_one();
    }

    void StopTimerManager()
    {
        __stop = true;
        __cv.notify_one();
        if (__timer_thread.joinable())
        {
            __timer_thread.join();
        }
    }

    void StartTimerManager()
    {
        if (!__stop)
        {
            return;
        }
        __stop         = false;
        __timer_thread = std::thread(&TimerManager::RunTimerManager, this);
    }

private:
    using TIMETYPE = std::chrono::steady_clock;
    struct TimerElement
    {
        rb_node rbNode = {nullptr, nullptr, nullptr, RB_RED};
        TIMETYPE::time_point alarm;
        std::function<void()> fun;
        T key;
        std::chrono::milliseconds interval = std::chrono::milliseconds(0);
        bool run_in_main_thread            = true;
        bool operator<(const TimerElement& t) const
        {
            return alarm < t.alarm;
        }
    };

    std::atomic<bool> __stop = {true};
    rbtreewrap::v1::RBTreeWrap<TimerElement> __timer_queue;
    std::thread __timer_thread;
    std::condition_variable __cv;
    std::mutex __mutex;

    TimerManager()
    {
        StartTimerManager();
    }

    ~TimerManager()
    {
        StopTimerManager();
    }

    void RunTimerManager()
    {
        pthread_setname_np(pthread_self(), "ACCM-TimerManager");
        while (!__stop)
        {
            std::unique_lock<std::mutex> lck(__mutex);
            auto timepointopt = __timer_queue.GetTopObjKey(
                std::function<TIMETYPE::time_point(TimerElement*)>(
                    [](TimerElement* element) {
                        return element->alarm;
                    }));

            if (timepointopt.has_value())
            {
                __cv.wait_until(lck, timepointopt.value());
            }
            else
            {
                __cv.wait_until(lck, TIMETYPE::now() + std::chrono::seconds(1));
            }

            auto elementopt
                = __timer_queue.GetTopObjAndDelete([](TimerElement* e) {
                      return TIMETYPE::now() > e->alarm;
                  });
            if (elementopt.has_value())
            {
                auto element = elementopt.value();
                if (element.interval > std::chrono::milliseconds(0))
                    AddAlarm(element.interval,
                        element.key,
                        element.fun,
                        element.interval,
                        element.run_in_main_thread);
                if (element.fun)
                {
                    if (element.run_in_main_thread)
                    {
                        element.fun();
                    }
                    else
                    {
                        std::thread tmp([fun = element.fun]() {
                            fun();
                        });
                        tmp.detach();
                    }
                }
            }
        }
        while (__timer_queue.PopTopObj())
        { };
    }
};
}  // namespace v1

namespace v2
{

template <typename T>
class TimerManager
{
public:
    enum class DeleteModel
    {
        ANY,
        ALL
    };
    static TimerManager* GetInstance()
    {
        static TimerManager instance;
        return &instance;
    }

    void AddAlarm(std::chrono::milliseconds alarm,
        const T& key,
        const std::string& additional,
        std::function<void()> fun,
        std::chrono::milliseconds interval = std::chrono::milliseconds(0),
        bool runinmainthread               = true)
    {
        TIMETYPE::time_point timepoint = TIMETYPE::now() + alarm;
        auto element = std::make_shared<TimerElement>(TimerElement{
            timepoint, fun, key, additional, interval, runinmainthread});

        std::lock_guard<std::recursive_mutex> lck(__data_mutex);
        auto rbtreenode     = __timer_queue.AddObj(element);
        element->rbtreenode = rbtreenode;
        __key_2_element.insert({key, element});

        __cv.notify_one();
    }

    bool DeleteAlarm(const T& key,
        const std::string& additional,
        DeleteModel deletemodel = DeleteModel::ALL)
    {
        bool flag = false;
        std::lock_guard<std::recursive_mutex> lck(__data_mutex);
        auto members = __key_2_element.equal_range(key);

        for (auto member = members.first; member != members.second;)
        {
            if (member->second->additional == additional)
            {
                __timer_queue.DeleteObj(member->second->rbtreenode);
                member = __key_2_element.erase(member);
                flag   = true;
                if (deletemodel == DeleteModel::ANY)
                {
                    break;
                }
            }
            else
            {
                member++;
            }
        }

        __cv.notify_one();
        return flag;
    }

    bool DeleteAlarm(const T& key)
    {
        bool flag = false;
        std::lock_guard<std::recursive_mutex> lck(__data_mutex);
        auto members = __key_2_element.equal_range(key);
        for (auto member = members.first; member != members.second;)
        {
            __timer_queue.DeleteObj(member->second->rbtreenode);
            member = __key_2_element.erase(member);
            flag   = true;
        }
        __cv.notify_one();
        return flag;
    }

    size_t GetSize()
    {
        return __timer_queue.GetSize();
    }

    void StopTimerManager()
    {
        __stop = true;
        __cv.notify_one();
        if (__timer_thread.joinable())
        {
            __timer_thread.join();
        }
        std::lock_guard<std::recursive_mutex> lck2(__data_mutex);
        __key_2_element.clear();
        __timer_queue.Clear();
    }

    void StartTimerManager()
    {
        if (!__stop)
        {
            return;
        }
        __stop         = false;
        __timer_thread = std::thread(&TimerManager::RunTimerManager, this);
    }

private:
    using TIMETYPE = std::chrono::steady_clock;
    struct TimerElement
    {
        TIMETYPE::time_point alarm;
        std::function<void()> fun;
        T key;
        std::string additional             = "";
        std::chrono::milliseconds interval = std::chrono::milliseconds(0);
        bool run_in_main_thread            = true;
        void* rbtreenode                   = nullptr;
        bool operator<(const TimerElement& t) const
        {
            return alarm < t.alarm;
        }
    };

    std::atomic<bool> __stop = {true};
    rbtreewrap::v2::RBTreeWrap<std::shared_ptr<TimerElement>> __timer_queue;
    std::thread __timer_thread;
    std::condition_variable __cv;
    std::mutex __cv_mutex;
    std::recursive_mutex __data_mutex;
    std::multimap<T, std::shared_ptr<TimerElement>> __key_2_element;

    TimerManager()
    {
        StartTimerManager();
    }

    ~TimerManager()
    {
        StopTimerManager();
    }

    void RunTimerManager()
    {
        pthread_setname_np(pthread_self(), "TimerManager");
        while (!__stop)
        {
            Wait();
            auto element = PopTopObjIfTimeOut();
            DealWithTimerElement(element);
        }
    }

    void Wait()
    {
        std::unique_lock<std::mutex> lck(__cv_mutex);
        bool flag;
        TIMETYPE::time_point timepoint;
        std::tie(flag, timepoint) = __timer_queue.GetTopObjKeyByFunction(
            std::function<TIMETYPE::time_point(
                const std::shared_ptr<TimerElement>&)>(
                [](const std::shared_ptr<TimerElement>& e) {
                    return e->alarm;
                }));

        if (flag)
        {
            __cv.wait_until(lck, timepoint);
        }
        else
        {
            __cv.wait_until(lck, TIMETYPE::now() + std::chrono::seconds(1));
        }
    }

    bool __DeleteAlarm(std::shared_ptr<TimerElement> element)
    {
        std::lock_guard<std::recursive_mutex> lck(__data_mutex);
        auto members = __key_2_element.equal_range(element->key);
        for (auto member = members.first; member != members.second;)
        {
            if (member->second == element)
            {
                __timer_queue.DeleteObj(element->rbtreenode);
                member = __key_2_element.erase(member);
                return true;
            }
            else
            {
                member++;
            }
        }
        __cv.notify_one();
        return false;
    }

    std::shared_ptr<TimerElement> PopTopObjIfTimeOut()
    {
        bool flag;
        std::shared_ptr<TimerElement> element;
        std::lock_guard<std::recursive_mutex> lck(__data_mutex);
        std::tie(flag, element) = __timer_queue.GetTopObjByFunction(
            [](const std::shared_ptr<TimerElement>& e) {
                return TIMETYPE::now() > e->alarm;
            });
        if (flag)
        {
            if (__DeleteAlarm(element))
            {
                return element;
            }
        }
        return nullptr;
    }

    void DealWithTimerElement(std::shared_ptr<TimerElement> element)
    {
        if (element)
        {
            if (element->fun)
            {
                if (element->run_in_main_thread)
                {
                    element->fun();
                }
                else
                {
                    std::thread tmp([fun = element->fun]() {
                        fun();
                    });
                    tmp.detach();
                }
            }
            if (element->interval > std::chrono::milliseconds(0))
            {
                AddAlarm(element->interval,
                    element->key,
                    element->additional,
                    element->fun,
                    element->interval,
                    element->run_in_main_thread);
            }
        }
    }
};
}  // namespace v2

inline namespace v3
{
template <typename T>
class TimerManager
{
    using TIMETYPE = std::chrono::steady_clock;

public:
    enum class DeleteModel
    {
        ANY,
        ALL
    };

    static TimerManager* GetInstance()
    {
        static TimerManager instance;
        return &instance;
    }

    void AddAlarm(std::chrono::milliseconds alarm,
        const T& key,
        const std::string& additional,
        std::function<void()>&& fun,
        std::chrono::milliseconds interval = std::chrono::milliseconds(0))
    {
        TimerElement* element = nullptr;
        if (!__stop)
        {
            element = ObjectPool<TimerElement>::GetInstance()->GetObject(
                TIMETYPE::now() + alarm,
                std::move(fun),
                std::move(key),
                std::move(additional),
                interval);

            std::lock_guard<std::mutex> lck(__mutex);
            auto [flag, timepoint] = __timer_queue.GetTopObjIf(
                std::function<TIMETYPE::time_point(const TimerElement*)>(
                    [](const TimerElement* e) -> TIMETYPE::time_point {
                        return e->alarm;
                    }));
            element->iter = __key2element.emplace(key, element);
            __timer_queue.AddObj(element);
            if (flag && element->alarm < timepoint)
                __cv.notify_one();
        }
    }

    void DeleteAlarm(const T& key,
        const std::string& additional,
        DeleteModel deletemodel = DeleteModel::ANY)
    {
        std::lock_guard<std::mutex> lck(__mutex);
        auto members = __key2element.equal_range(key);
        for (auto member = members.first; member != members.second; member++)
        {
            if (additional == member->second->additional)
            {
                member->second->SetCallBack(nullptr);
                if (deletemodel == DeleteModel::ANY)
                    break;
            }
        }
        __cv.notify_one();
    }

    void DeleteAlarm(const T& key)
    {
        std::lock_guard<std::mutex> lck(__mutex);
        auto members = __key2element.equal_range(key);
        for (auto member = members.first; member != members.second; member++)
            member->second->SetCallBack(nullptr);
        __cv.notify_one();
    }

    void StopTimerManager()
    {
        __stop = true;
        __cv.notify_one();
        if (__timerThread.joinable())
        {
            __timerThread.join();
        }
        std::unique_lock<std::mutex> lck(__mutex);
        auto timer_queue(std::move(__timer_queue));
        lck.unlock();
        timer_queue.ExecuteAll([&](TimerElement* element) {
            // element->SetCallBack(nullptr);
            timer_queue.DeleteObj((rb_node*)element);
            __key2element.erase(element->iter);
            ObjectPool<TimerElement>::GetInstance()->PutObject(element);
            return false;
        });
    }

    void StartTimerManager()
    {
        if (!__stop)
        {
            return;
        }
        __stop        = false;
        __timerThread = std::thread(&TimerManager::RunTimerManager, this);
    }

private:
    struct TimerElement
    {
        rb_node rbnode = {nullptr, nullptr, nullptr, RB_RED};
        typename std::multimap<T, TimerElement*>::iterator iter;
        TIMETYPE::time_point alarm;
        std::function<void()> fun;
        T key;
        std::string additional             = "";
        std::chrono::milliseconds interval = std::chrono::milliseconds(0);
        std::mutex cb_mutex;
        bool operator<(const TimerElement& t) const
        {
            return alarm < t.alarm;
        }
        TimerElement(TIMETYPE::time_point _alarm,
            std::function<void()>&& _fun,
            const T& _key,
            const std::string& _additional,
            std::chrono::milliseconds _interval)
            : alarm(_alarm)
            , fun(std::move(_fun))
            , key(std::move(_key))
            , additional(std::move(_additional))
            , interval(_interval)
        { }
        // TimerElement(TimerElement&& element)
        //     : rbnode(element.rbnode)
        //     , iter(element.iter)
        //     , alarm(element.alarm)
        //     , fun(std::move(element.fun))
        //     , key(std::move(element.key))
        //     , additional(std::move(element.additional))
        //     , interval(element.interval)
        // { }

        ~TimerElement()
        {
            SetCallBack(nullptr);
        }
        void SetCallBack(std::function<void()> fun)
        {
            std::lock_guard<std::mutex> lck(cb_mutex);
            this->fun = fun;
        }
        bool Callback()
        {
            std::lock_guard<std::mutex> lck(cb_mutex);
            if (fun)
            {
                fun();
                return true;
            }
            return false;
        }
    };
    std::atomic<bool> __stop = {true};
    rbtreewrap::v3::RBTreeWrap<TimerElement> __timer_queue;
    std::thread __timerThread;
    std::condition_variable __cv;
    std::mutex __mutex;
    std::multimap<T, TimerElement*> __key2element;

    TimerManager()
    {
        StartTimerManager();
    }

    ~TimerManager()
    {
        StopTimerManager();
    }

    void RunTimerManager()
    {
        pthread_setname_np(pthread_self(), "ACCM-TimerManager");
        while (!__stop)
        {
            Wait();
            DealWithTimerElement(GetTimerElement());
        }
    }

    void Wait()
    {
        bool flag;
        TIMETYPE::time_point timepoint;
        std::unique_lock<std::mutex> lck(__mutex);
        std::tie(flag, timepoint) = __timer_queue.GetTopObjIf(
            std::function<TIMETYPE::time_point(const TimerElement*)>(
                [](const TimerElement* e) -> TIMETYPE::time_point {
                    return e->alarm;
                }));

        if (flag)
        {
            __cv.wait_until(lck, timepoint);
        }
        else
        {
            __cv.wait_until(lck, TIMETYPE::now() + std::chrono::seconds(1));
        }
    }

    void DealWithTimerElement(TimerElement* element)
    {
        if (element)
        {
            if (element->Callback()
                && element->interval > std::chrono::milliseconds(0))
            {
                element->alarm = TIMETYPE::now() + element->interval;
                std::lock_guard<std::mutex> lck(__mutex);
                __timer_queue.AddObj(element);
            }
            else
            {
                {
                    std::lock_guard<std::mutex> lck(__mutex);
                    __key2element.erase(element->iter);
                }
                ObjectPool<TimerElement>::GetInstance()->PutObject(element);
            }
        }
    }

    TimerElement* GetTimerElement()
    {
        std::lock_guard<std::mutex> lck(__mutex);
        auto [flag, element]
            = __timer_queue.GetTopObjIfAndDelete([](const TimerElement* e) {
                  return TIMETYPE::now() > e->alarm;
              });
        if (flag)
            return element;
        else
            return nullptr;
    }
};
}  // namespace v3

namespace v4
{
template <typename T>
class TimerManager
{
    using TIMETYPE = std::chrono::steady_clock;

public:
    static TimerManager* GetInstance()
    {
        static TimerManager instance;
        return &instance;
    }

    void AddAlarm(std::chrono::milliseconds alarm,
        const T& key,
        const std::string& additional,
        std::function<void()>&& fun,
        std::chrono::milliseconds interval = std::chrono::milliseconds(0))
    {
        if (!__stop)
        {
            size_t hash_value = CalculateHashValue(key, additional);
            TimerElement* element
                = ObjectPool<TimerElement>::GetInstance()->GetObject(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        TIMETYPE::now().time_since_epoch())
                        + alarm,
                    std::move(fun),
                    std::move(key),
                    std::move(additional),
                    interval);

            std::lock_guard<std::mutex> lck(__mutex);
            if (__key_node_map.find(hash_value) != __key_node_map.end())
            {
                std::cerr << "key conflict" << std::endl;
                ObjectPool<TimerElement>::GetInstance()->PutObject(element);
                return;
            }

            typename SimpleList<TimerElement*>::Node* node_ptr;

            if (auto simple_list = __timer_map.find(element->alarm);
                simple_list != __timer_map.end())
                node_ptr = simple_list->second->AddNode(std::move(element));
            else
                node_ptr
                    = __timer_map
                          .emplace(element->alarm,
                              std::make_shared<SimpleList<TimerElement*>>())
                          .first->second->AddNode(std::move(element));
            element->iter = __key_node_map.emplace(hash_value, node_ptr).first;

            if (alarm == std::chrono::milliseconds(0))
                __cv.notify_one();
        }
    }

    void DeleteAlarm(const T& key, const std::string& additional)
    {
        if (!__stop)
        {
            size_t hash_value = CalculateHashValue(key, additional);
            std::lock_guard<std::mutex> lck(__mutex);
            if (auto iter = __key_node_map.find(hash_value);
                iter != __key_node_map.end())
            {
                iter->second->data->SetCallBack(nullptr);
            }
        }
    }

    void StopTimerManager()
    {
        __stop = true;
        __cv.notify_one();
        if (__timerThread.joinable())
            __timerThread.join();

        std::map<std::chrono::milliseconds,
            std::shared_ptr<SimpleList<TimerElement*>>>
            timer_map;
        {
            std::lock_guard<std::mutex> lck(__mutex);
            __key_node_map.clear();
            timer_map.swap(__timer_map);
        }
        for (auto& [timepoint, simple_list] : timer_map)
            simple_list->RemoveNode([](TimerElement* element) {
                ObjectPool<TimerElement>::GetInstance()->PutObject(element);
                return true;
            });
    }

    void StartTimerManager()
    {
        if (!__stop)
        {
            return;
        }
        __stop        = false;
        __timerThread = std::thread(&TimerManager::RunTimerManager, this);
    }

    // only for test
    size_t GetSize()
    {
        std::lock_guard<std::mutex> lck(__mutex);
        return __key_node_map.size();
    }

private:
    struct Key
    {
        T key;
        std::string additional = "";
        bool operator<(const Key& k) const
        {
            if (key < k.key)
                return true;
            else if (key == k.key)
                return additional < k.additional;
            else
                return false;
        }
    };
    struct TimerElement
    {
        std::chrono::milliseconds alarm;
        std::function<void()> fun;
        std::chrono::milliseconds interval = std::chrono::milliseconds(0);
        std::unordered_map<size_t,
            typename SimpleList<TimerElement*>::Node*>::iterator iter;
        std::mutex cb_mutex;
        bool operator<(const TimerElement& t) const
        {
            return alarm < t.alarm;
        }
        TimerElement(std::chrono::milliseconds _alarm,
            std::function<void()>&& _fun,
            const T& _key,
            const std::string& _additional,
            std::chrono::milliseconds _interval)
            : alarm(_alarm)
            , fun(std::move(_fun))
            , interval(_interval)
        { }

        ~TimerElement()
        {
            SetCallBack(nullptr);
        }
        void SetCallBack(std::function<void()> fun)
        {
            std::lock_guard<std::mutex> lck(cb_mutex);
            this->fun = fun;
        }
        bool Callback()
        {
            std::lock_guard<std::mutex> lck(cb_mutex);
            if (fun)
            {
                fun();
                return true;
            }
            return false;
        }
    };
    std::atomic<bool> __stop = {true};
    std::map<std::chrono::milliseconds,
        std::shared_ptr<SimpleList<TimerElement*>>>
        __timer_map;
    std::unordered_map<size_t, typename SimpleList<TimerElement*>::Node*>
        __key_node_map;
    std::thread __timerThread;
    std::condition_variable __cv;
    std::mutex __mutex;
    std::multimap<T, TimerElement*> __key2element;

    TimerManager()  = default;
    ~TimerManager() = default;

    void RunTimerManager()
    {
        pthread_setname_np(pthread_self(), "ACCM-TimerManager");
        while (!__stop)
        {
            Wait();
            if (auto simple_list = GetTimerElement())
                DealWithTimerElement(simple_list);
        }
    }

    void Wait()
    {
        bool flag;
        TIMETYPE::time_point timepoint;
        std::unique_lock<std::mutex> lck(__mutex);
        if (auto element = __timer_map.begin(); element != __timer_map.end())
            __cv.wait_for(
                lck, element->first - TIMETYPE::now().time_since_epoch());
        else
            __cv.wait_until(lck, TIMETYPE::now() + std::chrono::seconds(1));
    }

    std::shared_ptr<SimpleList<TimerElement*>> GetTimerElement()
    {
        std::lock_guard<std::mutex> lck(__mutex);
        if (auto iter = __timer_map.begin(); iter != __timer_map.end())
        {
            if (TIMETYPE::now().time_since_epoch() > iter->first)
            {
                auto simple_list = iter->second;
                __timer_map.erase(iter);
                return simple_list;
            }
            else
                return nullptr;
        }
        else
            return nullptr;
    }

    void DealWithTimerElement(
        std::shared_ptr<SimpleList<TimerElement*>> simple_list)
    {
        while (typename SimpleList<TimerElement*>::Node* node_ptr
            = simple_list->PopHead())
        {
            if (node_ptr->data->Callback()
                && node_ptr->data->interval > std::chrono::milliseconds(0))
            {
                node_ptr->data->alarm
                    = std::chrono::duration_cast<std::chrono::milliseconds>(
                          TIMETYPE::now().time_since_epoch())
                    + node_ptr->data->interval;
                std::lock_guard<std::mutex> lck(__mutex);
                if (auto simple_list_new
                    = __timer_map.find(node_ptr->data->alarm);
                    simple_list_new != __timer_map.end())
                    simple_list_new->second->AddNode(node_ptr);
                else
                    __timer_map
                        .emplace(node_ptr->data->alarm,
                            std::make_shared<SimpleList<TimerElement*>>())
                        .first->second->AddNode(node_ptr);
            }
            else
            {
                {
                    std::lock_guard<std::mutex> lck(__mutex);
                    __key_node_map.erase(node_ptr->data->iter);
                }
                ObjectPool<TimerElement>::GetInstance()->PutObject(
                    node_ptr->data);
                ObjectPool<
                    typename SimpleList<TimerElement*>::Node>::GetInstance()
                    ->PutObject(node_ptr);
            }
        }
    }

    size_t CalculateHashValue(const T& key, const std::string& additional)
    {

        auto hash_t = std::hash<T>{}(key);
        if (hash_t == 0)
            return std::hash<std::string>{}(additional);
        else
            return HashCombine(hash_t, std::hash<std::string>{}(additional));
    }
};
}  // namespace v4

}  // namespace timermanager