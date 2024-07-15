#pragma once

#include <functional>
#include <mutex>
#include <memory>
#include <set>
#include <type_traits>
#include <optional>
#include "rbtree.h"

namespace rbtreewrap
{
inline namespace v1
{
template <typename T>
class RBTreeWrap
{
public:
    ~RBTreeWrap()
    {
        DeleteObj([](const T&) {
            return true;
        });
    }
    void AddObj(const T& t)
    {
        auto newentry = new Entry(t);

        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto newPos = &(__rbtree.rb_node);
        auto parent = static_cast<rb_node*>(nullptr);
        while (*newPos != nullptr)
        {
            auto curr = reinterpret_cast<Entry*>(*newPos);
            parent    = *newPos;
            if (t < curr->t)
            {
                newPos = &((*newPos)->rb_left);
            }
            else
            {
                newPos = &((*newPos)->rb_right);
            }
        }
        rb_link_node(&newentry->rbnode, parent, newPos);
        rb_insert_color(&newentry->rbnode, &__rbtree);
    }

    void DeleteObj(std::function<bool(const T&)> comparefun)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        for (auto node = rb_first(&__rbtree); node;)
        {
            struct rb_node* next = rb_next(node);
            Entry* entry         = rb_entry(node, Entry, rbnode);
            if (comparefun(entry->t))
            {
                rb_erase(node, &__rbtree);
                delete entry;
                entry = nullptr;
            }
            node = next;
        }
    }

    void DeleteObj(const T& t)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        DeleteObj([element = t](const T& m) {
            return element == m;
        });
    }

    template <typename R>
    std::optional<R> GetTopObjKey(std::function<R(T*)> f)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto top = rb_first(&__rbtree);
        if (top)
        {
            return std::optional<R>(f(&(reinterpret_cast<Entry*>(top))->t));
        }
        return std::optional<R>();
    }

    std::optional<T> GetTopObjAndDelete(std::function<bool(T*)> f)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto top = rb_first(&__rbtree);
        if (top)
        {
            Entry* entry = (reinterpret_cast<Entry*>(top));
            if (f(&(entry->t)))
            {
                T t = entry->t;
                rb_erase(&(entry->rbnode), &__rbtree);
                delete entry;
                entry = nullptr;
                return std::optional<T>(t);
            }
        }
        return std::optional<T>();
    }

    T* GetTopObjPtr()
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto top = rb_first(&__rbtree);
        if (top)
        {
            return &(reinterpret_cast<Entry*>(top))->t;
        }
        return nullptr;
    }

    bool PopTopObj()
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto top = rb_first(&__rbtree);
        if (top)
        {
            Entry* entry = (reinterpret_cast<Entry*>(top));
            rb_erase(&(entry->rbnode), &__rbtree);
            delete entry;
            entry = nullptr;
            return true;
        }
        else
        {
            return false;
        }
    }

    T* SearchObj(const T& key)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        return SearchObj([key = key](const T& t) {
            return key == t;
        });
    }

    T* SearchObj(std::function<bool(const T&)> comparefun)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        struct rb_node* node;
        for (node = rb_first(&__rbtree); node; node = rb_next(node))
        {
            Entry* entry = rb_entry(node, Entry, rbnode);
            if (comparefun(entry->t))
            {
                return &entry->t;
            }
        }
        return nullptr;
    }

private:
    struct Entry
    {
        Entry() = default;
        Entry(T e)
            : t(std::move(e))
        { }
        rb_node rbnode = {nullptr, nullptr, nullptr, RB_RED};
        T t;
    };
    rb_root __rbtree = {nullptr};
    std::recursive_mutex __mutex;
};
}  // namespace v1
namespace v2
{

template <typename T>
struct isPtr : public std::false_type
{ };

template <typename T>
struct isPtr<std::shared_ptr<T>> : public std::true_type
{ };

template <typename T>
struct isPtr<T*> : public std::true_type
{ };

template <typename T>
class RBTreeWrap
{
public:
    ~RBTreeWrap()
    {
        Clear();
    }

    void Clear()
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        for (auto node = rb_first(&__rbtree); node;)
        {
            struct rb_node* next = rb_next(node);
            DeleteObj(node);

            node = next;
        }
        __rbtree.rb_node = nullptr;
        __size           = 0;
    }

    template <typename U                                 = T,
        typename std::enable_if_t<!isPtr<U>::value, int> = 0>
    void* AddObj(const T& t)
    {
        auto newentry = new Entry({{nullptr, nullptr, nullptr, RB_RED}, t});

        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto newPos = &(__rbtree.rb_node);
        auto parent = static_cast<rb_node*>(nullptr);
        while (*newPos != nullptr)
        {
            auto curr = reinterpret_cast<Entry*>(*newPos);
            parent    = *newPos;
            if (t < curr->t)
            {
                newPos = &((*newPos)->rb_left);
            }
            else
            {
                newPos = &((*newPos)->rb_right);
            }
        }
        rb_link_node(&newentry->rbnode, parent, newPos);
        rb_insert_color(&newentry->rbnode, &__rbtree);
        __size++;
        return &newentry->rbnode;
    }

    template <typename U                                = T,
        typename std::enable_if_t<isPtr<U>::value, int> = 0>
    void* AddObj(const T& t)
    {
        auto newentry = new Entry({{nullptr, nullptr, nullptr, RB_RED}, t});

        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto newPos = &(__rbtree.rb_node);
        auto parent = static_cast<rb_node*>(nullptr);
        while (*newPos != nullptr)
        {
            auto curr = reinterpret_cast<Entry*>(*newPos);
            parent    = *newPos;
            if ((*t) < (*curr->t))
            {
                newPos = &((*newPos)->rb_left);
            }
            else
            {
                newPos = &((*newPos)->rb_right);
            }
        }
        rb_link_node(&newentry->rbnode, parent, newPos);
        rb_insert_color(&newentry->rbnode, &__rbtree);
        __size++;
        return &newentry->rbnode;
    }

    void DeleteObj(void* nodeptr)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto node = reinterpret_cast<rb_node*>(nodeptr);
        rb_erase(node, &__rbtree);
        delete reinterpret_cast<Entry*>(nodeptr);
        nodeptr = nullptr;
        __size--;
    }

    size_t GetSize()
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        return __size;
    }

    template <typename R>
    std::tuple<bool, R> GetTopObjKeyByFunction(std::function<R(const T&)> f)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto top = rb_first(&__rbtree);
        if (top)
        {
            Entry* entry = (reinterpret_cast<Entry*>(top));
            return std::make_tuple(true, f(entry->t));
        }
        return std::make_tuple(false, R());
    }

    std::tuple<bool, T> GetTopObjByFunction(std::function<bool(const T&)> f)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto top = rb_first(&__rbtree);
        if (top)
        {
            Entry* entry = (reinterpret_cast<Entry*>(top));
            if (f(entry->t))
            {
                T t = entry->t;
                return std::make_tuple(true, t);
            }
        }
        return std::make_tuple(false, T());
    }

    std::tuple<bool, T> GetTopObjByFunctionAndDelete(
        std::function<bool(const T&)> f)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto top = rb_first(&__rbtree);
        if (top)
        {
            Entry* entry = (reinterpret_cast<Entry*>(top));
            if (f(entry->t))
            {
                T t = entry->t;
                DeleteObj(&entry->rbnode);
                return std::make_tuple(true, t);
            }
        }
        return std::make_tuple(false, T());
    }

    void PrintAll(std::function<void(const T&)> fun)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        struct rb_node* node;
        for (node = rb_first(&__rbtree); node; node = rb_next(node))
        {
            Entry* entry = rb_entry(node, Entry, rbnode);
            fun(entry->t);
        }
    }

    std::tuple<bool, T> SearchObj(const T& key)
    {
        return SearchObj([key = key](const T& t) {
            return key == t;
        });
    }

    std::tuple<bool, T> SearchObj(std::function<bool(const T&)> comparefun)
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        struct rb_node* node;
        for (node = rb_first(&__rbtree); node; node = rb_next(node))
        {
            Entry* entry = rb_entry(node, Entry, rbnode);
            if (comparefun(entry->t))
            {
                return std::make_tuple(true, entry->t);
            }
        }
        return std::make_tuple(false, T());
    }

private:
    struct Entry
    {
        rb_node rbnode = {nullptr, nullptr, nullptr, RB_RED};
        T t;
    };
    rb_root __rbtree = {nullptr};
    std::recursive_mutex __mutex;
    size_t __size = 0;
};
}  // namespace v2

namespace v3
{

template <typename T>
struct isPtr : public std::false_type
{ };

template <typename T>
struct isPtr<std::shared_ptr<T>> : public std::true_type
{ };

template <typename T>
struct isPtr<T*> : public std::true_type
{ };

template <typename T>
class RBTreeWrap
{
public:
    RBTreeWrap() = default;
    RBTreeWrap(RBTreeWrap&& other) noexcept
        : m_rbtree(other.m_rbtree)
    {
        other.m_rbtree.rb_node = nullptr;
    }
    ~RBTreeWrap()
    {
        m_rbtree.rb_node = nullptr;
    }

    template <typename U                                 = T,
        typename std::enable_if_t<!isPtr<U>::value, int> = 0>
    void AddObj(T* t)
    {
        rb_node** newPos = &(m_rbtree.rb_node);
        rb_node* parent  = static_cast<rb_node*>(nullptr);
        while (*newPos != nullptr)
        {
            auto curr = reinterpret_cast<T*>(*newPos);
            parent    = *newPos;
            if (*t < *curr)
            {
                newPos = &((*newPos)->rb_left);
            }
            else
            {
                newPos = &((*newPos)->rb_right);
            }
        }
        rb_link_node((rb_node*)t, parent, newPos);
        rb_insert_color((rb_node*)t, &m_rbtree);
    }

    void DeleteObj(rb_node* nodeptr)
    {
        rb_erase(nodeptr, &m_rbtree);
    }

    template <typename R>
    std::tuple<bool, R> GetTopObjIf(std::function<R(const T*)> f)
    {
        auto top = rb_first(&m_rbtree);
        if (top)
        {
            return std::make_tuple(true, f((T*)top));
        }
        return std::make_tuple(false, R());
    }

    std::tuple<bool, T*> GetTopObjIf(std::function<bool(const T*)> f)
    {
        auto top = rb_first(&m_rbtree);
        if (top)
        {
            if (f((T*)top))
            {
                return std::make_tuple(true, (T*)top);
            }
        }
        return std::make_tuple(false, nullptr);
    }

    std::tuple<bool, T*> GetTopObjIfAndDelete(std::function<bool(const T*)> f)
    {
        auto top = rb_first(&m_rbtree);
        if (top)
        {
            if (f((T*)top))
            {
                DeleteObj(top);
                return std::make_tuple(true, (T*)top);
            }
        }
        return std::make_tuple(false, nullptr);
    }

    void ExecuteAll(std::function<bool(T*)> fun)
    {
        for (auto node = rb_first(&m_rbtree); node;)
        {
            struct rb_node* next = rb_next(node);
            if (fun((T*)node))
                break;
            node = next;
        }
    }

    std::tuple<bool, T*> SearchObj(std::function<bool(const T*)> f)
    {
        struct rb_node* node;
        for (node = rb_first(&m_rbtree); node; node = rb_next(node))
        {
            if (f((T*)node))
            {
                return std::make_tuple(true, (T*)node);
            }
        }
        return std::make_tuple(false, nullptr);
    }

private:
    rb_root m_rbtree = {nullptr};
};
}  // namespace v3
}  // namespace rbtreewrap
