#pragma once

#include "RBTree.h"
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>

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
                newPos = &((*newPos)->rb_left);
            else
                newPos = &((*newPos)->rb_right);
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
            return f(&(reinterpret_cast<Entry*>(top))->t);
        return std::nullopt;
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
                return t;
            }
        }
        return std::nullopt;
    }

    T* GetTopObjPtr()
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto top = rb_first(&__rbtree);
        if (top)
            return &(reinterpret_cast<Entry*>(top))->t;
        return nullptr;
    }

    void PopTopObj()
    {
        std::lock_guard<std::recursive_mutex> guard(__mutex);
        auto top = rb_first(&__rbtree);
        if (top)
        {
            Entry* entry = (reinterpret_cast<Entry*>(top));
            rb_erase(&(entry->rbnode), &__rbtree);
            delete entry;
            entry = nullptr;
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
                return &entry->t;
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