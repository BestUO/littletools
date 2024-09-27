#pragma once
#include <functional>
#include "objectpool.hpp"

template <typename T>
class SimpleList
{
public:
    SimpleList() = default;
    SimpleList(SimpleList&& simple_list)
    {
        __head             = simple_list.__head;
        simple_list.__head = nullptr;
    }

    struct Node
    {
        Node* next = nullptr;
        Node* prev = nullptr;
        T* data    = nullptr;
    };

    Node* AddNode(T* data)
    {
        Node* node = ObjectPool<Node>::GetInstance()->GetObject(
            nullptr, nullptr, nullptr);
        node->data = data;

        return AddNode(node);
    }

    Node* AddNode(Node* node)
    {
        node->prev = nullptr;
        node->next = nullptr;
        if (!__head)
            __head = node;
        else
        {
            node->next   = __head;
            __head->prev = node;
            __head       = node;
        }
        return node;
    }

    void RemoveNode(Node* node)
    {
        if (__head == node)
            __head = node->next;
        if (node->prev)
            node->prev->next = node->next;
        if (node->next)
            node->next->prev = node->prev;
        ObjectPool<Node>::GetInstance()->PutObject(node);
    }

    void RemoveNode(T* data)
    {
        Node* node = __head;
        while (node)
        {
            if (node->data == data)
            {
                RemoveNode(node);
                break;
            }
            node = node->next;
        }
    }

    void RemoveNode(std::function<bool(T* data)> find_and_clear_fun)
    {
        Node* node_next = __head;
        Node* node;
        while (node_next)
        {
            node      = node_next;
            node_next = node_next->next;
            if (find_and_clear_fun && find_and_clear_fun(node->data))
                RemoveNode(node);
        }
    }

    Node* PopHead()
    {
        Node* node_ptr = __head;
        if (__head)
        {
            __head = __head->next;
            if (__head)
                __head->prev = nullptr;
        }
        return node_ptr;
    }

private:
    Node* __head = nullptr;
};