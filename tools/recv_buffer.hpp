#pragma once
#include <cstddef>
#include <cstring>
#include <string>

namespace BUFFER
{

enum RecvBufferError
{
    RING_BUFFER_OK = 0,
    RING_BUFFER_FULL,
    RING_BUFFER_EMPTY,
    RING_BUFFER_INVALID_SIZE,
};

inline namespace V1
{

template <size_t N = 65536>
class RecvBuffer
{
public:
    operator std::string_view() const
    {
        return std::string_view(__buffer + __head, __tail - __head);
    }

    RecvBufferError Append(const char* data, size_t size)
    {
        if (!data || size == 0 || size > N)
        {
            return RecvBufferError::RING_BUFFER_INVALID_SIZE;
        }
        auto current_data_size = Size();
        if (current_data_size + size <= N)
        {
            if (__tail + size > N)
            {
                memcpy(__buffer, __buffer + __head, current_data_size);
                __head = 0;
                __tail = current_data_size;
            }
            memcpy(__buffer + __tail, data, size);
            __tail += size;
            return RecvBufferError::RING_BUFFER_OK;
        }
        else
        {
            return RecvBufferError::RING_BUFFER_FULL;
        }
    }

    std::string_view ConsumeAndGetString(size_t len)
    {
        len           = (len > __tail - __head) ? __tail - __head : len;
        auto old_head = __head;
        __head += len;
        if (__head == __tail)
        {
            __head = 0;
            __tail = 0;
        }
        return std::string_view(__buffer + old_head, len);
    }

    size_t Size() const
    {
        return __tail - __head;
    }

private:
    size_t __head = 0;
    size_t __tail = 0;
    char __buffer[N];
};
}  // namespace V1

namespace V2
{
template <size_t N = 65536>
class RecvBuffer
{
public:
    RecvBuffer()
    {
        __buffer.reserve(N);
    }

    operator std::string_view() const
    {
        return std::string_view(
            __buffer.data() + __head, __buffer.size() - __head);
    }

    RecvBufferError Append(const char* data, size_t size)
    {
        if (!data || size == 0 || size > N)
        {
            return RecvBufferError::RING_BUFFER_INVALID_SIZE;
        }

        RecvBufferError result = RecvBufferError::RING_BUFFER_INVALID_SIZE;

        if (Size() + size <= N)
        {
            if (__buffer.size() + size > N)
            {
                __buffer.erase(0, __head);
                __head = 0;
            }
            __buffer.append(data, size);
            result = RecvBufferError::RING_BUFFER_OK;
        }
        else
        {
            result = RecvBufferError::RING_BUFFER_FULL;
        }

        return result;
    }

    std::string ConsumeAndGetString(size_t len)
    {
        auto result = __buffer.substr(__head, len);
        if (__head + len >= __buffer.size())
        {
            __head = 0;
            __buffer.clear();
        }
        else
        {
            __head += result.size();
        }
        return result;
    }

    size_t Size() const
    {
        return __buffer.size() - __head;
    }

private:
    size_t __head = 0;
    std::string __buffer;
};
}  // namespace V2
}  // namespace BUFFER