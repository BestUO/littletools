#pragma once

#include <type_traits>
#include <fcntl.h>
#include <stdint.h>
#include <concepts>
#include <string_view>

template <typename T>
concept PodType = std::is_trivial_v<T> && std::is_standard_layout_v<T>;

template <typename T>
concept DeQueElement = requires(T t) {
    { decltype(t.__prev)(), decltype(t.__next)() } -> std::same_as<int32_t>;
};

template <typename T>
concept QueueElement = requires(T t) {
    { decltype(t.__next)() } -> std::same_as<int32_t>;
};

template <PodType T>
class Element
{
public:
    T* GetData()
    {
        return (T*)__t;
    }

private:
    char __t[sizeof(T)] = {0};
};

template <typename T>
class IndexDataTypeCheck
{
public:
    int32_t __index = 0;
};

#define READER_SIZE 64
#define CIRCLE_INDEX_TYPE uint8_t
#define SHM_QUEUE_SIZE CIRCLE_INDEX_TYPE(-1) + 1

enum class Result
{
    OK,
    NOREADER,
};