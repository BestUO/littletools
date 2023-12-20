#pragma once
#include <type_traits>

template <typename Type>
concept container = requires(Type container)
{
    typename std::remove_cvref_t<Type>::value_type;
    container.size();
    container.begin();
    container.end();
};

template <typename Type>
constexpr bool is_char_t
    = std::is_same_v<Type,
          signed char> || std::is_same_v<Type, char> || std::is_same_v<Type, unsigned char> || std::is_same_v<Type, wchar_t> || std::is_same_v<Type, char16_t> || std::is_same_v<Type, char32_t> || std::is_same_v<Type, char8_t>;

template <typename Type>
concept string = container<Type> && requires(Type container)
{
    requires is_char_t<typename std::remove_cvref_t<Type>::value_type>;
    container.length();
    container.data();
};

template <typename Type>
concept map_container = container<Type> && requires(Type container)
{
    typename std::remove_cvref_t<Type>::mapped_type;
};

template <typename Type>
concept set_container = container<Type> && requires
{
    typename std::remove_cvref_t<Type>::key_type;
};