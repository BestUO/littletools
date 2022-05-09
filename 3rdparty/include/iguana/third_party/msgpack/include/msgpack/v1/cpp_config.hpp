//
// MessagePack for C++ C++03/C++11 Adaptation
//
// Copyright (C) 2013-2016 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_CPP_CONFIG_HPP
#define MSGPACK_V1_CPP_CONFIG_HPP

#include "msgpack/cpp_config_decl.hpp"

#if defined(MSGPACK_USE_CPP03)

namespace msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

template <typename T>
struct unique_ptr : std::auto_ptr<T> {
    explicit unique_ptr(T* p = 0) throw() : std::auto_ptr<T>(p) {}
    unique_ptr(unique_ptr& a) throw() : std::auto_ptr<T>(a) {}
    template<class Y>
    unique_ptr (unique_ptr<Y>& a) throw() : std::auto_ptr<T>(a) {}
};

template <typename T>
T& move(T& t)
{
    return t;
}

template <typename T>
T const& move(T const& t)
{
    return t;
}

template <bool P, typename T>
struct enable_if {
    typedef T type;
};

template <typename T>
struct enable_if<false, T> {
};

template<typename T, T val>
struct integral_constant {
    static T const value = val;
    typedef T value_type;
    typedef integral_constant<T, val> type;
};

typedef integral_constant<bool, true> true_type;
typedef integral_constant<bool, false> false_type;

template<class T, class U>
struct is_same : false_type {};

template<class T>
struct is_same<T, T> : true_type {};

template<typename T>
struct underlying_type {
    typedef int type;
};

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace msgpack

#endif // MSGPACK_USE_CPP03

#endif // MSGPACK_V1_CPP_CONFIG_HPP
