#pragma once
#include <iostream>
#include <map>
#include <functional>
#include <type_traits>
#include <memory>
#include <vector>
#include <cassert>
#include "function_traits.hpp"

class FunctionsManager
{
public:
    static FunctionsManager* GetInstance()
    {
        static FunctionsManager instance;
        return &instance;
    }

	template<typename Function>
	bool register_handler(std::string const & name, const Function& f)
	{
        if(check_duplicate(name))
		    this->invokers_[name] = { std::bind(&invoker<Function>::apply, f, std::placeholders::_1, std::placeholders::_2) };
        else
            return false;
        return true;
	}

    template <typename Function, typename Self, typename = std::enable_if_t<std::is_member_function_pointer<Function>::value>>
    bool register_handler(std::string const & name, Function f, Self* t) 
    {
        if(check_duplicate(name))
            invokers_[name] = { std::bind(&invoker<Function>::template apply_mem<Self>, f, t, std::placeholders::_1, std::placeholders::_2) };
        else
            return false;
        return true;
    }

	template <typename T, typename ... Args>
    typename std::enable_if<!std::is_void<T>::value,T>::type
	call(const std::string& name, Args&& ... args)
	{
		auto it = invokers_.find(name);
		if (it == invokers_.end())
			return T();

		auto args_tuple = std::make_tuple(std::forward<Args>(args)...);

		// char data[sizeof(std::tuple<Args...>)];
		// std::tuple<Args...>* tp = new (data) std::tuple<Args...>;
		// *tp = args_tuple;

		// T t;
		// it->second(tp, &t);

		T t;
		it->second(&args_tuple, &t);
		return t;
	}

	template <typename T=void, typename ... Args>
    typename std::enable_if<std::is_void<T>::value,T>::type
	call(const std::string& name, Args&& ... args)
	{
		auto it = invokers_.find(name);
		if (it == invokers_.end())
			return;

		auto args_tuple = std::make_tuple(std::forward<Args>(args)...);
		it->second(&args_tuple, nullptr);
	}
private:
    FunctionsManager()=default;
    FunctionsManager(const FunctionsManager&)=delete;
    FunctionsManager(const FunctionsManager&&)=delete;
    virtual ~FunctionsManager()=default;
    FunctionsManager& operator = (const FunctionsManager&)=delete;

    bool check_duplicate(const std::string& name)
    {
        auto it = invokers_.find(name);
        if(it!=invokers_.end())
            return false;
        return true;
    }

	template<typename Function>
	struct invoker
	{
		static inline void apply(const Function& func, void* bl, void* result)
		{
			using tuple_type = typename function_traits<Function>::tuple_type;
			const tuple_type* tp = static_cast<tuple_type*>(bl);

			call(func, *tp, result);
		}

		template<typename F, typename ... Args>
		static typename std::enable_if<std::is_void<typename std::result_of<F(Args...)>::type>::value>::type
			call(const F& f, const std::tuple<Args...>& tp, void*)
		{
			call_helper(f, std::make_index_sequence<sizeof... (Args)>{}, tp);
		}

		template<typename F, typename ... Args>
		static typename std::enable_if<!std::is_void<typename std::result_of<F(Args...)>::type>::value>::type
			call(const F& f, const std::tuple<Args...>& tp, void* result)
		{
			auto r = call_helper(f, std::make_index_sequence<sizeof... (Args)>{}, tp);
			*(decltype(r)*)result = r;
		}

		template<typename F, size_t... I, typename ... Args>
		static auto call_helper(const F& f, const std::index_sequence<I...>&, const std::tuple<Args...>& tup)
		{
			return f(std::get<I>(tup)...);
		}

        template <typename Self>
        static inline void apply_mem(Function f, Self* self, void* bl, void* result)
        {
            using tuple_type = typename function_traits<Function>::bare_tuple_type;
            const tuple_type* tp = static_cast<tuple_type*>(bl);

            using return_type = typename function_traits<Function>::return_type;
            call_mem(f, self, *tp, result, std::integral_constant<bool, std::is_void<return_type>::value>{});
        }

        template<typename F, typename Self, typename ... Args>
        static void call_mem(F f, Self* self, const std::tuple<Args...>& tp, void*, std::true_type)
        {
            call_member_helper(f, self, std::make_index_sequence<sizeof...(Args)>{}, tp);
        }

        template<typename F, typename Self, typename ... Args>
        static void call_mem(F f, Self* self, const std::tuple<Args...>& tp, void* result, std::false_type)
        {
            auto r = call_member_helper(f, self, std::make_index_sequence<sizeof...(Args)>{}, tp);
            if(result)
            *(decltype(r)*)result = r;
        }

        template<typename F, typename Self, size_t... I, typename ... Args>
        static auto call_member_helper(F f, Self* self, const std::index_sequence<I...>&, const std::tuple<Args...>& tup)-> decltype((self->*f)(std::get<I>(tup)...))
        {
            return (self->*f)(std::get<I>(tup)...);
        }
	};

	std::map<std::string, std::function<void(void*, void*)>> invokers_;
};

class TTT
{
public:
    void test(int a)
    {
        std::cout << a << std::endl;
    }
    int htest(int a,int b)
    {
        return a+b;
    }
    int a =11111;
};