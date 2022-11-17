#pragma once

#include <unordered_map>
#include <list>
#include <cstddef>
#include <stdexcept>
#include <shared_mutex>
#include <chrono>
#include <optional>

template<typename K, typename V>
class LRUCache 
{
    struct Node
    {
        std::chrono::system_clock::time_point create_time = std::chrono::system_clock::now();
        V value;
        Node(V value):value(value){}
    };
public:
	typedef typename std::pair<K, Node> key_value_pair_t;
	typedef typename std::list<key_value_pair_t>::iterator list_iterator_t;

	LRUCache(size_t max_size=100, size_t remain_days=1) :__max_size(max_size),__remain_days(remain_days) {}

    void SetMaxSizeAndRemainDay(size_t max_size=100, size_t remain_days=1)
    {
        __max_size = max_size;
        __remain_days = remain_days;
    }
	
	void StoreKeyValue(const K& key, const V& value) 
    {
        std::unique_lock<std::shared_mutex> lock(__rwlock);
		auto it = __cache_items_map.find(key);
		__cache_items_list.push_front(key_value_pair_t(key, Node(value)));
		if (it != __cache_items_map.end()) 
        {
			__cache_items_list.erase(it->second);
			__cache_items_map.erase(it);
		}
		__cache_items_map[key] = __cache_items_list.begin();
		
		while (__cache_items_map.size() > __max_size) 
        {
			auto last = __cache_items_list.end();
			last--;
            if(std::chrono::system_clock::now() - last->second.create_time > std::chrono::hours(24*__remain_days))
            {
                __cache_items_map.erase(last->first);
                __cache_items_list.pop_back();
            }
            else
                break;
		}
	}
	
	std::optional<V> GetValue(const K& key) 
    {
        std::unique_lock<std::shared_mutex> lock(__rwlock);
		auto it = __cache_items_map.find(key);
		if (it == __cache_items_map.end())
			return std::nullopt;
		else 
        {
            if(std::chrono::system_clock::now() - it->second->second.create_time > std::chrono::hours(24))
            {
                it->second->second.create_time = std::chrono::system_clock::now();
                __cache_items_list.splice(__cache_items_list.begin(), __cache_items_list, it->second);
            }
			
			return it->second->second.value;
		}
	}
	
	bool IsKeyExists(const K& key) const 
    {
        std::shared_lock<std::shared_mutex> lock(__rwlock);
		return __cache_items_map.find(key) != __cache_items_map.end();
	}
	
	size_t GetLRUCacheSize() const 
    {
        std::shared_lock<std::shared_mutex> lock(__rwlock);
		return __cache_items_map.size();
	}

    void DeleteKey(const K& key)
    {
        std::unique_lock<std::shared_mutex> lock(__rwlock);
        auto it = __cache_items_map.find(key);
        if (it != __cache_items_map.end()) 
        {
			__cache_items_list.erase(it->second);
			__cache_items_map.erase(it);
		}
    }
	
private:
    std::shared_mutex __rwlock;
	std::list<key_value_pair_t> __cache_items_list;
	std::unordered_map<K, list_iterator_t> __cache_items_map;
	size_t __max_size;
    size_t __remain_days;
};