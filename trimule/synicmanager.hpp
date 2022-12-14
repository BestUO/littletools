#pragma once
#include <unordered_map>
#include <mutex>
#include <shared_mutex>
#include <optional>
class SynicManager
{
public:
    static SynicManager *GetInstance()
    {
        static SynicManager instance;
        return &instance;
    }
    std::optional<std::string_view> GetFromSynicMap(std::string_view key)
    {
        std::shared_lock<std::shared_mutex> lock(__rwlock);
        if(auto tmp = __synic_map.find(key.data());tmp != __synic_map.end())
            return __synic_map[key.data()];
        else
            return std::nullopt;
    }
    void InsertToSynicMap(std::string_view key, std::string_view value)
    {
        std::unique_lock<std::shared_mutex> lock(__rwlock);
        __synic_map[key.data()]=value.data();
    }
    void DeleteFromSynicMap(std::string_view key)
    {
        std::unique_lock<std::shared_mutex> lock(__rwlock);
        __synic_map.erase(key.data());
    }
private:
    std::unordered_map<std::string,std::string> __synic_map;
    std::shared_mutex __rwlock;

    SynicManager()=default;
    ~SynicManager()=default;
};