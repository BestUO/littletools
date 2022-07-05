#include "redisclient.h"
#include <iostream>


RedisOperate* RedisOperate::getInstance()
{
    static RedisOperate instance;
    return &instance;
}

RedisOperate::~RedisOperate()
{
   LOGGER->info("RedisOperate::~RedisOperate()");
}
void RedisOperate::RedisConnect()
{
    ConnectionOptions connection_options;
    connection_options.host = "172.17.214.17";  // Required.
    connection_options.port = 16379; // Optional. The default port is 6379.
    connection_options.password = "greeisgood";   // Optional. No password by default.
    connection_options.db = 1;  // Optional. Use the 0th database by default.
    connection_options.socket_timeout = std::chrono::milliseconds(200);
    Redis redis1(connection_options);
    redis = std::move(redis1);
}

void RedisOperate::CacheRules(const std::string &key,const std::string &rules)
{
    redis.set(key,rules);
    redis.expire(key,std::chrono::minutes(10));
    LOGGER->info("set rules {}",rules);
}

void RedisOperate::CacheData(const std::string &key,const std::string &str,int time)
{
    redis.set(key,str);
    redis.expire(key,std::chrono::seconds(time));
    LOGGER->info("set str  {}",str);
}

void RedisOperate::CacheData(const std::string &key,const std::string &str)
{
    redis.set(key,str);
    LOGGER->info("set str  {}",str);
}

void RedisOperate::DelKey(const std::string &key)
{
    int num = redis.del(key);
    LOGGER->info("{} key delete",num);
}

void RedisOperate::DelKey(const std::vector<std::string> &key)
{
    int num  = redis.del(key.begin(),key.end());
    LOGGER->info("{} key delete",num);
}


std::string RedisOperate::SearchRules(const std::string &str)
{
    auto value = redis.get(str);
    if(value)
        return *value;
    else return "null";
}

std::unordered_set<std::string> RedisOperate::GetSetFromRedis(const std::string & set)
{
    std::unordered_set<std::string> union_ ;
    redis.smembers(set,union_);
    return union_;
}

void RedisOperate::REMForSet(const std::string &set_name,const std::unordered_set<std::string> &values)
{
    int num = redis.srem(set_name,values.begin(),values.end());
    LOGGER->info("{} elements rem set",num);
}

void RedisOperate::InsertSet(const std::string &set_name,const std::unordered_set<std::string> &values)
{
    int num = redis.sadd(set_name,values.begin(),values.end());
    LOGGER->info("{} elements insert set",num);
}