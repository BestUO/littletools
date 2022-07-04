#include "redisclient.h"


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


std::string RedisOperate::SearchRules(const std::string &str)
{
    auto value = redis.get(str);
    if(value)
        return *value;
    else return "null";
}

RedisOperate* RedisOperate::getInstance()
{
    static RedisOperate instance;
    return &instance;
}

RedisOperate::~RedisOperate()
{
   LOGGER->info("RedisOperate::~RedisOperate()");
}