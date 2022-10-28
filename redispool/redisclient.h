#ifndef _REDIS_H_
#define _REDIS_H_

#include <redis++/redis++.h>
#include <iostream>
#include "spdlog/spdlog.h"
#include "../settingParser/settingParser.h"
#include <unordered_set>
#include <list>
#include <vector>
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)
using namespace sw::redis;

class RedisOperate
{
public:
    RedisOperate() : connection_options_(_build_options()),
                     redis(connection_options_){};
    void CacheRules(const std::string &key, const std::string &rules);
    std::string SearchRules(const std::string &str);
    void CacheData(const std::string &key, const std::string &str, int time);
    void CacheData(const std::string &key, const std::string &str);
    void DelKey(const std::string &key);
    void DelKey(const std::vector<std::string> &key);
    std::unordered_set<std::string> GetSetFromRedis(const std::string &set);
    void REMForSet(const std::string &set_name, const std::unordered_set<std::string> &values);
    void InsertSet(const std::string &set_name, const std::unordered_set<std::string> &values);
    std::vector<std::string> GetListFromRedis(const std::string &list);
    void LREMForList(const std::string &list_name, const std::vector<std::string> &values);
    void Rpush(const std::string &list_name, const std::vector<std::string> &values);
    // ~RedisOperate();
private:
    ConnectionOptions _build_options()
    {
        ConnectionOptions opts;
        settingParser redis_settting;
        redis_account conne = redis_settting.GetRedisSetting("conf/config.json");
        opts.host = conne.host;         // Required.
        opts.port = stoi(conne.port);   // Optional. The default port is 6379.
        opts.password = conne.password; // Optional. No password by default.
        return opts;
    }
    ConnectionOptions connection_options_;
    Redis redis;
};

#endif