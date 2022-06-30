#ifndef _REDIS_H_
#define _REDIS_H_

#include <sw/redis++/redis++.h>
#include <iostream>
#include "spdlog/spdlog.h"
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)
using namespace sw::redis;

class RedisOperate
{
public:
  
    void CacheRules(const std::string &key,const std::string &rules);

private:
    Redis redis;
    void RedisConnect();
    std::string SearchRules(const std::string &str);


};

#endif