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
  
    static RedisOperate *getInstance();
    void CacheRules(const std::string &key,const std::string &rules);
    std::string SearchRules(const std::string &str);
private:
    Redis redis;
    void RedisConnect();
    
    RedisOperate();

    RedisOperate(const RedisOperate &);

    RedisOperate &operator=(const RedisOperate &);

    ~RedisOperate();


};

#endif