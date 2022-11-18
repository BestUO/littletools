#ifndef _TEST_H_
#define _TEST_H_

#include <iostream>
#include <json/json.h>

using namespace std;

struct sqlconnect{
    // "conf//config.json"
    std::string alarm;
    std::string user;
    std::string password;
    std::string host;
    std::string db;
    int db_port;
    int db_timeout;
};
struct redis_account{
    std::string host;
    std::string port;
    std::string password;
};
class settingParser
{
public:
    sqlconnect GetSettinghParser(std::string filepath);
    redis_account  GetRedisSetting(std::string filepath);
    int GetSleepTime(std::string filepath);
};

//。。。

#endif