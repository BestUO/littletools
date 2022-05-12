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
};
class settingParser
{
public:
    sqlconnect GetSettinghParser(std::string filepath);

};

//。。。

#endif