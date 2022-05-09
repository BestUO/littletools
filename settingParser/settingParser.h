#ifndef _TEST_H_
#define _TEST_H_

#include <iostream>
#include <json/json.h>

using namespace std;
class settingParser
{
public:
    std::string GetSettinghParser(std::string target);

    std::string GetMysqlUser();

    std::string GetMysqlPassord();

    std::string GetMysqlHost();

    std::string GetMysqlDb();
};

//。。。

#endif