#ifndef _TEST_H_
#define _TEST_H_


#include <iostream>
#include <json/json.h>

using namespace std;
class settingParser
{
    public:
    std::string GetSettinghParser(std::string target);

    std::string GetMysqlAccount();
    
    std::string GetMysqlPassord();
    
};

//。。。

#endif