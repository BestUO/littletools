#include "settingParser.h"
#include <iostream>
// #include <json/json.h>
#include <fstream>
using namespace std;


sqlconnect settingParser::GetSettinghParser(std::string filepath)
{

    sqlconnect connect;
    std::string str = "";
    Json::Reader reader;
    Json::Value root;

    //确认文件读取状态
    std::ifstream in(filepath, std::ios::binary);

    if (!in.is_open())
    {   connect.alarm = "file is not open!!";
        return connect;
    }

    if (reader.parse(in, root))
    {
        Json::Value mysql_setting = root["mysql_setting"];
        // Json::Value  res = mysql_setting[target];
        // str = res.asString();
        connect.db = mysql_setting["mysql_db"].asString();
        connect.user = mysql_setting["mysql_user"].asString();
        connect.password = mysql_setting["mysql_password"].asString();
        connect.host = mysql_setting["mysql_host"].asString();
        
        return connect;
    }
    connect.alarm = "other error";
    return connect;
}

redis_account  settingParser::GetRedisSetting(std::string filepath)
{
    redis_account connect;
    std::string str = "";
    Json::Reader reader;
    Json::Value root;

    //确认文件读取状态
    std::ifstream in(filepath, std::ios::binary);

    // if (!in.is_open())
    // {   connect.alarm = "file is not open!!";
    //     return connect;
    // }

    if (reader.parse(in, root))
    {
        Json::Value redis_setting = root["redis_setting"];
        // Json::Value  res = mysql_setting[target];
        // str = res.asString();

        connect.port = redis_setting["port"].asString();
        connect.password = redis_setting["password"].asString();
        connect.host = redis_setting["host"].asString();
        
        return connect;
    }
    return connect;
}