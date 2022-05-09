#include "settingParser.h"
#include <iostream>
// #include <json/json.h>
#include <fstream>
using namespace std;

std::string settingParser::GetSettinghParser(std::string target)
{
    std::string str = "";
    Json::Reader reader;
    Json::Value root;

    //确认文件读取状态
    std::ifstream in("conf//example.json", std::ios::binary);

    if (!in.is_open())
    {
        return "Error opening file";
    }

    if (reader.parse(in, root))
    {
        Json::Value data = root["data"];
        Json::Value mysql_setting = data["mysql_setting"];
        Json::Value  res = mysql_setting[target];
        str = res.asString();
        cout<<str<<endl;
        return str;
    }
    return str;
}
// settingParser::settingParser(){
// int a = 1;
// std::cout << "aaa" << std::endl;
// };
std::string settingParser::GetMysqlAccount()
{

    return GetSettinghParser("mysql_account");
}
std::string settingParser::GetMysqlPassord()
{

    return GetSettinghParser("mysql_password");
}