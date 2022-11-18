#include "dbstruct.h"
#include "../../settingParser/settingParser.h"

// MySql* MySql::getInstance()
// {   
//     static MySql instance;
//     return &instance;
// }

// void MySql::connect()
// {

//     settingParser mysql_example;
//     sqlconnect conne = mysql_example.GetSettinghParser("conf/trimule_config.json");

//     mysqlclient.connect(conne.host.c_str(), conne.user.c_str(), conne.password.c_str(), conne.db.c_str());
//     mysqlclient.ping();
// }

// void MySql::ReSetStatus()
// {
//     if(!mysqlclient.ping())
//     {
//         LOGGER->info("mysql out of sync ,reconnect");
//         connect();
//     }

// }

// MySql::~MySql()
// {
//     LOGGER->info("delete mysql");
// }

