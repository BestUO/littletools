#include "dbstruct.h"
#include "../../settingParser/settingParser.h"

MySql* MySql::getInstance()
{
    static MySql instance;
    return &instance;
}

void MySql::connect()
{

    settingParser mysql_example;
    sqlconnect conne = mysql_example.GetSettinghParser("conf/config.json");

    mysqlclient.connect(conne.host.c_str(), conne.user.c_str(), conne.password.c_str(), conne.db.c_str());
}

MySql::~MySql()
{
    LOGGER->info("delete mysql");
}