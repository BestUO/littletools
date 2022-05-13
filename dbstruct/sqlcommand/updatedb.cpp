
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include <vector>
#include <string>
#include "updatedb.h"
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

std::string GenerateSQL::MysqlGenerateUpdateSQL(std::string db_name, std::vector<std::string> values, std::vector<std::string> columns, std::vector<std::string> condition, std::vector<std::string> condition_name, std::vector<std::string> condition_symbols)
{
    std::string command = "update " + db_name + " set ";
    
    int num=0;
    for (int i = 0; i < values.size(); i++)
    {
        if (values[i] != "")
        {   
            num++;
            command += " " + columns[i] + " = \"" + values[i] + "\"";
            if (i != values.size() - 1)
                command += ",";
        }
    }

    if(num!=0&&command[command.size()-1]==',')
         command.pop_back();
    
    command += " where ";
    for (int i = 0; i < condition.size(); i++)
    {
        if (condition[i] != "")
            command += " " + condition_name[i] + " " + condition_symbols[i] + " \"" + condition[i] + "\" ";
    }

    if (num==0)
         return "no command";

    return command;
}
