#ifndef UPDATEDB_H
#define UPDATEDB_H
#include "../dbstruct/dbstruct.h"
#include <string>
class GenerateSql
{public:
    std::string MysqlGenerateUpdateSql(std::string db_name, std::vector<std::string> values, std::vector<std::string> columns, std::vector<std::string> condition, std::vector<std::string> condition_name, std::vector<std::string> condition_symbols);

};

#endif