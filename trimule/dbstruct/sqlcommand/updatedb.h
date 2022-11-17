#ifndef UPDATEDB_H
#define UPDATEDB_H
#include <string>

class GenerateSQL
{
    public:
    std::string MysqlGenerateUpdateSQL(std::string db_name, std::vector<std::string> values, std::vector<std::string> columns, 
                                      std::vector<std::string> condition, std::vector<std::string> condition_name, std::vector<std::string> condition_symbols);
    
    std::string MysqlGenerateSelectSQL(std::string db_name, std::vector<std::string> values, std::vector<std::string> condition,
                                         std::vector<std::string> condition_name, std::vector<std::string> condition_symbols);

    std::string MysqlGenerateMySqlCondition(const std::vector<std::string> &condition,const std::vector<std::string> &condition_name,
                                            const std::vector<std::string> &condition_symbols);

};

#endif