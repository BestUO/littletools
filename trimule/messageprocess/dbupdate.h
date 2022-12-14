#pragma once

#include <string_view>
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "messageprocessstruct.hpp"

class UpdateMessage
{
public:
    static void UpdateCalllog(CallInfo &callog,const std::string &id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void UpdateOutCallClue(CallInfo &callog,std::string &clue_id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void UpdateAiCalllogExtension(CallInfo &callog,std::string &calllog_id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void UpdateAicallCalllogSubsidiary(const std::string &calllog_id,ormpp::dbng<ormpp::mysql> &mysqlclient);

private:
    static std::string MysqlGenerateUpdateSQL(std::string db_name, std::vector<std::string> values, std::vector<std::string> columns, 
                                      std::vector<std::string> condition, std::vector<std::string> condition_name, std::vector<std::string> condition_symbols);
    
    static std::string MysqlGenerateSelectSQL(std::string db_name, std::vector<std::string> values, std::vector<std::string> condition,
                                         std::vector<std::string> condition_name, std::vector<std::string> condition_symbols);

    static std::string MysqlGenerateMySqlCondition(const std::vector<std::string> &condition,const std::vector<std::string> &condition_name,
                                            const std::vector<std::string> &condition_symbols);
    static void ExecuteCommand(std::string &s,std::string children_db_name,ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void CheckAndUpdateAicallCalllogContinuousSync(CallInfo &callog,const std::string &id,ormpp::dbng<ormpp::mysql> &mysqlclient);
};