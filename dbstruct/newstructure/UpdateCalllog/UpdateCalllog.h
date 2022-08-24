#ifndef UPDATE_H
#define UPDATE_H

#include "cinatra.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include <vector>
#include <string>
#include "../GetCallRecord/GetCallRecord.h"
#include "../CallBack/CallBack.h"
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

class UpdateMessage
{
public:
    void HandleSQL(std::string &s,ormpp::dbng<ormpp::mysql> &mysqlclient,const int &class_judge = 0,const std::string &calllog_id="");
    std::tuple<std::string, std::string,std::string,std::string,std::string,std::string> GetIdFromMysql(const int &class_judge,const std::string &condition,ormpp::dbng<ormpp::mysql> &mysqlclient);
private: 
    void UpdateCalllog(CallInfo callog,const std::string &id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    void UpdateOutCallClue(CallInfo callog,std::string clue_id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    void UpdateAiCalllogExtension(CallInfo callog,std::string calllog_id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    void UpdateAicallCalllogSubsidiary(const std::string &calllog_id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    void ExecuteCommand(std::string &s,std::string children_db_name,ormpp::dbng<ormpp::mysql> &mysqlclient);
    // int  NewGetHangupCauseFromCallRecord(CallInfo info);
    // int  GetCallResult(int cause); 
    std::string CalculateTransferManualCost(CallInfo callog);


};

#endif