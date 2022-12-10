#ifndef UPDATE_H
#define UPDATE_H

#include "cinatra/cinatra.hpp"
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
    static void HandleSQL(std::string &message_from_cm, ormpp::dbng<ormpp::mysql> &mysqlclient,const std::string &wherecondition = "",const std::string &calllog_id="");
    static std::tuple<std::string, std::string, std::string, std::string, std::string, std::string> GetIdFromMysql(ormpp::dbng<ormpp::mysql> &mysqlclient,const std::string &wherecondition);
private: 
    static void UpdateCalllog(CallInfo &callog,const std::string &id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void UpdateOutCallClue(CallInfo &callog,std::string &clue_id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void UpdateAiCalllogExtension(CallInfo &callog,std::string &calllog_id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void CheckAndUpdateAicallCalllogContinuousSync(CallInfo &callog,const std::string &id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void UpdateAicallCalllogSubsidiary(const std::string &calllog_id,ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void ExecuteCommand(std::string &s,std::string children_db_name,ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void CheckCallResultSilence(CallInfo &callog,ormpp::dbng<ormpp::mysql> &mysqlclient,const std::string &wherecondition);//if callresult from sql is 1 ,donot update cm's callresult
    // int  NewGetHangupCauseFromCallRecord(CallInfo info);
    // int  GetCallResult(int cause); 
    static std::string CalculateTransferManualCost(CallInfo &callog);
};

#endif