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
    void HandleSQL(std::string &s,const bool &class_judge = 0,const std::string &calllog_id="");
    std::tuple<std::string, std::string,std::string,std::string,std::string> GetIdFromMysql(const bool &class_judge,const std::string &condition);
private: 
    void UpdateCalllog(CallInfo callog);
    void UpdateOutCallClue(CallInfo callog,std::string clue_id);
    void UpdateAiCalllogExtension(CallInfo callog,std::string calllog_id);
    void ExecuteCommand(std::string &s,std::string children_db_name);
    // int  NewGetHangupCauseFromCallRecord(CallInfo info);
    // int  GetCallResult(int cause); 
    std::string CalculateTransferManualCost(CallInfo callog);


};

#endif