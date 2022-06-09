
#include "cinatra.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include <vector>
#include <string>
#include "GetCallRecord.h"
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)
class UpdateMessage
{
public:
    void HandleSQL(ormpp::dbng<ormpp::mysql> &mysql, std::string &s);
 
private: 
    void UpdateCalllog( ormpp::dbng<ormpp::mysql> &mysql,CallInfo callog);
    void UpdateOutCallClue( ormpp::dbng<ormpp::mysql> &mysql,CallInfo callog,std::string clue_id);
    void UpdateAiCalllogExtension(ormpp::dbng<ormpp::mysql> &mysql, CallInfo callog,std::string calllog_id);
    void ExecuteCommand(ormpp::dbng<ormpp::mysql> &mysql, std::string &s,std::string children_db_name);
    // int  NewGetHangupCauseFromCallRecord(CallInfo info);
    // int  GetCallResult(int cause); 
    std::string CalculateTransferManualCost(CallInfo callog);
};