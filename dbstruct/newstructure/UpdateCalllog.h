
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
class UpdateCalllog
{
public:
    void handleSql(ormpp::dbng<ormpp::mysql> &mysql, std::string &s);
private: 
    void updateCalllog( ormpp::dbng<ormpp::mysql> &mysql,CallInfo callog);
    void updateOutCallClue( ormpp::dbng<ormpp::mysql> &mysql,CallInfo callog,std::string clue_id);
    void updateAiCalllogExtension(ormpp::dbng<ormpp::mysql> &mysql, CallInfo callog,std::string calllog_id);
};