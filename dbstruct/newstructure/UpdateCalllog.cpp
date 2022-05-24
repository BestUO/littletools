
#include "cinatra.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include <vector>
#include <string>
#include <tuple>
#include "GetCallRecord.h"
#include "UpdateCalllog.h"
#include "../sqlcommand/updatedb.h"
#include "../dbstruct/dbstruct.h"
#include "ormpp/dbng.hpp"
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

void UpdateMessage::HandleSQL(ormpp::dbng<ormpp::mysql> &mysql, std::string &s)
{
	LOGGER->info("handle coming message {}", s);

	CallRecord record;
	CallInfo callog = record.GetCallRecord(s, 2);

	if (callog.cc_number != "")
	{
		LOGGER->info("update calllog,cc_number is {}", callog.cc_number);
		std::string cc_ = R"(cc_number = ')" + callog.cc_number + R"(')";

		auto result = mysql.query<std::tuple<int, int>>("select id, clue_id from calllog where " + cc_);
		if (result.size())
		{
			std::string id = std::to_string(std::get<0>(result[0]));
			std::string clue_id = std::to_string(std::get<1>(result[0]));

			UpdateCalllog(mysql, callog);
			UpdateOutCallClue(mysql, callog, clue_id);
			UpdateAiCalllogExtension(mysql, callog, id);
		}
	}
	else
	{
		LOGGER->info("cc_number is null ,cannot update calllog...");
	}
}

void UpdateMessage::UpdateCalllog(ormpp::dbng<ormpp::mysql> &mysql, CallInfo calllog)
{
	std::vector<std::string> columns = {"duration", "transfer_number", "transfer_duration", "call_record_url","manual_status"};
	std::string call_result = std::to_string(calllog.stop_reason == 0 ? calllog.stop_reason: calllog.customer_fail_reason);
	std::vector<std::string> values = {std::to_string(calllog.duration_time), calllog.transfer_number, std::to_string(calllog.transfer_duration), calllog.record_url,std::to_string(calllog.manual_type)};
	std::vector<std::string> condition(1);
	condition[0] = calllog.cc_number;
	std::vector<std::string> condition_name(1);
	condition_name[0] = "cc_number";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" calllog ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(mysql,sql_command,"UpdateCalllog");
}
int UpdateMessage::NewGetHangupCauseFromCallRecord(CallInfo info) {
    int cause = 3;
    if (info.call_type == -1 || info.end_time == "0") {
        return 0;
    } else if (info.customer_fail_reason == 25)
        cause = 1;
    else if (info.stop_reason == 9 || info.stop_reason == 10 || info.stop_reason == 11 || info.stop_reason == 27 ||
             info.stop_reason == 28 || info.stop_reason == 29 || info.stop_reason == 31 || info.stop_reason == 33 ||
             info.stop_reason == 34 || info.stop_reason == 26 )
        cause = 2;
    return cause;
}

int UpdateMessage::GetCallResult(int hangup_cause_) 
{
    if(hangup_cause_ == 3)
        return 3;
    else
        return 2;
}

void UpdateMessage::UpdateOutCallClue(ormpp::dbng<ormpp::mysql> &mysql, CallInfo calllog, std::string clue_id)
{
	std::vector<std::string> columns = {"call_result", "manual_status"};
	int hangup_cause_ = NewGetHangupCauseFromCallRecord(calllog);
	// std::string call_result = std::to_string(calllog.stop_reason != 0 ? calllog.stop_reason: calllog.customer_fail_reason);
	std::string call_result = std::to_string(GetCallResult(hangup_cause_));
	std::string manual_status = std::to_string(calllog.manual_type);
	std::vector<std::string> values = {call_result, manual_status};
	std::vector<std::string> condition(1);
	condition[0] = clue_id;

	std::vector<std::string> condition_name(1);
	condition_name[0] = "id";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" outcall_clue ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(mysql,sql_command,"UpdateOutCallClue");
}

void UpdateMessage::UpdateAiCalllogExtension(ormpp::dbng<ormpp::mysql> &mysql, CallInfo calllog, std::string calllog_id)
{
	std::vector<std::string> columns = {"transfer_manual_cost", "call_state", "switch_number"};

	std::string transfer_manual_cost = (calllog.transfer_start_time!=""&&calllog.end_time!="")==true?std::to_string(stoi(calllog.transfer_start_time)-stoi(calllog.end_time)):"0";
	
	std::string switch_number = calllog.switch_number;
	std::string call_state = std::to_string(calllog.call_state);
	std::vector<std::string> values = {transfer_manual_cost, call_state, switch_number};
	std::vector<std::string> condition(1);
	condition[0] = calllog_id;
	std::vector<std::string> condition_name(1);
	condition_name[0] = " calllog_id ";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" aicall_calllog_extension ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(mysql,sql_command,"UpdateAiCalllogExtension");
}
 void UpdateMessage::ExecuteCommand(ormpp::dbng<ormpp::mysql> &mysql, std::string &sql_command,std::string children_db_name){
    
	LOGGER->info(" sql_command is {}", sql_command);
	
	if (sql_command == "no command")
		LOGGER->info("{}",children_db_name+" update failed ,no command");
	else if (mysql.execute(sql_command))
		LOGGER->info("{}",children_db_name+" update success ");
	else
		LOGGER->info("{}",children_db_name+" update failed ");

 }