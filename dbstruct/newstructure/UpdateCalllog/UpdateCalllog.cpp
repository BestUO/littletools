
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

void UpdateMessage::HandleSQL(std::string &s,const bool &class_judge)
{
	LOGGER->info("handle coming message {}", s);

	CallRecord record;
	CallInfo callog = record.GetCallRecord(s, 2);
	MySql *mysql = MySql::getInstance();
	if (callog.cc_number != "")
	{
		LOGGER->info("update calllog,cc_number is {}", callog.cc_number);
		std::string cc_ = R"(cc_number = ')" + callog.cc_number + R"(')";

		auto result = mysql->mysqlclient.query<std::tuple<int, int,int,int,int>>("select id, clue_id ,task_id,enterprise_uid,call_count from calllog where " + cc_);
		if (result.size())
		{
			std::string id = std::to_string(std::get<0>(result[0]));
			std::string clue_id = std::to_string(std::get<1>(result[0]));
			std::string task_id = std::to_string(std::get<2>(result[0]));
			std::string eid = std::to_string(std::get<3>(result[0]));


			UpdateCalllog(callog);
			UpdateOutCallClue(callog, clue_id);
			UpdateAiCalllogExtension(callog, id);
			
			std::string call_count = std::to_string(std::get<4>(result[0]));
			LOGGER->info("calllog_id is {},clue_id is {},task_id is {},eid is {}", id,clue_id,task_id,eid);
			std::tuple<std::string,std::string,std::string,std::string,std::string> id_cluster = std::make_tuple(id,clue_id,task_id,eid,call_count);
			CallBackManage data_handle;
			data_handle.CallBackHandle(callog,id_cluster,const bool &class_judge);
		}
	}
	else
	{
		LOGGER->info("cc_number is null ,cannot update calllog...");
	}

}
// int UpdateMessage::NewGetHangupCauseFromCallRecord(CallInfo info)
// {
// 	int cause = 3; // NOT_CONNECTED
// 	if (info.call_type == -1 || info.end_time == "0")
// 	{
// 		return 0; // NO_HANGUP_CAUSE
// 	}
// 	else if (info.stop_reason == 1 || info.stop_reason == 2 || info.stop_reason == 3 || info.stop_reason == 16 || info.stop_reason == 18 ||
// 			 info.stop_reason == 21 || info.stop_reason == 22 || info.stop_reason == 23 || info.stop_reason == 24 ||
// 			 info.stop_reason == 34 || info.stop_reason == 37 || info.stop_reason == 41)
// 		cause = 3; // fail
// 	else if (info.stop_reason == 25)
// 		cause = 2; // USER_HANGUP
// 	// else if (info.stop_reason == 9 || info.stop_reason == 7|| info.stop_reason == 10 || info.stop_reason == 11 || info.stop_reason == 27 ||
// 	// 		 info.stop_reason == 28 || info.stop_reason == 29 || info.stop_reason == 31 || info.stop_reason == 33 || info.stop_reason == 26)
// 	// 	cause = 1; // AI_HANGUP
// 	else
// 		cause = 1;
// 	return cause;
// }

// int UpdateMessage::GetCallResult(int hangup_cause_)
// {
// 	if (hangup_cause_ == 1 || hangup_cause_ == 2)
// 		return 2;				 //通话成功
// 	else if (hangup_cause_ == 0) //未接听
// 		return 3;
// 	else
// 		return 4;
// }
void UpdateMessage::UpdateCalllog(CallInfo calllog)
{

	std::string call_result = calllog.call_result==0?"":std::to_string(calllog.call_result);
	std::vector<std::string> columns = {"duration", "call_result", "transfer_number", "transfer_duration", "call_record_url", "manual_status","answer_time","hangup_time"};
	std::string manual_status = calllog.manual_type==0?"":std::to_string(calllog.manual_type); 
	std::vector<std::string> values = {std::to_string(calllog.duration_time), call_result, calllog.transfer_number, std::to_string(calllog.transfer_duration), calllog.record_url, manual_status,calllog.confirm_time,calllog.end_time};
	std::vector<std::string> condition(1);
	condition[0] = calllog.cc_number;
	std::vector<std::string> condition_name(1);
	condition_name[0] = "cc_number";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" calllog ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "UpdateCalllog");
}

void UpdateMessage::UpdateOutCallClue(CallInfo calllog, std::string clue_id)
{
	std::vector<std::string> columns = {"call_result", "manual_status","call_time","call_duration"};
	
	std::string call_result = calllog.call_result == 0?"":std::to_string(calllog.call_result);
	std::string manual_status = calllog.manual_type == 0?"":std::to_string(calllog.manual_type);
	std::vector<std::string> values = {call_result, manual_status,calllog.start_time,std::to_string(calllog.duration_time)};
	std::vector<std::string> condition(1);
	condition[0] = clue_id;

	std::vector<std::string> condition_name(1);
	condition_name[0] = "id";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" outcall_clue ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "UpdateOutCallClue");
}

std::string UpdateMessage::CalculateTransferManualCost(CallInfo calllog)
{
	std::string transfer_manual_cost = "0";
	if (calllog.transfer_start_time == "0")
		transfer_manual_cost = std::to_string(stoi(calllog.transfer_end_time) - std::stoi(calllog.start_time) - calllog.duration_time);
	else if (calllog.transfer_start_time != "" && calllog.end_time != "" )
		transfer_manual_cost = std::to_string(stoi(calllog.transfer_end_time) - calllog.transfer_duration - calllog.duration_time - std::stoi(calllog.confirm_time));
	return transfer_manual_cost;
}

void UpdateMessage::UpdateAiCalllogExtension(CallInfo calllog, std::string calllog_id)
{
	std::vector<std::string> columns = {"transfer_manual_cost","ring_duration", "call_state", "switch_number", "hangup_type","manual_incoming","manual_confirm","manual_disconnect"};

	// std::string transfer_manual_cost = CalculateTransferManualCost(calllog);
	std::string hangup_cause_ = calllog.hangup_type==0?"":std::to_string(calllog.hangup_type);
	std::string switch_number = calllog.switch_number;
	std::string call_state = std::to_string(calllog.call_state);
	std::vector<std::string> values = {calllog.transfer_manual_cost,calllog.ring_time,call_state, switch_number, hangup_cause_,calllog.start_time,calllog.transfer_confirm_time,calllog.end_time};
	std::vector<std::string> condition(1);
	condition[0] = calllog_id;
	std::vector<std::string> condition_name(1);
	condition_name[0] = " calllog_id ";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" aicall_calllog_extension ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "UpdateAiCalllogExtension");
}
void UpdateMessage::ExecuteCommand(std::string &sql_command, std::string children_db_name)
{
	MySql *mysql = MySql::getInstance();

	if (sql_command == "no command")
		LOGGER->info("{}", children_db_name + " update failed ,no command");
	else if (mysql->mysqlclient.execute(sql_command))
		LOGGER->info("{}", children_db_name + " update success ");
	else
		LOGGER->info("{}", children_db_name + " update failed ");
}

 