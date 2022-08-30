#ifndef DATACACHE_H
#define DATACACHE_H
#include "cinatra.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include <vector>
#include <string>
#include <tuple>
#include "UpdateCalllog.h"
#include "../../sqlcommand/updatedb.h"
#include "../../dbstruct/dbstruct.h"
#include "ormpp/dbng.hpp"
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

void UpdateMessage::HandleSQL(std::string &s, ormpp::dbng<ormpp::mysql> &mysqlclient, const int &class_judge, const std::string &calllog_id)
{
	LOGGER->info("handle coming message {}", s);

	CallRecord record;
	int a = 2;
	CallInfo callog = record.GetCallRecord(s, a);

	std::tuple<std::string, std::string, std::string, std::string, std::string, std::string> result;
	if (class_judge == 0)
		result = GetIdFromMysql(class_judge, callog.cc_number, mysqlclient);
	else
		result = GetIdFromMysql(class_judge, calllog_id, mysqlclient);

	if (std::get<0>(result) != "")
	{
		std::string id = std::get<0>(result);
		std::string clue_id = std::get<1>(result);
		std::string task_id = std::get<2>(result);
		std::string eid = std::get<3>(result);
		std::string caller_phone = std::get<5>(result);
		UpdateCalllog(callog, id, mysqlclient);
		UpdateOutCallClue(callog, clue_id, mysqlclient);
		UpdateAiCalllogExtension(callog, id, mysqlclient);
		UpdateAicallCalllogSubsidiary(id, mysqlclient);
		std::string call_count = std::get<4>(result);
		// std::string url = std::to_string(std::get<5>(result));
		LOGGER->info("calllog_id is {},clue_id is {},task_id is {},eid is {}", id, clue_id, task_id, eid);
		std::tuple<std::string, std::string, std::string, std::string, std::string, std::string> id_cluster = std::make_tuple(id, clue_id, task_id, eid, call_count, caller_phone);
		CallBackManage data_handle;
		data_handle.CallBackHandle(callog, id_cluster, class_judge, mysqlclient);
	}
}
std::tuple<std::string, std::string, std::string, std::string, std::string, std::string> UpdateMessage::GetIdFromMysql(const int &class_judge, const std::string &condition, ormpp::dbng<ormpp::mysql> &mysqlclient)
{

	std::tuple<std::string, std::string, std::string, std::string, std::string, std::string> null_tu = std::make_tuple("", "", "", "", "", "");
	if (class_judge == 0)
	{
		std::string cc_ = R"(cc_number = ')" + condition + R"(')";
		auto res = mysqlclient.query<std::tuple<std::string, std::string, std::string, std::string, std::string, std::string>>("select id, clue_id ,task_id,enterprise_uid,call_count,caller_phone from calllog where " + cc_);
		LOGGER->info("GetIdFromMysql  use cc_number select,command is,select id, clue_id ,task_id,enterprise_uid,call_count,caller_phone from calllog where {}", cc_);
		if (res.size())
			return res[0];
	}
	else
	{
		std::string calllog_id = R"(id = ')" + condition + R"(')";
		auto res = mysqlclient.query<std::tuple<std::string, std::string, std::string, std::string, std::string, std::string>>("select id, clue_id ,task_id,enterprise_uid,call_count,caller_phone from calllog where " + calllog_id);
		LOGGER->info("GetIdFromMysql  use calllog_id select,command is,select id, clue_id ,task_id,enterprise_uid,call_count,caller_phone from calllog where {}", calllog_id);

		if (res.size())
			return res[0];
	}
	LOGGER->info("GetIdFromMysql id illegal");
	return null_tu;
}

void UpdateMessage::UpdateCalllog(CallInfo &calllog, const std::string &id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{

	std::string call_result = calllog.call_result == 0 ? "" : std::to_string(calllog.call_result);
	std::vector<std::string> columns = {"duration", "call_result", "transfer_number", "transfer_duration", "call_record_url", "manual_status", "answer_time", "hangup_time"};
	std::string manual_status = calllog.manual_type == 0 ? "" : std::to_string(calllog.manual_type);
	std::vector<std::string> values = {std::to_string(calllog.duration_time), call_result, calllog.transfer_number, std::to_string(calllog.transfer_duration), calllog.record_url, manual_status, calllog.confirm_time, calllog.end_time};
	std::vector<std::string> condition(1);
	condition[0] = calllog.cc_number;
	std::vector<std::string> condition_name(1);
	if (calllog.cc_number != "")
		condition_name[0] = "cc_number";
	else
	{
		condition_name[0] = "id";
		condition[0] = id;
	}
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" calllog ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "UpdateCalllog", mysqlclient);
	CheckAndUpdateAicallCalllogContinuousSync(calllog,id,mysqlclient);
}

void UpdateMessage::UpdateOutCallClue(CallInfo &calllog, std::string &clue_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
	std::vector<std::string> columns = {"call_result", "manual_status", "call_time", "call_duration"};

	std::string call_result = calllog.call_result == 0 ? "" : std::to_string(calllog.call_result);
	std::string manual_status = calllog.manual_type == 0 ? "" : std::to_string(calllog.manual_type);
	std::vector<std::string> values = {call_result, manual_status, calllog.start_time, std::to_string(calllog.duration_time)};
	std::vector<std::string> condition(1);
	condition[0] = clue_id;

	std::vector<std::string> condition_name(1);
	condition_name[0] = "id";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" outcall_clue ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "UpdateOutCallClue", mysqlclient);
}

void UpdateMessage::CheckAndUpdateAicallCalllogContinuousSync(CallInfo &calllog, const std::string &id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{

	auto res = mysqlclient.query<std::tuple<std::string>>("select id from  aicall_calllog_continuous_sync where id = " + id);
	if (!res.size())
	{
		LOGGER->info("calllog id {} do not  need update aicall_calllog_continuous_sync ", id);
		return;
	}


	auto intention_type_res = mysqlclient.query<std::tuple<std::string>>("select intention_type from  calllog where id = " + id);

	std::string intention_type = std::get<0>(intention_type_res[0]);

	std::string call_result = calllog.call_result == 0 ? "" : std::to_string(calllog.call_result);
	std::vector<std::string> columns = {"call_record_url", "call_result", "intention_type", "transfer_manual_cost",
										"call_state", "switch_number", "ring_duration", "hangup_type", "manual_incoming", "manual_confirm",
										"manual_disconnect", "duration", "call_time", "answer_time", "hangup_time",
										"manual_status", "transfer_number"};
	std::string manual_status = calllog.manual_type == 0 ? "" : std::to_string(calllog.manual_type);
	std::vector<std::string> values = {calllog.record_url, call_result, intention_type, calllog.transfer_manual_cost, std::to_string(calllog.call_state), 
										calllog.switch_number, calllog.ring_time, std::to_string(calllog.hangup_type), calllog.transfer_start_time, calllog.transfer_confirm_time, 
										calllog.transfer_end_time, std::to_string(calllog.duration_time), calllog.start_time, calllog.confirm_time, calllog.end_time, manual_status, calllog.transfer_number};
	std::vector<std::string> condition(1);
	condition[0] = calllog.cc_number;
	std::vector<std::string> condition_name(1);

	condition_name[0] = "id";
	condition[0] = id;
	
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" aicall_calllog_continuous_sync ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "UpdateAicallCalllogContinuousSync", mysqlclient);
}

std::string UpdateMessage::CalculateTransferManualCost(CallInfo &calllog)
{
	std::string transfer_manual_cost = "0";
	if (calllog.transfer_start_time == "0")
		transfer_manual_cost = std::to_string(stoi(calllog.transfer_end_time) - std::stoi(calllog.start_time) - calllog.duration_time);
	else if (calllog.transfer_start_time != "" && calllog.end_time != "")
		transfer_manual_cost = std::to_string(stoi(calllog.transfer_end_time) - calllog.transfer_duration - calllog.duration_time - std::stoi(calllog.confirm_time));
	return transfer_manual_cost;
}

void UpdateMessage::UpdateAiCalllogExtension(CallInfo &calllog, std::string &calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
	std::vector<std::string> columns = {"transfer_manual_cost", "ring_duration", "call_state", "switch_number", "hangup_type", "manual_incoming", "manual_confirm", "manual_disconnect"};

	// std::string transfer_manual_cost = CalculateTransferManualCost(calllog);
	std::string hangup_cause_ = calllog.hangup_type == 0 ? "" : std::to_string(calllog.hangup_type);
	std::string switch_number = calllog.switch_number;
	std::string call_state = std::to_string(calllog.call_state);
	std::vector<std::string> values = {calllog.transfer_manual_cost, calllog.ring_time, call_state, switch_number, hangup_cause_, calllog.transfer_start_time, calllog.transfer_confirm_time, calllog.transfer_end_time};
	std::vector<std::string> condition(1);
	condition[0] = calllog_id;
	std::vector<std::string> condition_name(1);
	condition_name[0] = " calllog_id ";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" aicall_calllog_extension ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "UpdateAiCalllogExtension", mysqlclient);
}

void UpdateMessage::UpdateAicallCalllogSubsidiary(const std::string &calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{

	std::vector<std::string> columns = {"update_status"};
	std::vector<std::string> values = {"1"};
	std::vector<std::string> condition(1);
	condition[0] = calllog_id;
	std::vector<std::string> condition_name(1);
	condition_name[0] = " calllog_id ";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	GenerateSQL command;
	std::string sql_command = command.MysqlGenerateUpdateSQL(" aicall_calllog_subsidiary ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "aicall_calllog_subsidiary", mysqlclient);
}
void UpdateMessage::ExecuteCommand(std::string &sql_command, std::string children_db_name, ormpp::dbng<ormpp::mysql> &mysqlclient)
{

	LOGGER->info("update command is {}", sql_command);
	if (sql_command == "no command")
		LOGGER->info("{}", children_db_name + " update failed ,no command");
	else if (mysqlclient.execute(sql_command))
		LOGGER->info("{}", children_db_name + " update success ");
	else
		LOGGER->info("{}", children_db_name + " update failed ");
}

#endif