
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

void UpdateCalllog::handleSql(ormpp::dbng<ormpp::mysql> &mysql, std::string &s)
{
	LOGGER->info("{}", s);

	CallRecord record;
	CallInfo callog = record.GetCallRecord(s, 2);

	if (callog.cc_number != "")
	{
		LOGGER->info("update calllog,cc_number is ", callog.cc_number);
		std::string cc_ = R"(cc_number = ')"+callog.cc_number+R"(')";
		
		std::cout << cc_ << std::endl;

		auto result = mysql.query<std::tuple<int, int>>("select id, clue_id from calllog where " + cc_);
		if (result.size())
		{
			std::string id = std::to_string(std::get<0>(result[0]));
			std::string clue_id = std::to_string(std::get<1>(result[0]));

			updateCalllog(mysql, callog);
			updateOutCallClue(mysql, callog, clue_id);
			updateAiCalllogExtension(mysql, callog, id);
		}
	}
	else
	{
		LOGGER->info("cc_number is null ,cannot update calllog...");
	}
}

void UpdateCalllog::updateCalllog(ormpp::dbng<ormpp::mysql> &mysql, CallInfo calllog)
{
	std::vector<std::string> columns = {"duration", "transfer_number", "transfer_duration", "call_record_url", "transfer_manual_cost"};
	std::string call_result = "";
	std::string transfer_manual_cost = "";

	std::vector<std::string> values = {std::to_string(calllog.duration_time), calllog.transfer_number, std::to_string(calllog.transfer_duration), calllog.record_url, transfer_manual_cost};
	std::vector<std::string> condition(1);
	condition[0] = calllog.cc_number;
	std::vector<std::string> condition_name(1);
	condition_name[0] = "cc_number";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	updatecommmand command;
	std::string sql_command = command.MysqlUpdateCommand(" calllog ", values, columns, condition, condition_name, condition_symbols);
	LOGGER->info("sql_command is ", sql_command);
	if (sql_command == "no command")
		LOGGER->info("update failed ,no command");
	else if (mysql.execute(sql_command))
		LOGGER->info("update sql success ");
	else
		LOGGER->info("update failed ");

}
void UpdateCalllog::updateOutCallClue(ormpp::dbng<ormpp::mysql> &mysql, CallInfo calllog, std::string clue_id)
{
	std::vector<std::string> columns = {"call_result", "manual_status"};
	std::string call_result = "";
	std::string manual_status = "";
	std::vector<std::string> values = {call_result, manual_status};
	std::vector<std::string> condition(1);
	
	condition[0] = clue_id;

	std::vector<std::string> condition_name(1);
	condition_name[0] = "id";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	updatecommmand command;
	std::string sql_command = command.MysqlUpdateCommand(" outcall_clue ", values, columns, condition, condition_name, condition_symbols);
	LOGGER->info(" sql_command is ", sql_command);
	if (sql_command == "no command")
	{
		LOGGER->info(" update failed ,no command");
	}
	else if (mysql.execute(sql_command))
	{
		LOGGER->info(" update success ");
	}
	else
	{
		LOGGER->info(" update failed ");
	}
}
void UpdateCalllog::updateAiCalllogExtension(ormpp::dbng<ormpp::mysql> &mysql, CallInfo calllog, std::string calllog_id)
{
	std::vector<std::string> columns = {"transfer_manual_cost", "call_state", "switch_number"};
	std::string transfer_manual_cost = "";
	std::string switch_number = "";
	std::string call_state = "";
	std::vector<std::string> values = {transfer_manual_cost, call_state, switch_number};
	std::vector<std::string> condition(1);
	condition[0] = calllog_id;
	std::vector<std::string> condition_name(1);
	condition_name[0] = " calllog_id ";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	updatecommmand command;
	std::string sql_command = command.MysqlUpdateCommand(" aicall_calllog_extension ", values, columns, condition, condition_name, condition_symbols);
	LOGGER->info(" sql_command is ", sql_command);
	if (sql_command == "no command")
	{
		LOGGER->info(" update failed ,no command");
	}
	else if (mysql.execute(sql_command))
	{
		LOGGER->info(" update success ");
	}
	else
	{
		LOGGER->info(" update failed ");
	}
}
