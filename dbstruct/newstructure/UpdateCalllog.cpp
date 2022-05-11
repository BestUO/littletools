
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
	LOGGER->info("message #{}", s);

	s = "{\"code\":200,\"info\":\"Success\",\"data\":{\"enterprise_type\":3,\"record_url\":\"https:\\/\\/wangpanold.emic.com.cn:1056\\/User\\/Api\\/getFile?access_token=945dab494348f629308261a6995ef7441650785401UtlYqYVE&res_token=f535e430374a7f2582303cce91554d3c1650785986NpyYpFJA&sign=5537b9c5cbc4df4b22d61e5375ba21c4&expires=1650787790\",\"records\":[{\"id\":11248088,\"seid\":34,\"ccgeid\":754,\"gid\":1000003935,\"uid\":1000279547,\"cc_number\":\"cc_ai_100_1000_00020b31_1650785955_947775322_1650785955750073\",\"cr_detail_id\":0,\"call_type\":\"1\",\"call_state\":15,\"dialing\":\"1000279547_02566699740\",\"confirm_timestamp\":1650785956,\"switch_number\":\"02566699740\",\"initiate_time\":false,\"valid_duration\":\"21\\u79d2\",\"valid_duration_m\":\"1\\u5206\",\"valid_duration_6s\":\"24\\u79d2\",\"duration\":\"21\\u79d2\",\"duration_m\":\"1\\u5206\",\"duration_6s\":\"24\\u79d2\",\"duration_time\":\"21\",\"start_time\":1650785956,\"end_time\":1650785986,\"stop_reason\":31,\"customer_fail_reason\":\"0\",\"incoming\":\"13072592950\",\"group_name\":\"\\u673a\\u5668\\u4eba\\u6280\\u80fd\\u7ec4\",\"seat_displayname\":\"Bot1000\",\"seat_number\":\"1000\",\"device_number\":\"1000\",\"seat_work_number\":\"Rw1000\",\"created_at\":\"2022-04-24 15:39:46\",\"updated_at\":\"2022-04-24 15:39:46\"},{\"id\":11248089,\"seid\":34,\"ccgeid\":754,\"gid\":1000003941,\"uid\":1000260385,\"cc_number\":\"cc_ai_100_1000_00020b31_1650785955_947775322_1650785955750073\",\"cr_detail_id\":1,\"call_type\":\"3\",\"call_state\":15,\"dialing\":\"1000260385_02566699740\",\"confirm_timestamp\":1650785977,\"switch_number\":\"02566699740\",\"initiate_time\":false,\"valid_duration\":\"0\\u79d2\",\"valid_duration_m\":\"0\\u5206\",\"valid_duration_6s\":\"0\\u79d2\",\"duration\":\"9\\u79d2\",\"duration_m\":\"1\\u5206\",\"duration_6s\":\"12\\u79d2\",\"duration_time\":\"9\",\"start_time\":1650785977,\"end_time\":1650785986,\"stop_reason\":25,\"customer_fail_reason\":\"0\",\"incoming\":\"13072592950\",\"group_name\":\"\\u674e\\u5a77\\u6d4b\\u8bd5\\u6280\\u80fd\\u7ec4\",\"seat_displayname\":\"\\u674e\\u5a77\",\"seat_number\":\"1002\",\"device_number\":\"1002\",\"seat_work_number\":\"1002\",\"created_at\":\"2022-04-24 15:39:46\",\"updated_at\":\"2022-04-24 15:39:46\"}]}}";
	CallRecord record;
	CallInfo callog = record.GetCallRecord(s, 2);

	std::cout << callog.enterprise_type << callog.duration_time << std::endl;

	std::cout << "callog.cc_number is " << callog.cc_number << std::endl;
	if (callog.cc_number != "")
	{
		LOGGER->info("update calllog,cc_number is ", callog.cc_number);
		std::string cc_ = R"(cc_number = '1582769702983860conf_1582769746138')";
		std::cout << cc_ << std::endl;

		auto result = mysql.query<std::tuple<int, int>>("select id, clue_id from calllog where " + cc_);

		std::cout << result.size() << std::endl;
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

	std::cout << calllog.call_state << std::endl;

	std::vector<std::string> columns = {"duration", "transfer_number", "transfer_duration", "call_record_url", "transfer_manual_cost"};
	std::string call_result = "";
	std::string transfer_manual_cost = "";

	std::vector<std::string> values = {std::to_string(calllog.duration_time), calllog.transfer_number, std::to_string(calllog.transfer_duration), calllog.record_url, transfer_manual_cost};
	std::vector<std::string> condition(1);
	condition[0] = "1582769702983860conf_1582769746138";
	// condition[0] = calllog.cc_number;
	std::vector<std::string> condition_name(1);
	condition_name[0] = "cc_number";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	updatecommmand command;
	std::string sql_command = command.MysqlUpdateCommand(" calllog ", values, columns, condition, condition_name, condition_symbols);
	LOGGER->info(" sql_command is ", sql_command);
	std::cout << " sql_command is " << sql_command << std::endl;
	if (sql_command == "no command")
	{
		LOGGER->info(" update failed ,no command");
		std::cout << "no command" << std::endl;
	}
	else if (mysql.execute(sql_command))
	{
		LOGGER->info(" update success ");
		std::cout << "success " << std::endl;
	}
	else
	{
		LOGGER->info(" update failed ");
		std::cout << "faile" << std::endl;
	}

	// auto res = mysql.query<aicall_tts_file_cache>("id = 5622");
	// for(auto& file : res)
	//     std::cout<<file.id<<" "<<file.TTS_text<<" "<<file.TTS_version_code<<std::endl;

	// auto res = mysql.query<aicall_tts_file_cache>("id = 5622");
	// for(auto& file : res)
	//     std::cout<<file.id<<" "<<file.TTS_text<<" "<<file.TTS_version_code<<std::endl;
}
void UpdateCalllog::updateOutCallClue(ormpp::dbng<ormpp::mysql> &mysql, CallInfo calllog, std::string clue_id)
{
	std::cout << calllog.call_state << std::endl;

	std::vector<std::string> columns = {"call_result", "manual_status"};
	std::string call_result = "2222";
	std::string manual_status = "55555";
	std::vector<std::string> values = {call_result, manual_status};
	std::vector<std::string> condition(1);
	condition[0] = "63197576";
	// condition[0] = clue_id;

	std::vector<std::string> condition_name(1);
	condition_name[0] = "id";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	updatecommmand command;
	std::string sql_command = command.MysqlUpdateCommand(" outcall_clue ", values, columns, condition, condition_name, condition_symbols);
	LOGGER->info(" sql_command is ", sql_command);
	std::cout << " sql_command is " << sql_command;
	if (sql_command == "no command")
	{
		LOGGER->info(" update failed ,no command");
		std::cout << "no command" << std::endl;
	}
	else if (mysql.execute(sql_command))
	{
		LOGGER->info(" update success ");
		std::cout << "success " << std::endl;
	}
	else
	{
		LOGGER->info(" update failed ");
		std::cout << "faile" << std::endl;
	}
}
void UpdateCalllog::updateAiCalllogExtension(ormpp::dbng<ormpp::mysql> &mysql, CallInfo calllog, std::string calllog_id)
{

	std::cout << calllog.call_state << std::endl;

	std::vector<std::string> columns = {"transfer_manual_cost", "call_state", "switch_number"};
	std::string transfer_manual_cost = " 35";
	std::string switch_number = "   switch_number ";
	std::string call_state = ""
							 "";
	std::vector<std::string> values = {transfer_manual_cost, call_state, switch_number};
	std::vector<std::string> condition(1);
	condition[0] = "301992";
	std::vector<std::string> condition_name(1);
	condition_name[0] = " calllog_id ";
	std::vector<std::string> condition_symbols(1);
	condition_symbols[0] = " = ";

	updatecommmand command;
	std::string sql_command = command.MysqlUpdateCommand(" aicall_calllog_extension ", values, columns, condition, condition_name, condition_symbols);
	LOGGER->info(" sql_command is ", sql_command);
	std::cout << " sql_command is " << sql_command;
	if (sql_command == "no command")
	{
		LOGGER->info(" update failed ,no command");
		std::cout << "no command" << std::endl;
	}
	else if (mysql.execute(sql_command))
	{
		LOGGER->info(" update success ");
		std::cout << "success " << std::endl;
	}
	else
	{
		LOGGER->info(" update failed ");
		std::cout << "faile" << std::endl;
	}
}
