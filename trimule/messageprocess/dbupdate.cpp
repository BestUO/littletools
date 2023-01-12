#include "dbupdate.h"
#include "global.hpp"

std::string UpdateMessage::MysqlGenerateUpdateSQL(std::string db_name, std::vector<std::string> values, std::vector<std::string> columns, std::vector<std::string> condition, std::vector<std::string> condition_name, std::vector<std::string> condition_symbols)
{
    std::string command = "update " + db_name + " set ";
    
    int num=0;

    for (int i = 0; i < values.size(); i++)
    {
        if (values[i] != "")
        {   
            num++;
            command += " " + columns[i] + " = \"" + values[i] + "\"";
            if (i != values.size() - 1)
                command += ",";
        }
    }

    if(num!=0&&command[command.size()-1]==',')
         command.pop_back();
    
   command+=MysqlGenerateMySqlCondition(condition,condition_name,condition_symbols);

    if (num==0)
         return "no command";

    return command;
}

std::string UpdateMessage::MysqlGenerateSelectSQL(std::string db_name, std::vector<std::string> values, std::vector<std::string> condition,
                                         std::vector<std::string> condition_name, std::vector<std::string> condition_symbols)
{
    std::string command = "select ";

    for (int i = 0; i < values.size(); i++)
    {
        if (values[i] != "")
            command += values[i] + ",";
    }
    command.pop_back();
    command+= " from "+db_name ;
    command+=MysqlGenerateMySqlCondition(condition,condition_name,condition_symbols);

    LOGGER->info("command is {}", command);
    return command;
}


std::string UpdateMessage::MysqlGenerateMySqlCondition(const std::vector<std::string> &condition,const std::vector<std::string> &condition_name,
                                            const std::vector<std::string> &condition_symbols)
{
    std::string command;
     command += " where ";
    for (int i = 0; i < condition.size(); i++)
    {
        if(i!=condition.size()-1)
        {
            if (condition[i] != "")
            command += " " + condition_name[i] + " " + condition_symbols[i] + " \"" + condition[i] + "\" " ;
            command += "and";
        }
        else if (condition[i] != "")
            command += " " + condition_name[i] + " " + condition_symbols[i] + " \"" + condition[i] + "\" ";
    }
    return command;
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
	std::string manual_status = calllog.manual_type == 0 ? " 0 " : std::to_string(calllog.manual_type);
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

	std::string sql_command = MysqlGenerateUpdateSQL(" aicall_calllog_continuous_sync ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "UpdateAicallCalllogContinuousSync", mysqlclient);
}

void UpdateMessage::UpdateCalllog(CallInfo &calllog, const std::string &id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
	std::string call_result = calllog.call_result == 0 ? "" : std::to_string(calllog.call_result);
	std::vector<std::string> columns = {"duration","call_time", "call_result", "transfer_number", "transfer_duration", "call_record_url", "manual_status", "answer_time", "hangup_time"};
	std::string manual_status = calllog.manual_type == 0 ? "" : std::to_string(calllog.manual_type);
	std::vector<std::string> values = {std::to_string(calllog.duration_time),calllog.start_time, call_result, calllog.transfer_number, std::to_string(calllog.transfer_duration), calllog.record_url, manual_status, calllog.confirm_time, calllog.end_time};
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

	std::string sql_command = MysqlGenerateUpdateSQL(" calllog ", values, columns, condition, condition_name, condition_symbols);
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

	std::string sql_command = MysqlGenerateUpdateSQL(" outcall_clue ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "UpdateOutCallClue", mysqlclient);
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

	std::string sql_command = MysqlGenerateUpdateSQL(" aicall_calllog_extension ", values, columns, condition, condition_name, condition_symbols);
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

	std::string sql_command = MysqlGenerateUpdateSQL(" aicall_calllog_subsidiary ", values, columns, condition, condition_name, condition_symbols);
	ExecuteCommand(sql_command, "aicall_calllog_subsidiary", mysqlclient);
}