
#include "cinatra.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include <vector>
#include <string>
#include "GetCallRecord.h"
#include "UpdateCalllog.h"
#include "../sqlcommand/updatedb.h"
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

void UpdateCalllog::handleSql(ormpp::dbng<ormpp::mysql> &mysql, std::string &&s)
{
  LOGGER->info("message #{}", s);

  s = "{\"status\":0,\"info\":\"请求成功\",\"data\":{\"enterprise_type\":1,\"record_url\":\"https:\\/\\/ks3-cn-beijing.ksyun.com\\/yimi-record-1-year\\/1000_00020a4f_918705532687_10001_10000000001_2_20220407_145607_145615_8_1344299.mp3?Expires=1649321794&response-content-type=audio%2Fmp3&KSSAccessKeyId=AKLT0ChC2dqZRK6viMM8BwGZLQ&Signature=7qezhD2bmtkGFWWxueSflOI2S%2BE%3D\",\"records\":[{\"call_type\":\"1\",\"call_state\":\"15\",\"cc_number\":\"1649314541655804conf_1649314555346\",\"dialing\":\"1000_00020a4f\",\"incoming\":\"918705532687\",\"confirm_timestamp\":\"1649314567\",\"start_time\":\"1649314555\",\"end_time\":\"1649314575\",\"duration_time\":\"8\",\"record_filename\":\"1000_00020a4f_918705532687_10001_10000000001_2_20220407_145607_145615_8.mp3\",\"record_status\":\"3\",\"res_token\":null,\"enqueue_timestamp\":\"0\",\"send_query_msg_timestamp\":\"0\",\"recv_answer_msg_timestamp\":\"0\",\"send_invite_timestamp\":\"1649314558\",\"static_time_index\":\"1649314800\",\"switch_number\":\"02566203172\",\"asr_state\":\"\",\"final_ring_time\":9,\"call_result\":1}]}}";

  CallRecord record;
  CallInfo callog = record.GetCallRecord(s, 2);
  updateOutCallClue(mysql,callog);
  updateOutCallClue(mysql,callog);
  updateAiCalllogExtension(mysql,callog);
}

// std::cout << callog.call_state << std::endl;

// std::vector<std::string> columns = {"transfer_number", "transfer_duration"};
// std::vector<std::string> values = {"callog.transfer_number", std::to_string(callog.transfer_duration)};
// std::vector<std::string> condition(1);
// condition[0] = "301992";
// std::vector<std::string> condition_name(1);
// condition_name[0] = "id";
// std::vector<std::string> condition_symbols(1);
// condition_symbols[0] = " = ";

// std::string sql_command = MysqlUpdateCommand(" calllog ", values, columns, condition, condition_name, condition_symbols);
// LOGGER->info(" sql_command is ", sql_command);
// std::cout << sql_command;
// if (mysql.execute(sql_command))
// {
//   LOGGER->info(" update success ");
//   std::cout << "success " << std::endl;
// }
// else
// {
//   LOGGER->info(" update failed ");
//   std::cout << "faile" << std::endl;
// }

// // auto res = mysql.query<aicall_tts_file_cache>("id = 5622");
// // for(auto& file : res)
// //     std::cout<<file.id<<" "<<file.TTS_text<<" "<<file.TTS_version_code<<std::endl;

// // auto res = mysql.query<aicall_tts_file_cache>("id = 5622");
// // for(auto& file : res)
// //     std::cout<<file.id<<" "<<file.TTS_text<<" "<<file.TTS_version_code<<std::endl;

void UpdateCalllog::updateCalllog(ormpp::dbng<ormpp::mysql> &mysql,CallInfo calllog)
{

  std::cout << calllog.call_state << std::endl;

  std::vector<std::string> columns = {"duration", "transfer_number", "transfer_duration", "call_record_url", "transfer_manual_cost", "call_state", "switch_number"};
  std::string call_result = "";
  std::string transfer_manual_cost = "";

  std::vector<std::string> values = {std::to_string(calllog.duration_time), calllog.transfer_number, std::to_string(calllog.transfer_duration), calllog.record_url, transfer_manual_cost, std::to_string(calllog.call_state), calllog.switch_number};
  std::vector<std::string> condition(1);
  condition[0] = "301992";
  std::vector<std::string> condition_name(1);
  condition_name[0] = "id";
  std::vector<std::string> condition_symbols(1);
  condition_symbols[0] = " = ";

  
  updatecommmand command;
  std::string sql_command = command.MysqlUpdateCommand(" calllog ", values, columns, condition, condition_name, condition_symbols);
  LOGGER->info(" sql_command is ", sql_command);
  std::cout << sql_command;
  if (mysql.execute(sql_command))
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
void UpdateCalllog::updateOutCallClue(ormpp::dbng<ormpp::mysql> &mysql,CallInfo calllog)
{
    std::cout << calllog.call_state << std::endl;

  std::vector<std::string> columns = {"call_result", "manual_status"};
  std::string call_result = "";
  std::string manual_status = "";
  std::vector<std::string> values = {call_result, manual_status};
  std::vector<std::string> condition(1);
  condition[0] = "301992";
  std::vector<std::string> condition_name(1);
  condition_name[0] = "id";
  std::vector<std::string> condition_symbols(1);
  condition_symbols[0] = " = ";

  
  updatecommmand command;
  std::string sql_command = command.MysqlUpdateCommand(" outcall_clue ", values, columns, condition, condition_name, condition_symbols);
  LOGGER->info(" sql_command is ", sql_command);
  std::cout << sql_command;
  if (mysql.execute(sql_command))
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
void UpdateCalllog::updateAiCalllogExtension(ormpp::dbng<ormpp::mysql> &mysql,CallInfo calllog)
{

    std::cout << calllog.call_state << std::endl;

  std::vector<std::string> columns = {"call_result", "manual_status"};
  std::string call_result = "";
  std::string manual_status = "";
  std::vector<std::string> values = {call_result, manual_status};
  std::vector<std::string> condition(1);
  condition[0] = "301992";
  std::vector<std::string> condition_name(1);
  condition_name[0] = "id";
  std::vector<std::string> condition_symbols(1);
  condition_symbols[0] = " = ";

  
  updatecommmand command;
  std::string sql_command = command.MysqlUpdateCommand(" aicall_calllog_extension ", values, columns, condition, condition_name, condition_symbols);
  LOGGER->info(" sql_command is ", sql_command);
  std::cout << sql_command;
  if (mysql.execute(sql_command))
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
