#ifndef CALLBACK_H
#define CALLBACK_H

#include <iostream>
#include <string>
#include <json/json.h>
#include "../../dbstruct/dbstruct.h"
#include "../GetCallRecord.h"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "ormpp/connection_pool.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/ormpp_cfg.hpp"
#include "../../sqlcommand/updatedb.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#define TEST_MAIN
#include "ormpp/unit_test.hpp"
using namespace std::string_literals;
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include  "../../../redispool/redisclient.h"
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

struct CallBackRules{
    int task_id;
    int eid;
    
    int callback;//0:donot callback,1:callback
    int global_judge;//0 :all  1:auto_task should use rules
    int call_count;
    int auto_recall_max_times;
    int auto_recall_status;
    int scope_judge;
    std::string uuid;
    std::string api_callback_scene_status;
    std::string intention_type_judge;
    std::string call_result_judge;
    std::string auto_recall_scenes;
    CallBackRules() : task_id(0),eid(0),callback(0),
                      call_count(0),auto_recall_max_times(0),auto_recall_status(0),scope_judge(0),uuid(""),api_callback_scene_status("0")
                      ,intention_type_judge("0000000000000"),call_result_judge("00000000000"),auto_recall_scenes(""){}
};

struct CallBackData{
    //May be cm/oc  data
        std::string record_url;
        std::string answer_time;//sql confirm_timestamp
        std::string hangup_time;//end_time
        int duration_time;
        std::string transfer_number;
        int transfer_duration;
        std::string switch_number;
        int manual_status;
        std::string cc_number;
        int call_result;
        int hangup_type;
    //OC_data
        std::string	uuid;
        std::string	task_id;
        std::string	script_name;
        std::string	callee_phone;
        std::string	caller_phone;
        std::string	calllog_txt;
        std::string	intention_type;
        std::string	label;
        std::string	call_count;
        std::string	match_global_keyword;
        std::string	clue_no;
        std::string	collect_info;
        std::string	buttons;
        std::string	calllog_id;

        CallBackData() : record_url(""),answer_time(""),hangup_time(""),duration_time(0),transfer_number(""),
                         transfer_duration(0),switch_number(""),manual_status(0),cc_number(""),call_result(0),
                        hangup_type(0), uuid(""),task_id(""),script_name(""),callee_phone(""),caller_phone(""),
                        calllog_txt(""),intention_type("0"),label(""),call_count("0"),match_global_keyword(""),
                        clue_no(""),collect_info(""),buttons(""),calllog_id{}
};

struct OC_data{
        std::string	uuid;
        std::string	task_id;
        std::string	script_name;
        std::string	callee_phone;
        std::string	caller_phone;
        std::string	calllog_txt;
        std::string	intention_type;
        std::string	label;
        std::string	call_count;
        std::string	match_global_keyword;
        std::string	clue_no;
        std::string	collect_info;
        std::string	buttons;
        std::string	calllog_id;
};
REFLECTION(OC_data,uuid,task_id,script_name,callee_phone,caller_phone,calllog_txt,intention_type,label,call_count,match_global_keyword,clue_no,collect_info,buttons,calllog_id)

struct outcall_clue{
    std::string	label;
    std::string	alias;
};
REFLECTION(outcall_clue,label,alias)


struct calllog{
        std::string	task_id;
        std::string	script_name;
        std::string	callee_phone;
        std::string	caller_phone;
        std::string	calllog_txt;
        std::string	intention_type;
        std::string	call_count;
        std::string	match_global_keyword;
        std::string	collect_info;
        std::string	buttons;
        std::string	id;
};
REFLECTION(calllog,task_id,script_name,callee_phone,caller_phone,calllog_txt,intention_type,call_count,match_global_keyword,collect_info,buttons,id)

struct outcall_task{
        std::string	uuid;
};
REFLECTION(outcall_task,uuid)

enum IdCluster{
    CalllogId,
    ClueId,
    TaskId,
    EnterpriseUid
};

enum IntentionType
{
    IntentionA = 1,
    IntentionB,
    IntentionC,
    IntentionD,
    IntentionE,
    IntentionF,
    IntentionG,
    IntentionH,
    IntentionI,
    IntentionJ //J is not used, just use as sentinel
};
class CallBackManage:public CallRecord{

public:
    void CallBackHandle(ormpp::dbng<ormpp::mysql> &mysql,CallInfo & cm_data,const std::tuple<std::string,std::string,std::string,std::string> &id_cluster);
    void CmDataSwitch(CallInfo & cm_data,CallBackData &data);
    void GetOCSyncData(ormpp::dbng<ormpp::mysql> &mysql,CallBackData &data);
    void ParseIntetionAndCallResult(CallBackRules &rules);
    std::string SetMySqlRules(std::vector<std::string> rule_name,std::vector<std::string> rule);
    CallBackRules MakeCallBackRulesFromMySql(ormpp::dbng<ormpp::mysql> &mysql,const std::tuple<std::string,std::string,std::string,std::string> &id_cluster);
    std::string SetRulesRedisCache(const CallBackRules &rules);
    bool GetRulesFromRedis(CallBackRules &rules);
    bool CallBackJudge(const CallBackRules &rules,const CallBackData &data);
    bool AutoTaskMatch(const CallBackRules &rules,const CallBackData &data);
    
};      

#endif