#ifndef CALLBACK_H
#define CALLBACK_H
#include "cinatra.hpp"
#include <iostream>
#include <string>
#include <json/json.h>
#include "../../dbstruct/dbstruct.h"
#include "../UpdateCalllog/UpdateCalllog.h"
#include "../GetCallRecord/GetCallRecord.h"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "ormpp/connection_pool.hpp"
#include "ormpp/ormpp_cfg.hpp"
#include "../../sqlcommand/updatedb.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
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
    int global_judge;//1 :all  0:just api data
    int call_count;
    int auto_recall_max_times;
    int auto_recall_status;
    int scope_judge;//tcallback every time :0,if autorecall lasttime 1
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
        std::string eid;
        std::string clue_id;
        std::string	calllog_id;
        std::string	task_id;
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
        int call_time;
    //OC_data
        std::string	uuid;
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

        std::string call_progress;
        std::string url;
        CallBackData() : eid(""),clue_id(""),record_url(""),answer_time(""),hangup_time(""),duration_time(0),transfer_number(""),
                         transfer_duration(0),switch_number(""),manual_status(0),cc_number(""),call_result(0),
                        hangup_type(0),call_time(0), uuid(""),task_id(""),script_name(""),callee_phone(""),caller_phone(""),
                        calllog_txt(""),intention_type("0"),label(""),call_count("0"),match_global_keyword(""),
                        clue_no(""),collect_info(""),buttons(""),calllog_id(""),call_progress(""),url(""){}
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

enum  class OC_data_enum{
     uuid,task_id,script_name,callee_phone,
     caller_phone,calllog_txt,intention_type,
     label,call_count,match_global_keyword,clue_no,
     collect_info,buttons,calllog_id
};
// REFLECTION(OC_data,uuid,task_id,script_name,callee_phone,caller_phone,calllog_txt,intention_type,label,call_count,match_global_keyword,clue_no,collect_info,buttons,calllog_id)

struct outcall_clue{
    std::string	label;
    std::string	alias;
};
// REFLECTION(outcall_clue,label,alias)
enum  class outcall_clue_enum{
    label,alias
};

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
// REFLECTION(calllog,task_id,script_name,callee_phone,caller_phone,calllog_txt,intention_type,call_count,match_global_keyword,collect_info,buttons,id)
enum  class calllog_enum{
    task_id,script_name,callee_phone,caller_phone,
    calllog_txt,intention_type,call_count,match_global_keyword,
    buttons,id,label,duration, call_result, transfer_number,
    transfer_duration, call_record_url, manual_status, 
    answer_time, hangup_time, call_time
};
// struct outcall_task{
//         std::string	uuid;
// };
// REFLECTION(outcall_task,uuid)

struct outcall_task
{
        std::string uuid; // 0:donot callback,1:callback
        std::string auto_recall_scenes;
        std::string auto_recall_max_times;
        std::string auto_recall_status;
};
// REFLECTION(outcall_task,uuid,auto_recall_scenes,auto_recall_max_times,auto_recall_status)
enum  class outcall_task_enum
{
    uuid, auto_recall_scenes,
    auto_recall_max_times,auto_recall_status
};

struct aicall_config
{
    int api_callback_scene_status;
};
// REFLECTION(aicall_config,api_callback_scene_status)
enum  class aicall_config_enum
{
    api_callback_scene_status
};


enum  class IdCluster{
    CalllogId,
    ClueId,
    TaskId,
    EnterpriseUid,
    CallCount,
    Url
};

enum  class IntentionType
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


struct WebOcApiData
{
    int type;
    std::vector<std::string> calllog_id_array;

    WebOcApiData():type(0),calllog_id_array({}){}
};

class CallBackManage:public CallRecord{

public:
    WebOcApiData ParsePostData(const std::string &str);
    void MakeQueueCache(const std::string &str);
    void CallBackHandle(CallInfo & cm_data,const std::tuple<std::string,std::string,std::string,std::string,std::string> &id_cluster,const int &class_judge);
    void CmDataSwitch(CallInfo & cm_data,CallBackData &data);
    void GetOCSyncData(CallBackData &data);
    void ParseIntetionAndCallResult(CallBackRules &rules);
    CallBackRules MakeCallBackRulesFromMySql(const std::tuple<std::string,std::string,std::string,std::string,std::string> &id_cluster);
    std::string SetRulesRedisCache(const CallBackRules &rules);
    bool GetRulesFromRedis(CallBackRules &rules);
    bool CallBackJudge(const CallBackRules &rules,const CallBackData &data);
    bool OC_sync_judge(const std::string &calllog_id);
    CallBackData CacheCmJsonSwitch(const std::string &cm_data);
    std::string MergeCacheJson(const CallBackData &data,const std::string &redis_cache);
    std::string GetCallRecordFromCm(const std::string url);
    void MutipleCallBackManage(CallBackData data, CallBackRules rule, const int &class_judge, const std::tuple<std::string,std::string, std::string, std::string, std::string> &id_cluster, const bool &callback_class);
    std::string GetCallBackUrl(const std::string &eid);
    void CallBackAction(const std::string &data,const std::string &url);
    void PrepareId(CallBackData &data,CallBackRules &rule,const int &class_judge,const std::string &id);
private:
    
    bool AutoTaskMatch(const CallBackRules &rules,const CallBackData &data);
    void CacheCmData(const CallBackData &data, std::string &result,const int &class_judge);
    std::string MakeCacheJson(const CallBackData &data);
    void ParseApiCallbackSceneStatus(CallBackRules &rules);
    // std::string CollectInfoXML2JSON(const std::string xml)

};      

#endif