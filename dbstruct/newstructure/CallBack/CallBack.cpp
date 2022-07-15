#include "CallBack.h"
#include <vector>
#include <iostream>
#include <vector>


int stoi_s(const std::string &str, int defval) {
    int i = defval;
    try {
        i = std::stoi(str);
    } catch (...) {
        
    }
    return i;
}
void CallBackManage::CallBackHandle(CallInfo &cm_data, const std::tuple<std::string, std::string, std::string, std::string,std::string> &id_cluster)
{
    LOGGER->info("begin CallBackHandle");
    CallBackData data;
    CallBackRules rule;
    data.eid = std::get<static_cast<int>(IdCluster::EnterpriseUid)>(id_cluster);
    data.task_id = std::get<static_cast<int>(IdCluster::TaskId)>(id_cluster);
    data.clue_id = std::get<static_cast<int>(IdCluster::ClueId)>(id_cluster);
    data.calllog_id = std::get<static_cast<int>(IdCluster::CalllogId)>(id_cluster);
    data.call_count = std::get<static_cast<int>(IdCluster::CallCount)>(id_cluster);
    CmDataSwitch(cm_data, data);


    std::shared_ptr<RedisOperate> instance = std::make_shared<RedisOperate>();
    // instance->RedisConnect();
    std::string locate = data.eid+"-"+data.task_id;
    if(instance->SearchRules(locate)!="null")
    {
        GetRulesFromRedis(rule);
        if(CallBackJudge(rule,data))
        {
            //callback
            if(OC_sync_judge(std::get<static_cast<int>(IdCluster::CalllogId)>(id_cluster)))
            {
                GetOCSyncData(data);
                std::string cm_data_json = MakeCacheJson(data);
                std::string caback_data = MergeCacheJson(data,cm_data_json);
            }
            else
            { 
               CacheCmData(data);
            }

        }

    } else {
        rule = MakeCallBackRulesFromMySql(id_cluster);
        instance->CacheRules(locate,SetRulesRedisCache(rule));
        if(CallBackJudge(rule,data))
        {

            if(OC_sync_judge(std::get<static_cast<int>(IdCluster::CalllogId)>(id_cluster)))
            {
                GetOCSyncData(data);
                std::string cm_data_json = MakeCacheJson(data);
                std::string caback_data = MergeCacheJson(data,cm_data_json);
                //callback
            }
            else
            { 
               CacheCmData(data);    
            }

        }
    }
}


bool CallBackManage::OC_sync_judge(const std::string &calllog_id)
{
    MySql * instance = MySql::getInstance();
    auto sync_judge = instance->mysqlclient.query<std::tuple<std::string>>("select script_name from calllog where id = "+calllog_id);
    return std::get<0>(sync_judge[0])==""?0:1;
}


void CallBackManage::GetOCSyncData(CallBackData &data)
{
    MySql * instance = MySql::getInstance();
    LOGGER->info("GetOCSyncData eid is {}",data.eid);
    GenerateSQL sql_command;
    //clue
    std::string db_name_clue = "outcall_clue";
    std::vector<std::string> values_clue = {"label","alias"};
    std::vector<std::string> condition_clue = {data.eid,data.clue_id};
    std::vector<std::string> condition_name_clue{"enterprise_uid","id"};
    std::vector<std::string> condition_symbols_clue = {"=","="};

    std::string command_clue = sql_command.MysqlGenerateSelectSQL(db_name_clue,values_clue,condition_clue,condition_name_clue,condition_symbols_clue);
    auto result_outcall_clue = instance->mysqlclient.query<std::tuple<std::string,std::string>>(command_clue.c_str());
    LOGGER->info("command_clue is {}",command_clue);

    //calllog
    std::string db_name_calllog = "calllog";
    std::vector<std::string> values_calllog = {"task_id","script_name","callee_phone","caller_phone","calllog_txt",
                                        "intention_type","call_count","match_global_keyword","collect_info","buttons","id"};
    std::vector<std::string> condition_calllog = {data.eid,data.calllog_id};
    std::vector<std::string> condition_name_calllog{"enterprise_uid","id"};
    std::vector<std::string> condition_symbols_calllog = {"=","="};
    std::string command_calllog = sql_command.MysqlGenerateSelectSQL(db_name_calllog,values_calllog,condition_calllog,condition_name_calllog,condition_symbols_calllog);
    auto result_calllog = instance->mysqlclient.query<std::tuple<std::string,std::string,std::string,
                                                    std::string,std::string,std::string,std::string,
                                                    std::string,std::string,std::string,std::string>>(command_calllog.c_str());
    LOGGER->info("command_calllog is {}",command_calllog);

    //task 
    std::string db_name_task = "outcall_task";
    std::vector<std::string> values_task = { "uuid", "auto_recall_scenes",
                                        "auto_recall_max_times","auto_recall_status"};
    std::vector<std::string> condition_task = {data.eid,data.task_id};
    std::vector<std::string> condition_name_task{"enterprise_uid","id"};
    std::vector<std::string> condition_symbols_task = {"=","="};
    std::string command_task = sql_command.MysqlGenerateSelectSQL(db_name_task,values_task,condition_task,condition_name_task,condition_symbols_task);

    auto result_outcall_task = instance->mysqlclient.query<std::tuple<std::string,std::string,std::string,std::string>>(command_task.c_str());
    LOGGER->info("command_task is {}",command_task);


    // clue
    if(result_outcall_clue.size())
    {  
        data.label = std::get<static_cast<int>(outcall_clue_enum::label)>(result_outcall_clue[0]);
        data.clue_no = std::get<static_cast<int>(outcall_clue_enum::alias)>(result_outcall_clue[0]);
    }
    else 
    {
        LOGGER->info("enterprise {}  clue {}  label , alias no data",data.eid,data.clue_id);
    }
    // calllog
    if(result_calllog.size())
    {
        data.intention_type = std::get<static_cast<int>(calllog_enum::intention_type)>(result_calllog[0]);
        data.match_global_keyword = std::get<static_cast<int>(calllog_enum::match_global_keyword)>(result_calllog[0]);
        data.callee_phone = std::get<static_cast<int>(calllog_enum::callee_phone)>(result_calllog[0]);
        data.calllog_txt = std::get<static_cast<int>(calllog_enum::calllog_txt)>(result_calllog[0]);
        data.call_count = std::get<static_cast<int>(calllog_enum::call_count)>(result_calllog[0]);
        data.buttons = std::get<static_cast<int>(calllog_enum::buttons)>(result_calllog[0]);
        data.script_name = std::get<static_cast<int>(calllog_enum::script_name)>(result_calllog[0]);
        data.caller_phone = std::get<static_cast<int>(calllog_enum::caller_phone)>(result_calllog[0]);
    }
       else 
    {
        LOGGER->info("enterprise {}   calllog {}  intention_type,match_global_keyword.... no data",data.eid,data.calllog_id);
    }

    // task
     if(result_outcall_task.size())
    {
        data.uuid = std::get<static_cast<int>(outcall_task_enum::uuid)>(result_outcall_task[0]);
    }
    else 
    {
        LOGGER->info("enterprise {} task {} uuid .... no data",data.eid,data.task_id);
    }
}

// std::string CallBackManage::SetMySqlRules(std::vector<std::string> rule_name, std::vector<std::string> rule)
// {
//     std::string rules;
//     rules += "where ";
//     for (int i = 0; i < rule_name.size(); i++)
//     {
//         rules += rule_name[i];
//         rules += " = ";
//         rules += "\"" + rule[i] + "\"";
//     }
// }


void CallBackManage::CmDataSwitch(CallInfo &cm_data, CallBackData &data)
{
    data.cc_number = cm_data.cc_number;
    data.call_result = cm_data.call_result;
    data.duration_time = cm_data.duration_time;
    data.manual_status = cm_data.manual_type;
    data.record_url = cm_data.record_url;
    data.answer_time = cm_data.confirm_time;
    data.hangup_time = cm_data.end_time;
    data.switch_number = cm_data.switch_number;
    data.hangup_type = cm_data.hangup_type;
    data.transfer_number = cm_data.transfer_number;
    data.transfer_duration = cm_data.transfer_duration;
    data.call_time = stoi_s(cm_data.start_time);
}

CallBackRules CallBackManage::MakeCallBackRulesFromMySql(const std::tuple<std::string, std::string, std::string, std::string,std::string> &id_cluster)
{
    MySql * instance = MySql::getInstance();
    CallBackRules rules;
    rules.eid = stoi_s(std::get<static_cast<int>(IdCluster::EnterpriseUid)>(id_cluster));
    rules.task_id = stoi_s(std::get<static_cast<int>(IdCluster::TaskId)>(id_cluster));

    GenerateSQL general_sql;
    //outcall_task
    std::string calllog_id, clue_id, task_id;
    std::string db_name_outcall_task = "outcall_task";
    std::vector<std::string> values_outcall_task = {"auto_recall_scenes","auto_recall_status","auto_recall_max_times"};
    std::vector<std::string> condition_outcall_task{std::get<static_cast<int>(IdCluster::TaskId)>(id_cluster),std::get<static_cast<int>(IdCluster::EnterpriseUid)>(id_cluster)};
    std::vector<std::string> condition_name_outcall_task{"id","enterprise_uid"};
    std::vector<std::string> condition_symbols_outcall_task{"=","="};

    std::string outcall_task_rule = general_sql.MysqlGenerateSelectSQL(db_name_outcall_task,values_outcall_task,condition_outcall_task,condition_name_outcall_task,condition_symbols_outcall_task);
    LOGGER->info("command is {}",outcall_task_rule);
    auto result_outcall_task = instance->mysqlclient.query<std::tuple<std::string,std::string,std::string,std::string>>(outcall_task_rule.c_str());
    LOGGER->info("size is {}",result_outcall_task.size());

    //aicall_config
    std::string db_name_aicall_config = "aicall_config";
    std::vector<std::string> values_aicall_config = {"api_callback_scene_status"};
    std::vector<std::string> condition_name_aicall_config= {"eid"};
    std::vector<std::string> condition_aicall_config= {std::get<int(IdCluster::EnterpriseUid)>(id_cluster)};
    std::vector<std::string> condition_symbols_aicall_config= {"="};
    std::string aicall_config_rule = general_sql.MysqlGenerateSelectSQL(db_name_aicall_config,values_aicall_config,condition_aicall_config, condition_name_aicall_config, condition_symbols_aicall_config);
    auto result_aicall_config = instance->mysqlclient.query<std::tuple<std::string>>(aicall_config_rule);
    LOGGER->info("command is {}",aicall_config_rule);

    // TEST_CHECK(result_aicall_config.size() == 1);
    if(result_aicall_config.size())
    {
        rules.api_callback_scene_status = std::get<static_cast<int>(aicall_config_enum::api_callback_scene_status)>(result_aicall_config[0]);
    }
    else 
    {
        LOGGER->info("enterprise {} api_callback_scene_status no data",rules.eid);
    }
    if(result_outcall_task.size())
    {   
        rules.auto_recall_scenes = std::get<static_cast<int>(outcall_task_enum::auto_recall_scenes)>(result_outcall_task[0]);
        rules.auto_recall_status = stoi_s(std::get<static_cast<int>(outcall_task_enum::auto_recall_status)>(result_outcall_task[0]));
        rules.auto_recall_max_times =  stoi_s(std::get<static_cast<int>(outcall_task_enum::auto_recall_max_times)>(result_outcall_task[0]));
    }
    else 
    {
        LOGGER->info("enterprise {} auto_recall_scenes.... no data",rules.eid);
    }
    ParseIntetionAndCallResult(rules);
    return rules;
}
void CallBackManage::ParseIntetionAndCallResult(CallBackRules &rules)
{
    LOGGER->info("ParseIntetionAndCallResult, rules is {}", rules.auto_recall_scenes);
    rapidjson::Value root;
    rapidjson::Document doc;
    if(doc.Parse(rules.auto_recall_scenes.c_str()))
    {if (doc.IsObject())
    {
        if (doc.HasMember("call_result"))
        {
            root = doc["call_result"];
            for (int i = 0; i < root.Size(); i++)
            {
                int pos = root[i].GetInt();
                rules.call_result_judge.replace(pos, 1, "1");
            }
        }
        if (doc.HasMember("intention_type"))
        {
            root = doc["intention_type"];
            for (int i = 0; i < root.Size(); i++)
            {
                int pos = root[i].GetInt();
                rules.call_result_judge.replace(pos, 1, "1");
            }
        }
    }
}

std::string CallBackManage::SetRulesRedisCache(const CallBackRules &rules)
{

    LOGGER->info("SetRulesRedisCache, result is {} and so on", rules.uuid);
    rapidjson::Document doc;
    rapidjson::Value val;
    std::string result;
    std::string api_callback_scene_status = rules.api_callback_scene_status;
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();

    doc.AddMember("detail_judge", val.SetString(rules.api_callback_scene_status.c_str(),allocator), allocator);
    doc.AddMember("global_judge", val.SetInt(rules.global_judge), allocator);
    doc.AddMember("scope_judge", val.SetInt(rules.scope_judge), allocator);
    doc.AddMember("uuid", val.SetString(rules.uuid.c_str(),allocator), allocator);
    doc.AddMember("intention_type_judge", val.SetString(rules.intention_type_judge.c_str(),allocator), allocator);
    doc.AddMember("call_result_judge", val.SetString(rules.call_result_judge.c_str(),allocator), allocator);
    doc.AddMember("auto_recall_status", val.SetInt(rules.auto_recall_status), allocator);

    rapidjson::StringBuffer str_buf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(str_buf);
    doc.Accept(writer);
    return str_buf.GetString();
}

bool CallBackManage::GetRulesFromRedis(CallBackRules &rules)
{

    std::string location = std::to_string(rules.eid) + '-' + std::to_string(rules.task_id);

    std::shared_ptr<RedisOperate> instance = std::make_shared<RedisOperate>();
    // instance->RedisConnect();
    std::string rule = instance->SearchRules(location);
    rapidjson::Document doc;
    LOGGER->info("GetRulesFromRedis search {}", location);
    doc.Parse(rule.c_str());
    if (doc.IsObject())
    {
        rules.api_callback_scene_status = doc["detail_judge"].GetString();
        rules.global_judge = doc["global_judge"].GetInt();
        rules.scope_judge = doc["scope_judge"].GetInt();
        rules.uuid = doc["uuid"].GetString();
        rules.intention_type_judge = doc["intention_type_judge"].GetString();
        rules.call_result_judge = doc["call_result_judge"].GetString();
        rules.auto_recall_status = doc["auto_recall_status"].GetInt();
        return 1;
    }
    return 0;
}

bool CallBackManage::AutoTaskMatch(const CallBackRules &rules, const CallBackData &data)
{
    if (rules.intention_type_judge != "0000000000000")
    {

        if ((rules.intention_type_judge)[stoi_s(data.intention_type)] == '1')
            return 1;
    }
    if (rules.call_result_judge != "00000000000")
    {
        if (rules.call_result_judge[data.call_result] == '1')
            return 1;
    }
    return 0;
}

bool CallBackManage::CallBackJudge(const CallBackRules &rules, const CallBackData &data)
{
    if (((rules.global_judge) || (rules.uuid != "")) && rules.api_callback_scene_status != "0")
    {
        if ((rules.scope_judge == 1 && rules.auto_recall_status == 1))
        {
            if (AutoTaskMatch(rules, data))
            {
                if (rules.auto_recall_max_times == stoi_s(data.call_count))
                    return 1;
                else
                    return 0;
            }
            else
                return 1;
        }
        else
            return 1;
    }
}

void CallBackManage::CacheCmData(const CallBackData &data)
{
    auto now = time(NULL);    
    std::stringstream sstream;
    sstream << now;
    std::string time_ = sstream.str();
    std::string id = data.eid+"-"+data.task_id+"-"+data.calllog_id+"-"+time_;
    LOGGER->info("cache cm_data,which id is {}",id);
    
    std::string cache_data;
    std::shared_ptr<RedisOperate> instance = std::make_shared<RedisOperate>();
    // instance->RedisConnect();
    std::vector<std::string>list{id};
    std::string set_name = "cm_id_cluster";
    cache_data = MakeCacheJson(data);
    instance->CacheData(id,cache_data);
    instance->Rpush(set_name,list);
}

std::string CallBackManage::MakeCacheJson(const CallBackData &data)//from code cache
{
    rapidjson::Document doc;
    rapidjson::Value root;
    rapidjson::Value data_json;
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
    rapidjson::Value val;

    doc.AddMember("uuid",val.SetString(data.uuid.c_str(),allocator),allocator);
    root.AddMember("task_id",stoi_s(data.task_id),allocator);
    root.AddMember("call_result",data.call_result,allocator);
    root.AddMember("manual_status",data.manual_status,allocator);
    root.AddMember("call_time",data.call_time,allocator);
    root.AddMember("duration",data.duration_time,allocator);
    root.AddMember("answer_time",stoi_s(data.answer_time),allocator);
    root.AddMember("hangup_time",stoi_s(data.hangup_time),allocator);
    root.AddMember("transfer_number",val.SetString(data.transfer_number.c_str(),allocator),allocator);
    root.AddMember("transfer_duration",data.transfer_duration,allocator);
    root.AddMember("record_url",val.SetString(data.record_url.c_str(),allocator),allocator);

    data_json.PushBack(root,allocator);

    doc.AddMember("records",data_json,allocator);

    rapidjson::StringBuffer strBuffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strBuffer);
    doc.Accept(writer);
    LOGGER->info("oc has not sync,apicallback data is {}",strBuffer.GetString());
    return strBuffer.GetString();
}

std::string CallBackManage::MergeCacheJson(const CallBackData &data,const std::string &redis_cache)//from redis cache  ,add these data
{
    rapidjson::Document doc;
    rapidjson::Value root;
    rapidjson::Value data_json;
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
    rapidjson::Value val;
    doc.Parse(redis_cache.c_str());

    root = doc["records"][0];
    root.AddMember("script_name",val.SetString(data.script_name.c_str(),allocator),allocator);
    root.AddMember("callee_phone",val.SetString(data.callee_phone.c_str(),allocator),allocator);
    root.AddMember("calllog_txt",val.SetString(data.calllog_txt.c_str(),allocator),allocator);
    root.AddMember("intention_type",stoi_s(data.intention_type),allocator);
    root.AddMember("label",val.SetString(data.label.c_str(),allocator),allocator);
    root.AddMember("call_count",stoi_s(data.call_count),allocator);
    root.AddMember("match_global_keyword",val.SetString(data.match_global_keyword.c_str(),allocator),allocator);
    root.AddMember("clue_no",val.SetString(data.clue_no.c_str(),allocator),allocator);
    root.AddMember("collect_info",val.SetString(data.collect_info.c_str(),allocator),allocator);
    root.AddMember("buttons",val.SetString(data.buttons.c_str(),allocator),allocator);

    data_json.PushBack(root,allocator);
    doc.EraseMember("records");
    doc.AddMember("records",data_json,allocator);

    rapidjson::StringBuffer strBuffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strBuffer);
    doc.Accept(writer);
    LOGGER->info("oc has sync ,apicallback data is {}",strBuffer.GetString());
    return strBuffer.GetString();
}

CallBackData  CallBackManage::CacheCmJsonSwitch(const std::string &data)
{
    CallBackData cm_info;
    rapidjson::Document doc;
    rapidjson::Value root;
    doc.Parse(data.c_str());
    root = doc["records"][0];
    cm_info.cc_number = root["cc_number"].GetString();
    cm_info.call_result = root["call_result"].GetInt();
    cm_info.duration_time = root["duration_time"].GetInt();
    cm_info.manual_status = root["manual_type"].GetInt();
    cm_info.record_url = root["record_url"].GetString();
    cm_info.answer_time = root["confirm_time"].GetString();
    cm_info.hangup_time = root["end_time"].GetString();
    cm_info.switch_number = root["switch_number"].GetString();
    cm_info.hangup_type = root["hangup_type"].GetInt();
    cm_info.transfer_number = root["transfer_number"].GetString();
    cm_info.transfer_duration = root["transfer_duration"].GetInt();
    cm_info.call_time = root["start_time"].GetInt();
    return cm_info;

}