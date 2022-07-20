#include "CallBack.h"
#include <vector>
#include <iostream>
#include <vector>

int stoi_s(const std::string &str)
{
    int i = 0;
    try
    {
        i = std::stoi(str);
    }
    catch (...)
    {
    }
    return i;
}

void SplitString(const std::string &s, std::vector<std::string> &tokens, const std::string &delimiters = " ")
{
    std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    std::string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        tokens.push_back(s.substr(lastPos, pos - lastPos));
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}
void CallBackManage::CallBackHandle(CallInfo &cm_data, const std::tuple<std::string, std::string, std::string,std::string, std::string> &id_cluster, const bool &callback_class)
{
    LOGGER->info("begin CallBackHandle");
    CallBackData data;
    CallBackRules rule;
    data.eid = std::get<static_cast<int>(IdCluster::EnterpriseUid)>(id_cluster);
    data.task_id = std::get<static_cast<int>(IdCluster::TaskId)>(id_cluster);
    data.clue_id = std::get<static_cast<int>(IdCluster::ClueId)>(id_cluster);
    data.calllog_id = std::get<static_cast<int>(IdCluster::CalllogId)>(id_cluster);
    data.call_count = std::get<static_cast<int>(IdCluster::CallCount)>(id_cluster);
    rule.eid = stoi_s(std::get<static_cast<int>(IdCluster::EnterpriseUid)>(id_cluster));
    rule.task_id = stoi_s(std::get<static_cast<int>(IdCluster::TaskId)>(id_cluster));
    data.url = GetCallBackUrl(data.eid);
    CmDataSwitch(cm_data, data);
    MutipleCallBackManage(data, rule, callback_class,id_cluster,callback_class);
}

void CallBackManage::MutipleCallBackManage(CallBackData data, CallBackRules rule, const int &class_judge, const std::tuple<std::string, std::string,std::string, std::string, std::string> &id_cluster, const bool &callback_class)
{

    std::shared_ptr<RedisOperate> instance = std::make_shared<RedisOperate>();
    // instance->RedisConnect();
    std::string locate = data.eid + "-" + data.task_id;
    if (instance->SearchRules(locate) == "null")
    {
        rule = MakeCallBackRulesFromMySql(id_cluster);
    }
    else
    {
        GetRulesFromRedis(rule);
        instance->CacheRules(locate, SetRulesRedisCache(rule));
    }

    if (CallBackJudge(rule, data))
    {
        if (OC_sync_judge(data.calllog_id))
        {
            GetOCSyncData(data);
            if (CallBackJudge(rule, data))
            {
                std::string cm_data_json = MakeCacheJson(data);
                std::string caback_data = MergeCacheJson(data, cm_data_json);
                // callback
                LOGGER->info("data is sync ,begin callback");
                int call_class = 2;
                CacheCmData(data, caback_data, call_class);
            }
        }
        else
        {
            int call_class = 0;
            std::string str = "";
            CacheCmData(data, str, call_class);
            LOGGER->info("data is not sync ,donot callback");
        }
    }
    else
        LOGGER->info("do not pass CallBackJudge,cannot callback");
}

bool CallBackManage::OC_sync_judge(const std::string &calllog_id)
{
    MySql *instance = MySql::getInstance();
    auto sync_judge = instance->mysqlclient.query<std::tuple<std::string>>("select script_name from calllog where id = " + calllog_id);
    return std::get<0>(sync_judge[0]) == "" ? 0 : 1;
}

std::string CallBackManage::GetCallBackUrl(const std::string &eid)
{
    MySql *instance = MySql::getInstance();
    auto url = instance->mysqlclient.query<std::tuple<std::string>>("select value from aicall_config where `key` = 'api_callback_domain' and eid = "+eid);
    if(url.size())
        return  std::get<0>(url[0]);
    else
        return ""; 
}

void CallBackManage::GetOCSyncData(CallBackData &data)
{
    MySql *instance = MySql::getInstance();
    LOGGER->info("GetOCSyncData eid is {}", data.eid);
    GenerateSQL sql_command;

    // clue
    std::string db_name_clue = "outcall_clue";
    std::vector<std::string> values_clue = {"label", "alias"};
    std::vector<std::string> condition_clue = {data.clue_id};
    std::vector<std::string> condition_name_clue{"id"};
    std::vector<std::string> condition_symbols_clue = {"="};

    std::string command_clue = sql_command.MysqlGenerateSelectSQL(db_name_clue, values_clue, condition_clue, condition_name_clue, condition_symbols_clue);
    auto result_outcall_clue = instance->mysqlclient.query<std::tuple<std::string, std::string>>(command_clue.c_str());
    LOGGER->info("command_clue is {}", command_clue);


    if (result_outcall_clue.size())
    {
        data.label = std::get<static_cast<int>(outcall_clue_enum::label)>(result_outcall_clue[0]);
        data.clue_no = std::get<static_cast<int>(outcall_clue_enum::alias)>(result_outcall_clue[0]);
    }
    else
    {
        LOGGER->info("enterprise {}  clue {}  label , alias no data", data.eid, data.clue_id);
    }

    // // calllog
    // std::string db_name_calllog = "calllog";
    // std::vector<std::string> values_calllog = {"task_id", "script_name", "callee_phone", "caller_phone", "calllog_txt",
    //                                            "intention_type", "call_count", "match_global_keyword", "buttons", "id","label"};
    // std::vector<std::string> condition_calllog = {data.eid, data.calllog_id};
    // std::vector<std::string> condition_name_calllog{"enterprise_uid", "id"};
    // std::vector<std::string> condition_symbols_calllog = {"=", "="};
    // if (data.cc_number == "")
    // {
    //     LOGGER->info("cm no data should get all data from oc");
    //     std::vector<std::string> calllog_columns = {"duration", "call_result", "transfer_number", "transfer_duration", "call_record_url", "manual_status", "answer_time", "hangup_time", "call_time"};

    //     values_calllog.push_back(calllog_columns.begin(), calllog_columns.end());
    //     std::string command_calllog = sql_command.MysqlGenerateSelectSQL(db_name_calllog, values_calllog, condition_calllog, condition_name_calllog, condition_symbols_calllog);
    //     auto result_calllog_ = instance->mysqlclient.query<std::tuple<std::string, std::string, std::string,
    //                                                                  std::string, std::string, std::string,std::string,
    //                                                                  std::string, std::string, std::string,std::string,
    //                                                                  std::string, std::string, std::string,
    //                                                                  std::string, std::string, std::string,
    //                                                                  std::string, std::string, std::string>>(command_calllog.c_str());
    //     LOGGER->info("command_calllog is {}", command_calllog);
    // }
    // else
    // {
    //     std::string command_calllog = sql_command.MysqlGenerateSelectSQL(db_name_calllog, values_calllog, condition_calllog, condition_name_calllog, condition_symbols_calllog);
    //     auto result_calllog_ = instance->mysqlclient.query<std::tuple<std::string, std::string, std::string,
    //                                                                  std::string, std::string, std::string, std::string,
    //                                                                  std::string, std::string, std::string,std::string>>(command_calllog.c_str());
    //     LOGGER->info("command_calllog is {}", command_calllog);
    // }

    // if (result_calllog.size())
    // {
    //     data.intention_type = std::get<static_cast<int>(calllog_enum::intention_type)>(result_calllog[0]);
    //     data.match_global_keyword = std::get<static_cast<int>(calllog_enum::match_global_keyword)>(result_calllog[0]);
    //     data.callee_phone = std::get<static_cast<int>(calllog_enum::callee_phone)>(result_calllog[0]);
    //     data.calllog_txt = std::get<static_cast<int>(calllog_enum::calllog_txt)>(result_calllog[0]);
    //     data.call_count = std::get<static_cast<int>(calllog_enum::call_count)>(result_calllog[0]);
    //     data.buttons = std::get<static_cast<int>(calllog_enum::buttons)>(result_calllog[0]);
    //     data.script_name = std::get<static_cast<int>(calllog_enum::script_name)>(result_calllog[0]);
    //     data.caller_phone = std::get<static_cast<int>(calllog_enum::caller_phone)>(result_calllog[0]);
    //     data.collect_info = std::get<static_cast<int>(calllog_enum::calllog_txt)>(result_calllog[0]);
    //     data.label = std::get<static_cast<int>(calllog_enum::lebel)>(result_calllog[0]);
    //     if (data.cc_number == "")
    //     {
    //         //         duration, call_result, transfer_number,
    //         // transfer_duration, call_record_url, manual_status,
    //         // answer_time, hangup_time, call_time
    //         data.duration_time = std::get<static_cast<int>(calllog_enum::duration)>(result_calllog[0]);
    //         data.call_result = std::get<static_cast<int>(calllog_enum::call_result)>(result_calllog[0]);
    //         data.transfer_number = std::get<static_cast<int>(calllog_enum::transfer_number)>(result_calllog[0]);
    //         data.transfer_duration = std::get<static_cast<int>(calllog_enum::transfer_duration)>(result_calllog[0]);
    //         data.call_record_url = std::get<static_cast<int>(calllog_enum::call_record_url)>(result_calllog[0]);
    //         data.manual_status = std::get<static_cast<int>(calllog_enum::manual_status)>(result_calllog[0]);
    //         data.answer_time = std::get<static_cast<int>(calllog_enum::answer_time)>(result_calllog[0]);
    //         data.hangup_time = std::get<static_cast<int>(calllog_enum::hangup_time)>(result_calllog[0]);
    //         data.call_time = std::get<static_cast<int>(calllog_enum::call_time)>(result_calllog[0]);
    //     }
    // }
    // else
    // {
    //     LOGGER->info("enterprise {}   calllog {}  intention_type,match_global_keyword.... no data", data.eid, data.calllog_id);
    // }

    // task
    std::string db_name_task = "outcall_task";
    std::vector<std::string> values_task = {"uuid", "auto_recall_scenes",
                                            "auto_recall_max_times", "auto_recall_status"};
    std::vector<std::string> condition_task = {data.eid, data.task_id};
    std::vector<std::string> condition_name_task{"enterprise_uid", "id"};
    std::vector<std::string> condition_symbols_task = {"=", "="};
    std::string command_task = sql_command.MysqlGenerateSelectSQL(db_name_task, values_task, condition_task, condition_name_task, condition_symbols_task);

    auto result_outcall_task = instance->mysqlclient.query<std::tuple<std::string, std::string, std::string, std::string>>(command_task.c_str());
    LOGGER->info("command_task is {}", command_task);

    if (result_outcall_task.size())
    {
        data.uuid = std::get<static_cast<int>(outcall_task_enum::uuid)>(result_outcall_task[0]);
    }
    else
    {
        LOGGER->info("enterprise {} task {} uuid .... no data", data.eid, data.task_id);
    }

    // // calllog_extession
    // if (data.cc_number == "")
    // {
    //     std::vector<std::string> calllog_extension_columns = {"switch_number"};
    //     std::string db_name_extension = "aicall_calllog_extension";

    //     std::vector<std::string> condition_extension = {data.calllog_id};
    //     std::vector<std::string> condition_name_extension{"calllog_id"};
    //     std::vector<std::string> condition_symbols_extension = {"="};
    //     std::string command_extension = sql_command.MysqlGenerateSelectSQL(db_name_extension, calllog_extension_columns, condition_extension, condition_name_extension, condition_symbols_extension);

    //     auto result_outcall_extension = instance->mysqlclient.query<std::tuple<std::string>(command_extension.c_str());
    //     LOGGER->info("command_extension is {}", command_extension);
    //     if(result_outcall_extension.size())
    //     {
    //         data.switch_number = std::get<0>(result_calllog[0]);
    //     }
    // }

}

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

CallBackRules CallBackManage::MakeCallBackRulesFromMySql(const std::tuple<std::string, std::string, std::string, std::string, std::string> &id_cluster)
{
    MySql *instance = MySql::getInstance();
    CallBackRules rules;
    rules.eid = stoi_s(std::get<static_cast<int>(IdCluster::EnterpriseUid)>(id_cluster));
    rules.task_id = stoi_s(std::get<static_cast<int>(IdCluster::TaskId)>(id_cluster));

    GenerateSQL general_sql;
    // outcall_task
    std::string calllog_id, clue_id, task_id;
    std::string db_name_outcall_task = "outcall_task";
    std::vector<std::string> values_outcall_task = {"auto_recall_scenes", "auto_recall_status", "auto_recall_max_times"};
    std::vector<std::string> condition_outcall_task{std::get<static_cast<int>(IdCluster::TaskId)>(id_cluster), std::get<static_cast<int>(IdCluster::EnterpriseUid)>(id_cluster)};
    std::vector<std::string> condition_name_outcall_task{"id", "enterprise_uid"};
    std::vector<std::string> condition_symbols_outcall_task{"=", "="};

    std::string outcall_task_rule = general_sql.MysqlGenerateSelectSQL(db_name_outcall_task, values_outcall_task, condition_outcall_task, condition_name_outcall_task, condition_symbols_outcall_task);
    LOGGER->info("command is {}", outcall_task_rule);
    auto result_outcall_task = instance->mysqlclient.query<std::tuple<std::string, std::string, std::string, std::string>>(outcall_task_rule.c_str());
    LOGGER->info("size is {}", result_outcall_task.size());

    // aicall_config
    std::string db_name_aicall_config = "aicall_config";
    std::vector<std::string> values_aicall_config = {"`value`"};
    std::vector<std::string> condition_name_aicall_config = {"eid", "`key`"};
    std::vector<std::string> condition_aicall_config = {std::get<int(IdCluster::EnterpriseUid)>(id_cluster), "api_callback_scene_status"};
    std::vector<std::string> condition_symbols_aicall_config = {"=", "="};
    std::string aicall_config_rule = general_sql.MysqlGenerateSelectSQL(db_name_aicall_config, values_aicall_config, condition_aicall_config, condition_name_aicall_config, condition_symbols_aicall_config);
    auto result_aicall_config = instance->mysqlclient.query<std::tuple<std::string>>(aicall_config_rule);
    LOGGER->info("command is {}", aicall_config_rule);

    // TEST_CHECK(result_aicall_config.size() == 1);
    if (result_aicall_config.size())
    {
        rules.api_callback_scene_status = std::get<static_cast<int>(aicall_config_enum::api_callback_scene_status)>(result_aicall_config[0]);
    }
    else
    {
        LOGGER->info("enterprise {} api_callback_scene_status no data", rules.eid);
    }
    if (result_outcall_task.size())
    {
        rules.auto_recall_scenes = std::get<static_cast<int>(outcall_task_enum::auto_recall_scenes)>(result_outcall_task[0]);
        rules.auto_recall_status = stoi_s(std::get<static_cast<int>(outcall_task_enum::auto_recall_status)>(result_outcall_task[0]));
        rules.auto_recall_max_times = stoi_s(std::get<static_cast<int>(outcall_task_enum::auto_recall_max_times)>(result_outcall_task[0]));
    }
    else
    {
        LOGGER->info("enterprise {} auto_recall_scenes.... no data", rules.eid);
    }
    ParseIntetionAndCallResult(rules);
    ParseApiCallbackSceneStatus(rules);
    return rules;
}

// std::string CallBackManage::CollectInfoXML2JSON(const std::string xml)
// {
//     std::string jsonstr = "[]";
//     tinyxml2::XMLDocument doc;
//     tinyxml2::XMLError error = doc.Parse(xml.c_str());
//     auto root = doc.RootElement();
//     if(root)
//     {
//         auto collect_info_element = root->FirstChildElement("collect_info");
//         if(collect_info_element)
//         {
//             rapidjson::StringBuffer buffer;
//             rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
//             writer.StartArray();
//             auto child = collect_info_element->FirstChildElement("item");
//             while(child)
//             {
//                 writer.StartObject();
//                 writer.Key("question_name");
//                 writer.String(child->Attribute("match_standard_question"));
//                 writer.Key("match_result");
//                 writer.String(child->Attribute("match_cluster_name"));
//                 writer.Key("real_answer");
//                 writer.String(child->GetText()==nullptr?"":child->GetText());
//                 writer.EndObject();
//                 child = child->NextSiblingElement("item");
//             }
//             writer.EndArray();
//             jsonstr = buffer.GetString();
//         }
//     }
//     return jsonstr;
// }

void CallBackManage::ParseApiCallbackSceneStatus(CallBackRules &rules)
{
    LOGGER->info("ParseIntetionAndCallResult, api_callback_scene_status is {}", rules.api_callback_scene_status);
    rapidjson::Document doc;
    doc.Parse(rules.api_callback_scene_status.c_str());
    if (doc.IsObject())
    {
        if (doc.HasMember("callback_scope"))
        {
            rules.global_judge = doc["callback_scope"].GetInt();
        }
        if (doc.HasMember("call_record_detail"))
        {
            rules.callback = doc["call_record_detail"].GetInt();
        }
        if (doc.HasMember("call_record_detail_occasion"))
        {
            rules.scope_judge = doc["call_record_detail_occasion"].GetInt();
        }
    }
}
void CallBackManage::ParseIntetionAndCallResult(CallBackRules &rules)
{
    LOGGER->info("ParseIntetionAndCallResult, auto_recall_scenes is {}", rules.auto_recall_scenes);
    rapidjson::Value root;
    rapidjson::Document doc;
    doc.Parse(rules.auto_recall_scenes.c_str());
    if (doc.IsObject())
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
    // std::string api_callback_scene_status = rules.api_callback_scene_status;
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
    doc.SetObject();
    doc.AddMember("detail_judge", rapidjson::StringRef(rules.api_callback_scene_status.c_str()), allocator);
    doc.AddMember("global_judge", rules.global_judge, allocator);
    doc.AddMember("scope_judge", rules.scope_judge, allocator);
    doc.AddMember("uuid", rapidjson::StringRef(rules.uuid.c_str()), allocator);
    doc.AddMember("intention_type_judge", rapidjson::StringRef(rules.intention_type_judge.c_str()), allocator);
    doc.AddMember("call_result_judge", rapidjson::StringRef(rules.call_result_judge.c_str()), allocator);
    doc.AddMember("auto_recall_status", rules.auto_recall_status, allocator);

    rapidjson::StringBuffer str_buf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(str_buf);
    doc.Accept(writer);
    return str_buf.GetString();
}

bool CallBackManage::GetRulesFromRedis(CallBackRules &rules)
{

    std::string location = std::to_string(rules.eid) + '-' + std::to_string(rules.task_id);
    // LOGGER->info("eid is {} ,task_id is {} ",rules.eid,rules.task_id);
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
    if (((rules.global_judge) || (rules.uuid != "")) && rules.callback)
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
    return 0;
}

void CallBackManage::CacheCmData(const CallBackData &data, std::string &cache_data, const bool &class_judge)
{
    auto now = time(NULL);
    std::stringstream sstream;
    sstream << now;
    std::string time_ = sstream.str();
    std::string id;
    std::shared_ptr<RedisOperate> instance = std::make_shared<RedisOperate>();
    if (class_judge == 0)
    {
        id = data.eid + "-" + data.task_id + "-" + data.calllog_id + "-" + time_+"-"+data.url;
        cache_data = MakeCacheJson(data);
    }
    else if (class_judge == 1)
    {
        id = data.eid + "-" + data.task_id + "-" + data.calllog_id + "-web-"+data.url; // web callback
    }
    else if (class_judge == 2)
    {
        id = data.eid + "-" + data.task_id + "-" + data.calllog_id + "-now-"+data.url; // cm and oc  must callback now
    }
    else if (class_judge == 3)
    {
        id = data.eid + "-" + data.task_id + "-" + data.calllog_id + "-oc-"+data.url; // oc callback
    }
    LOGGER->info("cache cm_data,which id is {}", id);

    // instance->RedisConnect();
    std::vector<std::string> list{id};
    std::string set_name = "cm_id_cluster";
    instance->CacheData(id, cache_data);
    instance->Rpush(set_name, list);
}

std::string CallBackManage::MakeCacheJson(const CallBackData &data) // from code cache
{
    rapidjson::Document doc;
    rapidjson::Value root(rapidjson::kObjectType);
    rapidjson::Value data_json(rapidjson::Type::kArrayType);
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
    rapidjson::Value val;
    doc.SetObject();

    doc.AddMember("uuid", val.SetString(data.uuid.c_str(), allocator), allocator);
    // root.AddMember("eid",stoi_s(data.eid),allocator);
    // root.AddMember("calllog_id",stoi_s(data.calllog_id),allocator);
    // root.AddMember("clue_id",stoi_s(data.clue_id),allocator);
    // root.AddMember("task_id",stoi_s(data.task_id),allocator);
    root.AddMember("call_result", data.call_result, allocator);
    root.AddMember("manual_status", data.manual_status, allocator);
    root.AddMember("call_time", data.call_time, allocator);
    root.AddMember("duration", data.duration_time, allocator);
    root.AddMember("answer_time", stoi_s(data.answer_time), allocator);
    root.AddMember("hangup_time", stoi_s(data.hangup_time), allocator);
    root.AddMember("transfer_number", val.SetString(data.transfer_number.c_str(), allocator), allocator);
    root.AddMember("transfer_duration", data.transfer_duration, allocator);
    root.AddMember("record_url", val.SetString(data.record_url.c_str(), allocator), allocator);
    root.AddMember("cc_number", val.SetString(data.cc_number.c_str(), allocator), allocator);

    data_json.PushBack(root, allocator);

    doc.AddMember("records", data_json, allocator);

    rapidjson::StringBuffer strBuffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strBuffer);
    doc.Accept(writer);
    std::string res = strBuffer.GetString();
    LOGGER->info("oc has not sync,apicallback data is {}", res);

    return res;
}

std::string CallBackManage::MergeCacheJson(const CallBackData &data, const std::string &redis_cache) // from redis cache  ,add these data
{

    // std::string clue_no = data.clue_no;
    // if (!data.alias.empty()) {
    //     clue_no = clue->alias;
    //     std::vector<std::string> tokens;
    //     SplitString(clue->alias, tokens, "||");
    //     if (tokens.size() == 2) {
    //         uuid = tokens[0];
    //         clue_no = tokens[1];
    //     } else if (tokens.size() == 1 && tokens[0] != clue->alias) {
    //         uuid = tokens[0];
    //         clue_no = "";
    //     }
    // }

    rapidjson::Document doc;
    rapidjson::Value root(rapidjson::kObjectType);
    rapidjson::Value data_json(rapidjson::Type::kArrayType);
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
    rapidjson::Value val;
    doc.Parse(redis_cache.c_str());
    if (doc.IsObject() && doc.HasMember("records"))
    {
        root = doc["records"][0];
        // root.EraseMember("eid");
        // root.EraseMember("clue_id");
        // root.EraseMember("calllog_id");
        root.AddMember("script_name", val.SetString(data.script_name.c_str(), allocator), allocator);
        root.AddMember("callee_phone", val.SetString(data.callee_phone.c_str(), allocator), allocator);
        root.AddMember("calllog_txt", val.SetString(data.calllog_txt.c_str(), allocator), allocator);
        root.AddMember("intention_type", stoi_s(data.intention_type), allocator);
        root.AddMember("label", val.SetString(data.label.c_str(), allocator), allocator);
        root.AddMember("call_count", stoi_s(data.call_count), allocator);
        root.AddMember("match_global_keyword", val.SetString(data.match_global_keyword.c_str(), allocator), allocator);
        root.AddMember("clue_no", val.SetString(data.clue_no.c_str(), allocator), allocator);
        root.AddMember("collect_info", val.SetString(data.collect_info.c_str(), allocator), allocator);
        root.AddMember("buttons", val.SetString(data.buttons.c_str(), allocator), allocator);
    }

    data_json.PushBack(root, allocator);
    doc.EraseMember("records");
    doc.AddMember("records", data_json, allocator);

    rapidjson::StringBuffer strBuffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strBuffer);
    doc.Accept(writer);
    LOGGER->info("oc has sync ,apicallback data is {}", strBuffer.GetString());
    return strBuffer.GetString();
}

CallBackData CallBackManage::CacheCmJsonSwitch(const std::string &data)
{
    CallBackData cm_info;
    rapidjson::Document doc;
    rapidjson::Value root(rapidjson::kObjectType);
    ;
    doc.Parse(data.c_str());
    if (doc.IsObject() && doc.HasMember("records"))
    {
        root = doc["records"][0];
        // cm_info.uuid = root["uuid"].GetString();
        cm_info.task_id = std::to_string(root["task_id"].GetInt());
        cm_info.call_result = root["call_result"].GetInt();
        cm_info.manual_status = root["manual_status"].GetInt();
        cm_info.call_time = root["call_time"].GetInt();
        cm_info.duration_time = root["duration"].GetInt();
        cm_info.answer_time = std::to_string(root["answer_time"].GetInt());
        cm_info.hangup_time = std::to_string(root["hangup_time"].GetInt());
        cm_info.transfer_number = root["transfer_number"].GetString();
        cm_info.transfer_duration = root["transfer_duration"].GetInt();
        cm_info.record_url = root["record_url"].GetString();
        cm_info.cc_number = root["cc_number"].GetString();
    }
    return cm_info;
}
void CallBackManage::CallBackAction(const std::string &data, const std::string &url)
{
}

std::string CallBackManage::GetCallRecordFromCm(const std::string url)
{
    return "";
}
