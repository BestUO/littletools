#include "CallBack.h"
#include <vector>
#include <iostream>

void CallBackManage::CallBackHandle(ormpp::dbng<ormpp::mysql> &mysql, CallInfo &cm_data, const std::tuple<std::string, std::string, std::string, std::string> &id_cluster)
{
    LOGGER->info("begin CallBackHandle");
    CallBackData data;
    CallBackRules rule;
    data.eid = std::get<IdCluster::EnterpriseUid>(id_cluster);
    data.task_id = std::get<IdCluster::TaskId>(id_cluster);
    data.clue_id = std::get<IdCluster::ClueId>(id_cluster);
    data.calllog_id = std::get<IdCluster::CalllogId>(id_cluster);
    CmDataSwitch(cm_data, data);
    if(OC_sync_judge(std::get<IdCluster::CalllogId>(id_cluster)))
    {
        GetOCSyncData(mysql, data);
        RedisOperate *client = RedisOperate::getInstance();
        std::string locate = data.eid+"-"+data.task_id;
        if(client->SearchRules(locate)!="null")
        {
            GetRulesFromRedis(rule);
            if(CallBackJudge(rule,data))
            {
                //callback
            }

        } else {
            rule = MakeCallBackRulesFromMySql(mysql,id_cluster);
            client->CacheRules(locate,SetRulesRedisCache(rule));
            if(CallBackJudge(rule,data))
            {
                //callback
            }
        }

    } else{
        //queue
    }
}
bool CallBackManage::OC_sync_judge(const std::string &calllog_id)
{
    MySql * instance = MySql::getInstance();
    auto sync_judge = instance->mysqlclient.query<std::tuple<int>>("select sync_judge from aicall_calllog_extension where calllog_id = "+calllog_id);
    return std::get<0>(sync_judge[0]);
}
void CallBackManage::GetOCSyncData(ormpp::dbng<ormpp::mysql> &mysql, CallBackData &data)
{
    LOGGER->info("GetOCSyncData eid is {}",data.eid);
    std::string where_eid = " and enterprise_uid = "+data.eid;
    std::string outcall_clue_rule = "where id = "+data.clue_id;
    auto result_outcall_clue = mysql.query<outcall_clue>(outcall_clue_rule);
    TEST_CHECK(result_outcall_clue.size() == 1);

    std::string calllog_rule = "where id = "+data.calllog_id+where_eid;
    auto result_calllog = mysql.query<calllog>(calllog_rule);
    TEST_CHECK(result_calllog.size() == 1);

    // clue
    data.label = result_outcall_clue[0].label;
    data.clue_no = result_outcall_clue[0].alias;
    // calllog
    data.intention_type = result_calllog[0].intention_type;
    data.match_global_keyword = result_calllog[0].match_global_keyword;
    data.calllog_id = result_calllog[0].id;
    // data.task_id = result_calllog[0].task_id;
    data.callee_phone = result_calllog[0].callee_phone;
    data.calllog_txt = result_calllog[0].calllog_txt;
    data.call_count = result_calllog[0].call_count;
    data.buttons = result_calllog[0].buttons;
    data.script_name = result_calllog[0].script_name;
    data.caller_phone = result_calllog[0].caller_phone;

    // task
    std::string outcall_task_rule = "where id = "+data.task_id+where_eid;
    auto result_outcall_task = mysql.query<outcall_task>(outcall_task_rule);
    TEST_CHECK(result_outcall_task.size() == 1);
    data.uuid = result_outcall_task[0].uuid;
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
}

CallBackRules CallBackManage::MakeCallBackRulesFromMySql(ormpp::dbng<ormpp::mysql> &mysql, const std::tuple<std::string, std::string, std::string, std::string> &id_cluster)
{

    struct outcall_task
    {
        int uuid; // 0:donot callback,1:callback
        std::string auto_recall_scenes;
        int auto_recall_max_times;
        int auto_recall_status;
    };
    struct aicall_config
    {
        int api_callback_scene_status;
    };

    CallBackRules rules;
    rules.eid = stoi(std::get<IdCluster::EnterpriseUid>(id_cluster));
    rules.task_id = stoi(std::get<IdCluster::TaskId>(id_cluster));

    GenerateSQL general_sql;

    std::string calllog_id, clue_id, task_id;
    std::vector<std::string> condition;
    std::vector<std::string> condition_name;
    std::vector<std::string> condition_symbols;
    condition_name.push_back("id");
    condition_name.push_back("enterprise_uid");
    condition.push_back(std::get<IdCluster::TaskId>(id_cluster));
    condition.push_back(std::get<IdCluster::EnterpriseUid>(id_cluster));
    condition_symbols.push_back("=");
    condition_symbols.push_back("=");
    std::string outcall_task_rule = general_sql.MysqlGenerateMySqlCondition(condition, condition_name, condition_symbols);
    auto result_outcall_task = mysql.query<outcall_task>(outcall_task_rule);

    rules.auto_recall_scenes = result_outcall_task[0].auto_recall_scenes;
    rules.auto_recall_status = result_outcall_task[0].auto_recall_status;
    rules.auto_recall_max_times = result_outcall_task[0].auto_recall_max_times;

    std::vector<std::string> condition_;
    std::vector<std::string> condition_name_;
    std::vector<std::string> condition_symbols_;
    condition_name_.push_back("eid");
    condition_.push_back(std::get<IdCluster::EnterpriseUid>(id_cluster));
    condition_symbols_.push_back("=");
    std::string aicall_config_rule = general_sql.MysqlGenerateMySqlCondition(condition_, condition_name_, condition_symbols_);
    auto result_aicall_config = mysql.query<aicall_config>(aicall_config_rule);
    rules.api_callback_scene_status = result_aicall_config[0].api_callback_scene_status;

    ParseIntetionAndCallResult(rules);
    return rules;
    //
}
void CallBackManage::ParseIntetionAndCallResult(CallBackRules &rules)
{
    LOGGER->info("ParseIntetionAndCallResult, rules is {}", rules.auto_recall_scenes);
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
    std::string result;
    std::string api_callback_scene_status = rules.api_callback_scene_status;
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();

    doc.AddMember("detail_judge", rules.api_callback_scene_status, allocator);
    doc.AddMember("global_judge", rules.global_judge, allocator);
    doc.AddMember("scope_judge", rules.scope_judge, allocator);
    doc.AddMember("uuid", rules.uuid, allocator);
    doc.AddMember("intention_type_judge", rules.intention_type_judge, allocator);
    doc.AddMember("call_result_judge", rules.call_result_judge, allocator);
    doc.AddMember("auto_recall_status", rules.auto_recall_status, allocator);

    rapidjson::StringBuffer str_buf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(str_buf);
    doc.Accept(writer);
    return str_buf.GetString();
}

bool CallBackManage::GetRulesFromRedis(CallBackRules &rules)
{

    std::string location = std::to_string(rules.eid) + '-' + std::to_string(rules.task_id);

    RedisOperate *client = RedisOperate::getInstance();
    std::string rule = client->SearchRules(location);
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

bool AutoTaskMatch(const CallBackRules &rules, const CallBackData &data)
{
    if (rules.intention_type_judge != "0000000000000")
    {

        if ((rules.intention_type_judge)[stoi(data.intention_type)] == '1')
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
                if (rules.auto_recall_max_times == stoi(data.call_count))
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