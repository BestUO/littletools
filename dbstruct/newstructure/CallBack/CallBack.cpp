#include "CallBack.h"
#include <vector>
#include <iostream>
 void CallBackManage::GetOCSyncData(ormpp::dbng<ormpp::mysql> &mysql,CallBackData &data)
 {
    std::string calllog_id,clue_id,task_id;
    std::string outcall_clue_rule = SetMySqlRules({"clue_id"},{clue_id});
    auto result_outcall_clue = mysql.query<outcall_clue>(outcall_clue_rule);
    TEST_CHECK(result_outcall_clue.size()==1);

    std::string calllog_rule = SetMySqlRules({"calllog_id"},{calllog_id});
    auto result_calllog = mysql.query<calllog>(calllog_rule);
    TEST_CHECK(result_calllog.size()==1);
    

//clue
    data.label = result_outcall_clue[0].label;
    data.clue_no = result_outcall_clue[0].alias;
//calllog
    data.intention_type = result_calllog[0].intention_type;
    data.match_global_keyword = result_calllog[0].match_global_keyword;
    data.calllog_id = result_calllog[0].id;
    data.task_id = result_calllog[0].task_id;
    data.callee_phone = result_calllog[0].callee_phone;
    data.calllog_txt = result_calllog[0].calllog_txt;
    data.call_count = result_calllog[0].call_count;
    data.buttons = result_calllog[0].buttons;
    data.script_name = result_calllog[0].script_name;
    data.caller_phone = result_calllog[0].caller_phone;

//task
    std::string outcall_task_rule = SetMySqlRules({"task_id"},{task_id});
    auto result_outcall_task = mysql.query<outcall_task>(outcall_task_rule);
    TEST_CHECK(result_outcall_task.size()==1);
    data.uuid = result_outcall_task[0].uuid;
 }

     std::string CallBackManage::SetMySqlRules(std::vector<std::string> rule_name,std::vector<std::string> rule)
     {
        std::string rules;
        rules += "where ";
        for(int i=0;i<rule_name.size();i++)
        {
            rules+=rule_name[i];
            rules+=" = ";
            rules+= "\""+rule[i]+"\"";
        } 
     }

     void CallBackManage::CallBackHandle(ormpp::dbng<ormpp::mysql> &mysql,CallInfo & cm_data,const std::tuple<std::string,std::string,std::string,std::string> &id_cluster)
    {
        CallBackData data;
        GetOCSyncData(mysql,data);
        CmDataSwitch(cm_data,data);
    }
    void CallBackManage::CmDataSwitch(CallInfo & cm_data,CallBackData &data)
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

CallBackRules MakeCallBackRulesFromMySql(ormpp::dbng<ormpp::mysql> &mysql,const std::tuple<std::string,std::string,std::string,std::string> &id_cluster)
{
    CallBackRules rules;
    rules.eid = stoi(std::get<IdCluster::EnterpriseUid>(id_cluster));
    rules.task_id = stoi(std::get<IdCluster::TaskId>(id_cluster));

}