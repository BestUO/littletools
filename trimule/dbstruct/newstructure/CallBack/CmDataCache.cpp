#include <iostream>
#include "CmDataCache.h"
#include <ctime>
#include "../../../redispool/redisclient.h"

void DataCache::PollingQueue()
{
    std::vector<std::string> list;
    RedisOperate instance;
    std::string list_name = "cm_id_cluster";
    ormpp::dbng<ormpp::mysql> mysqlclient;
    sqlconnect conne = SettingParser::GetSettinghParser("conf/trimule_config.json");
    std::string port = to_string(conne.db_port);
    try
    {
        if(!mysqlclient.connect(conne.host.c_str(), conne.user.c_str(), conne.password.c_str(), conne.db.c_str(), conne.db_timeout, conne.db_port))
        throw 1;
    }
    catch (int i)
    {
        if(i)
            LOGGER->info("mysql maybe error ,please take a check ");
        return;
    }
    
    int sleep_judge = 0;
    int time = SettingParser::GetSleepTime("conf/trimule_config.json");
    LOGGER->info("PollingQueue sleep time is {} second", time);
    while (true)
    {
        if (list.empty())
        {
            list = instance.GetListFromRedis(list_name);
            if (list.empty())
            {
                sleep(time);
                continue;
            }
        }
        for (auto &key : list)
        {
            LOGGER->info("from {} key is {}", list_name, key);
            IdMuster muster = ParseCmId(key);
            if (muster.time != "oc" && muster.time != "web" && muster.time != "now")
            {
                std::string cm_data_cache = instance.GetValue(key);
                
                if (!cm_data_cache.empty())
                {
                    CallBackData data = CacheCmJsonSwitch(cm_data_cache);
                    CallBackRules rule = GetRulesFromRedis(data.eid, data.task_id);
                    std::string wherecondition = R"(id = ')" + muster.calllog_id + R"(')";
                    PrepareId(data, wherecondition, mysqlclient);

                    if (OC_sync_judge(muster.calllog_id, mysqlclient) || CheckTimeOut(muster))
                    {
                        instance.DelKey(key);
                        instance.LREMForList(list_name, {key});
                        GetOCSyncData(data, mysqlclient);
                        if (CallBackJudge(rule, data))
                        {
                            // callback
                            std::string caback_data = MergeCacheJson(data, cm_data_cache);
                            CallBackManage::CacheCmData(data, caback_data, 2, mysqlclient);
                        }
                    }
                    else
                        sleep(time);
                }
                else
                {
                    instance.LREMForList(list_name, {key});
                    LOGGER->info("null cm_data_cache ,donot callback");
                }
            }
        }
        list.clear();
    }
}

void DataCache::OcWebPollingQueue()
{
    std::vector<std::string> list;
    RedisOperate instance;
    std::string list_name = "cm_id_cluster_ocweb";
    ormpp::dbng<ormpp::mysql> mysqlclient;
    sqlconnect conne = SettingParser::GetSettinghParser("conf/trimule_config.json");

    try
    {
        if(!mysqlclient.connect(conne.host.c_str(), conne.user.c_str(), conne.password.c_str(), conne.db.c_str(), conne.db_timeout, conne.db_port))
        throw 1;
    }
    catch (int  i)
    {
        if(i)
        {
            LOGGER->info("mysql maybe error ,please take a check ");
            cout<<"mysql error"<<endl;
        }
        return;
    }

    int time = SettingParser::GetSleepTime("conf/trimule_config.json");

    LOGGER->info("OcWebPollingQueue  sleep time is {} second", time);
    while (true)
    {
        if (list.empty())
        {
            list = instance.GetListFromRedis(list_name);
            if (list.empty())
            {
                sleep(time);
                continue;
            }
        }
        for (auto &key : list)
        {
            LOGGER->info("from {} key is {}", list_name, key);
            IdMuster muster = ParseCmId(key);

            std::string wherecondition = "";
            if (muster.time == "cm_whole")
                wherecondition = R"(cc_number = ')" + muster.calllog_id + R"(')";
            else
                wherecondition = R"(id = ')" + muster.calllog_id + R"(')";

            if (muster.calllog_id != "")
            {
                CallBackData data;
                PrepareId(data, wherecondition, mysqlclient); // muster.calllog_id  maybe  cc_number
                if (!OC_sync_judge(data.calllog_id, mysqlclient) && !CheckTimeOut(muster))
                {
                    LOGGER->info("calllog {} not sync ", data.calllog_id);
                    sleep(time);
                    continue;
                }

                std::string message_from_cm = "";
                if (muster.time == "web")
                    message_from_cm = GetCallRecordFromCm(data, mysqlclient);
                else if (muster.time == "oc")
                {}
                else if (muster.time == "cm_whole")
                    message_from_cm = instance.GetValue(key);

                if (data.calllog_id != "" && !message_from_cm.empty())
                    UpdateMessage::HandleSQL(message_from_cm, mysqlclient, wherecondition, data.calllog_id);
            }
            else
                LOGGER->info("cc_number or calllog_id is null ,key is {} ,delete it",key);
            instance.DelKey(key);
            instance.LREMForList(list_name, {key});
        }
        list.clear();
    }
}

void DataCache::CallBackActionQueue()
{
    std::vector<std::string> list;
    RedisOperate instance;
    std::string list_name = "cm_id_cluster_now";
    int time = SettingParser::GetSleepTime("conf/trimule_config.json");
    LOGGER->info("CallBackActionQueue  sleep time is {} second", time);
    while (true)
    {
        if (list.empty())
        {
            list = instance.GetListFromRedis(list_name);
            if (list.empty())
            {
                sleep(time);
                continue;
            }
        }
        for (auto &key : list)
        {
            LOGGER->info("from {} key is {}", list_name, key);
            IdMuster muster = ParseCmId(key);
            if (muster.time == "now")
            {
                std::string data_cache = instance.GetValue(key);
                LOGGER->info("begin call back!!!  data is {}", data_cache);
                if(!data_cache.empty())
                    CallBackAction(data_cache, muster.url);
            }
            instance.DelKey(key);
            instance.LREMForList(list_name, {key});
        }
        list.clear();
    }
}

IdMuster DataCache::ParseCmId(std::string_view key)
{
    IdMuster muster;
    int pos1 = 0, pos2 = 0;

    int delimcount = count(key.begin(), key.end(), '-');

    if (delimcount == 1)
    {

        pos1 = key.rfind("-");
        muster.calllog_id = key.substr(0, pos1);
        muster.time = key.substr(pos1 + 1, (key.size() - pos1 - 1));

    }
    else
    {
        pos1 = key.rfind("-");
        int pos2 = key.substr(0,pos1).rfind("-");
        muster.calllog_id = key.substr(0, pos2);
        muster.url = key.substr(pos2 + 1, pos1-pos2-1);
        muster.time = key.substr(pos1 + 1, pos2 - pos1 - 1);
    }

    return muster;
}

int stoi_l(const std::string &str)
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


bool DataCache::CheckTimeOut(const IdMuster &muster)
{
    auto now = time(NULL);
    std::stringstream sstream;
    sstream << now;
    std::string time_ = sstream.str();
    if (stoi(time_) - stoi_l(muster.time) >= 600)
    {
        LOGGER->info("Callback_data is timeout,force callback");
        return 1;
    }
    else
        return 0;
}

void DataCache::CheckUnUpdateId(const std::string &eid, const std::string &mini_time, const std::string &max_time)
{
    ormpp::dbng<ormpp::mysql> mysqlclient;
    sqlconnect conne = SettingParser::GetSettinghParser("conf/trimule_config.json");
    try
    {
        if(!mysqlclient.connect(conne.host.c_str(), conne.user.c_str(), conne.password.c_str(), conne.db.c_str(), conne.db_timeout, conne.db_port))
        throw 1;
    }
    catch (int  i)
    {
        if(i)
        {
            LOGGER->info("mysql maybe error ,please take a check ");
            cout<<"mysql error"<<endl;
        }
        return;
    }
    // mysqlclient.connect(conne.host.c_str(), conne.user.c_str(), conne.password.c_str(), conne.db.c_str(), conne.db_timeout, conne.db_port);

    auto res = mysqlclient.query<std::tuple<std::string>>("SELECT id from calllog WHERE enterprise_uid = " + eid + " and create_time >= " + mini_time + " and create_time <= " + max_time + " and id in (select calllog_id from aicall_calllog_subsidiary where update_status = 0) ");

    LOGGER->info("code is SELECT id from calllog WHERE enterprise_uid =  {}  and create_time >= {} and create_time <= {} and id in (select calllog_id from aicall_calllog_subsidiary where update_status = 0) ", eid, mini_time, max_time);

    LOGGER->info("Trimule may has core,so we has found {} calllog has not update now.", res.size());

    for (int i = 0; i < res.size(); i++)
    {
        CallBackData data;
        data.calllog_id = std::get<0>(res[i]);
        LOGGER->info("calllog is {}", data.calllog_id);
        CacheCmData(data, "", 1, mysqlclient);
    }
}

CallInfo MessageProcess::GetCallRecord(std::string_view real_data, int framework_class)
{
    CallInfo result;
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(real_data.data(), root))
    {
        Json::Value data = root;

        auto remove = [](std::string str) -> int
        {
            std::string::size_type pos = str.find("秒");
            int judge = 0;
            for (unsigned int nos = 0; nos != pos; nos++)
            {
                if ((str[nos]) > '0' && str[nos] <= '9')
                    return 2;
            }
            return 1;
        };
        auto remove_Chinese = [](std::string str) -> int
        {
            int num;
            std::string::size_type pos = str.find("秒");
            if (pos == std::string::npos)
                return stoi(str);
            for (int i = 0; i < str.size(); i++)
            {
                if (str[i] < 0)
                {
                    str.erase(i, i + 2);
                    break;
                }
            }
            return stoi(str);
        };
        // if (!data.isObject() || data["enterprise_type"].isNull() || data["record_url"].isNull() ||
        //     data["records"].isNull())
        //     return result;
        auto enterprise_type = (data)["enterprise_type"];
        auto record_url = (data)["record_url"];
        auto records = (data)["records"];

        if (record_url.isString() && !record_url.isNull())
            result.record_url = record_url.asString();

        if (enterprise_type.isInt())
            result.enterprise_type = enterprise_type.asInt();
        int duration = 0;
        int now_flow = 0;
        if (records.isArray() && records.size() > 0)
        {
            for (unsigned int i = 0; i < records.size(); ++i)
            {
                auto &record = records[i];
                if (!record.isObject())
                    return result;

                auto &call_type = record["call_type"];
                int type = call_type.isString() && !call_type.isNull() ? stoi(call_type.asString()) : 1;
                auto &duration_time = !record["valid_duration"].isNull() == true ? record["valid_duration"] : record["duration_time"];
                auto &end_time = record["end_time"];
                auto &call_state = record["call_state"];
                auto &start_time = record["start_time"];

                if (record["cc_number"].isString())
                    result.cc_number = record["cc_number"].asString();

                if (record["customer_fail_reason"].asString() != "0")
                    result.customer_fail_reason = stoi(record["customer_fail_reason"].asString());

                result.stop_reason = record["stop_reason"].asInt();

                result.flow_number = record["flow_number"].isNull() ? -1 : record["flow_number"].asInt();

                result.hangup_type = GetHangupType(result.stop_reason, result.customer_fail_reason);
                
                if (result.flow_number != 0 && result.flow_number >= now_flow) // transfer_manual
                {
                    now_flow = result.flow_number;

                    auto &dialing = record["dialing"];
                    auto &transfer_confirm_time = record["conversation_time"];
                    auto &send_query_msg_timestamp = record["query_msg_time"];
                    auto &send_invite_timestamp = record["invite_time"];
                    auto &transfer_manual_cost = record["seat_ring_duration"];
                    auto &conversation_time = record["conversation_time"];
                    auto &initiate_time = record["initiate_time"];

                    int conversation_type = conversation_time.asString() == "" ? 1 : (conversation_time.asString() == "0"?1:0);

                    result.manual_type = GetManualType(result.stop_reason, result.customer_fail_reason, conversation_type);

                    int motinor_duration = 0;
                    if (conversation_time.asString() != "" && conversation_time.asString() != "0" && transfer_confirm_time.asString() != "" && transfer_confirm_time.asString() != "0")
                    {
                        motinor_duration = stoi(conversation_time.asString()) - stoi(transfer_confirm_time.asString());
                    }

                    if (transfer_manual_cost.isString())
                    {
                        int duration = 0;
                        if(transfer_manual_cost.asString() != "" && transfer_manual_cost.asString() != "0")
                            duration = stoi(transfer_manual_cost.asString());//oc transfer_manual_cost = ring_duration + monitor_duration
                        
                        result.transfer_manual_cost = std::to_string(duration + motinor_duration);
                    }

                    if (dialing.isString())
                        result.transfer_number = dialing.asString();
                    if (duration_time.isString())
                        result.transfer_duration = remove_Chinese(duration_time.asString());

                    if (end_time.isString())
                        result.transfer_end_time = end_time.asString();
                    if (start_time.isString())
                        result.transfer_start_time = initiate_time.asString();

                    if (transfer_confirm_time.isString())
                        result.transfer_confirm_time = transfer_confirm_time.asString();
                    if (call_state.isString())
                        result.transfer_call_state = stoi(call_state.asString(), 0);

                    if (send_query_msg_timestamp.isString())
                        result.send_query_msg_timestamp = std::string(send_query_msg_timestamp.asString());
                    if (send_invite_timestamp.isString())
                        result.send_invite_timestamp = std::string(send_invite_timestamp.asString());

                    // int monitor_duration = 0;
                    // if(stoi)
                }
                else if(result.flow_number == 0 )// ai_
                {
                    result.call_result = GetCallResult(result.stop_reason, result.customer_fail_reason);
                    

                    auto &confirm_time = record["conversation_time"];
                    auto &call_type = record["call_type"];
                    auto &ring_time = record["customer_ring_duration"];

                    if (ring_time.isString())
                        result.ring_time = ring_time.asString();
                    if (confirm_time.isString())
                        result.confirm_time = std::string(confirm_time.asString());
                    if (end_time.isString())
                        result.end_time = end_time.asString();

                    if (duration_time.isString())
                        result.duration_time = remove_Chinese(duration_time.asString());

                    if (call_state.isString())
                        result.call_state = stoi(call_state.asString(), 0);
                    if (start_time.isString())
                        result.start_time = start_time.asString();
                    if (call_type.isString())
                        result.call_type = stoi(call_type.asString());

                    result.switch_number = !record["switch_number"].isNull() ? record["switch_number"].asString() : "";
                }
                if (framework_class == 2)
                {
                    auto str_associaction = [=](string str, int class_judge) -> string
                    {
                        if (stoi(str) >= 0 && stoi(str) <= 9)
                        {
                            string str1 = class_judge == 2 ? "20" : "10";
                            return str1 + str;
                        }
                        else
                        {
                            string str1 = class_judge == 2 ? "2" : "1";
                            return str1 + str;
                        }
                    };
                    if (result.customer_fail_reason != 0)
                    {
                        result.call_state = stoi(str_associaction(std::to_string(result.customer_fail_reason), 2));
                    }
                    else if (result.stop_reason != 0)
                    {
                        result.call_state = stoi(str_associaction(to_string(result.stop_reason), 1));
                    }
                }
            }
        }
    }
    return result;
}

int MessageProcess::GetManualType(int &stop_reason, int &customer_fail_reason, int &conversation_type)
{
    if (conversation_type == 1)
        return ManualType::ManualNotAnswered;

    if (customer_fail_reason == 9)

        return ManualType::ManualRefuse;

    switch (stop_reason)
    {
        case 9:
        case 10:
            return ManualType::NoFreeSeats;
        case 43:
        case 12:
        case 14:
        case 44:
            return ManualType::HangUpDuringTransfer;
        case 13:
        case 35:
        case 7:
            return ManualType::ManualNotAnswered;
        case 25:
        case 26:
        case 31:
        case 29:
        case 30:
            return ManualType::Answered;
        default:
            return ManualType::NoType;
    }
}

int MessageProcess::GetHangupType(int &stop_reason, int &customer_fail_reason)
{
    if (stop_reason == 25)
        return HangUpType::UserHangUp;
    else
        return HangUpType::AiHangUp;
}
int MessageProcess::GetCallResult(int &stop_reason, int &customer_fail_reason)
{
    switch (customer_fail_reason)
    {
        case 9:
            return CallResult::CALL_REJECT;
        case 8:
            return CallResult::NOT_EXIST;
        case 2:
            return CallResult::POWER_OFF;
        case 11:
            return CallResult::OUT_OF_SERVICE;
        case 13:
            return CallResult::NOT_IN_SERVICE;
        case 37:
            return CallResult::BUSY;
        case 30:
        case 31:
            return CallResult::SEAT_ARREARAGE;
        default:
            break;
    }
    switch (stop_reason)
    {
        case 25:
        case 26:
        case 31:
            return CallResult::CALL_SUCCESS;
        case 19:
            return CallResult::UNCONNECTED;
        default:
            return CallResult::CALL_FAIL;
    }
}

auto MessageProcess::GetIdsWithCCNumber(ormpp::dbng<ormpp::mysql> &mysqlclient,const std::string &cc_number)
{
    using type = std::tuple<std::string, std::string, std::string, std::string, std::string, std::string>;
    auto res = mysqlclient.query<type>("select id, clue_id ,task_id,enterprise_uid,call_count,caller_phone from calllog where cc_number=?", cc_number);
    LOGGER->info("select id, clue_id ,task_id,enterprise_uid,call_count,caller_phone from calllog where cc_number={}", cc_number);
    if (res.size())
        return res[0];
    else
        return type();
}

std::string MessageProcess::UpdateAllInfo(std::string_view message, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
	LOGGER->info("UpdateAllInfo message {}", message);
	CallInfo callog = GetCallRecord(message, 2);

    auto [calllog_id,clue_id,task_id,eid,call_count,caller_phone] = GetIdsWithCCNumber(mysqlclient,callog.cc_number);
    UpdateMessage::UpdateCalllog(callog, calllog_id, mysqlclient);
    UpdateMessage::UpdateOutCallClue(callog, clue_id, mysqlclient);
    UpdateMessage::UpdateAiCalllogExtension(callog, calllog_id, mysqlclient);
    UpdateMessage::UpdateAicallCalllogSubsidiary(calllog_id, mysqlclient);
    LOGGER->info("calllog_id is {},clue_id is {},task_id is {},eid is {}", calllog_id, clue_id, task_id, eid);
    return calllog_id;
}

std::string MessageProcess::CollectInfoXML2JSON(const std::string &xml)
{
    std::string jsonstr = "[]";
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError error = doc.Parse(xml.c_str());
    auto root = doc.RootElement();
    if (root)
    {
        auto collect_info_element = root->FirstChildElement("collect_info");
        if (collect_info_element)
        {
            rapidjson::StringBuffer buffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
            writer.StartArray();
            auto child = collect_info_element->FirstChildElement("item");
            while (child)
            {
                writer.StartObject();
                writer.Key("question_name");
                writer.String(child->Attribute("match_standard_question"));
                writer.Key("match_result");
                writer.String(child->Attribute("match_cluster_name"));
                writer.Key("real_answer");
                writer.String(child->GetText() == nullptr ? "" : child->GetText());
                writer.EndObject();
                child = child->NextSiblingElement("item");
            }
            writer.EndArray();
            jsonstr = buffer.GetString();
        }
    }
    return jsonstr;
}

std::string MessageProcess::GenerateCallBackString(CallBackDataNew &data)
{
    rapidjson::Document doc;
    rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();
    rapidjson::Value val;

    doc.SetObject();
    doc.AddMember("uuid", val.SetString(data.uuid.c_str(), allocator), allocator);
    rapidjson::Value records(rapidjson::Type::kArrayType);
    rapidjson::Value record(rapidjson::kObjectType);
    record.AddMember("calllog_id", data.record.calllog_id, allocator);
    record.AddMember("switch_number", val.SetString(data.record.switch_number.c_str(), allocator), allocator);
    record.AddMember("hangup_type", data.record.hangup_type, allocator);
    record.AddMember("call_result", data.record.call_result, allocator);
    record.AddMember("manual_status", data.record.manual_status, allocator);
    record.AddMember("call_time", data.record.call_time, allocator);
    record.AddMember("duration", data.record.duration, allocator);
    record.AddMember("answer_time", data.record.answer_time, allocator);
    record.AddMember("hangup_time", data.record.hangup_time, allocator);
    record.AddMember("transfer_number", val.SetString(data.record.transfer_number.c_str(), allocator), allocator);
    record.AddMember("transfer_duration", data.record.transfer_duration, allocator);
    record.AddMember("record_url", val.SetString(data.record.record_url.c_str(), allocator), allocator);
    record.AddMember("cc_number", val.SetString(data.record.cc_number.c_str(), allocator), allocator);

    record.AddMember("script_name", val.SetString(data.record.script_name.c_str(), allocator), allocator);
    record.AddMember("callee_phone", val.SetString(data.record.callee_phone.c_str(), allocator), allocator);
    record.AddMember("caller_phone", val.SetString(data.record.caller_phone.c_str(), allocator), allocator);
    record.AddMember("calllog_txt", val.SetString(data.record.calllog_txt.c_str(), allocator), allocator);
    record.AddMember("intention_type", data.record.intention_type, allocator);
    record.AddMember("label", val.SetString(data.record.label.c_str(), allocator), allocator);
    record.AddMember("call_count", data.record.call_count, allocator);
    record.AddMember("match_global_keyword", val.SetString(data.record.match_global_keyword.c_str(), allocator), allocator);
    record.AddMember("clue_no", val.SetString(data.record.clue_no.c_str(), allocator), allocator);
    record.AddMember("task_id", data.record.task_id, allocator);
    record.AddMember("collect_info", val.SetString(CollectInfoXML2JSON(data.record.calllog_txt).c_str(), allocator), allocator);
    record.AddMember("buttons", val.SetString(data.record.buttons.c_str(), allocator), allocator);

    records.PushBack(record, allocator);
    doc.AddMember("records", records, allocator);

    rapidjson::StringBuffer strBuffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strBuffer);
    doc.Accept(writer);
    std::string res = strBuffer.GetString();

    return res;
}

std::string MessageProcess::GetCallBackString(std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    // CallBackManage::CallBackHandle(callog, result, mysqlclient);
    CallBackDataNew data;
    GetLabelInfo(data,calllog_id,mysqlclient);
    GetCallLogInfo(data,calllog_id,mysqlclient);
    GetCallLogExtensionInfo(data,calllog_id,mysqlclient);
    GetTaskInfo(data,mysqlclient);
    GetClueInfo(data,mysqlclient);
    return GenerateCallBackString(data);
}

void MessageProcess::GetLabelInfo(CallBackDataNew &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    std::string sql = "select label from aicall_calllog_label where calllog_id=?";
    auto results = mysqlclient.query<std::tuple<std::string>>(sql,calllog_id.data());
    for(auto &&result:results)
        data.record.label += std::get<0>(result) + ",";
    if(!data.record.label.empty())
        data.record.label.pop_back();
}

void MessageProcess::GetCallLogInfo(CallBackDataNew &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    auto tmp = MakeMyTuple("task_id", "script_name", "callee_phone", "cc_number", "calllog_txt", "call_result", "call_time", "duration", "intention_type", "manual_status",
        "call_count", "call_record_url", "answer_time", "hangup_time", "match_global_keyword", "transfer_duration", "transfer_number","buttons","caller_phone","clue_id");
    auto results = mysqlclient.query<decltype(tmp)>("select " + TupleToString(tmp) + " from calllog where id=?",calllog_id.data());

    data.record.calllog_id = stoi(calllog_id.data());
    data.record.task_id = stoi(std::get<0>(results[0]));
    data.record.script_name = std::get<1>(results[0]);
    data.record.callee_phone = std::get<2>(results[0]);
    data.record.cc_number = std::get<3>(results[0]);
    data.record.calllog_txt = std::get<4>(results[0]);
    data.record.call_result = stoi(std::get<5>(results[0]));
    data.record.call_time = stoi(std::get<6>(results[0]));
    data.record.duration = stoi(std::get<7>(results[0]));
    data.record.intention_type = stoi(std::get<8>(results[0]));
    data.record.manual_status = stoi(std::get<9>(results[0]));
    data.record.call_count = stoi(std::get<10>(results[0]));
    data.record.record_url = std::get<11>(results[0]);
    data.record.answer_time = stoi(std::get<12>(results[0]));
    data.record.hangup_time = stoi(std::get<13>(results[0]));
    data.record.match_global_keyword = std::get<14>(results[0]);
    data.record.transfer_duration = stoi(std::get<15>(results[0]));
    data.record.transfer_number = std::get<16>(results[0]);
    data.record.buttons = std::get<17>(results[0]);
    data.record.caller_phone = std::get<18>(results[0]);
    data.clue_id = std::get<19>(results[0]);
}

void MessageProcess::GetCallLogExtensionInfo(CallBackDataNew &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    auto tmp = MakeMyTuple("switch_number", "hangup_type", "call_state");
    auto results = mysqlclient.query<decltype(tmp)>("select " + TupleToString(tmp) + " from aicall_calllog_extension where calllog_id=?",calllog_id.data());
    data.record.switch_number = std::get<0>(results[0]);
    data.record.hangup_type = std::stoi(std::get<1>(results[0]));
    data.record.call_state = std::stoi(std::get<2>(results[0]));
}

void MessageProcess::GetTaskInfo(CallBackDataNew &data, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    std::string sql = "select uuid from outcall_task where id=?";
    auto results = mysqlclient.query<std::tuple<std::string>>(sql,data.record.task_id);
    data.uuid += std::get<0>(results[0]);
}

std::vector<std::string> MessageProcess::Split(const std::string& str, const std::string& delims)
{
    std::vector<std::string> output;
    auto first = std::cbegin(str);

    while (first != std::cend(str))
    {
        const auto second = std::find_first_of(first, std::cend(str), std::cbegin(delims), std::cend(delims));

        if (first != second)
            output.emplace_back(first, second);

        if (second == std::cend(str))
            break;

        first = std::next(second);
    }

    return output;
}

void MessageProcess::GetClueInfo(CallBackDataNew &data, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    std::string sql = "select alias from outcall_clue where id=?";
    auto results = mysqlclient.query<std::tuple<std::string>>(sql, data.clue_id);
    auto alias = std::get<0>(results[0]);
//    1：clue_no=abc||efg 回调结果:uuid=abc clue_no=efg
//    2：clue_no=abc|| 回调结果:uuid=abc clue_no=空
//    3：clue_no=abc 回调结果:clue_no=abc uuid还是之前任务的uuid
    auto tmp = Split(alias,"||");
    if(tmp.size()==2)
    {
        data.uuid = tmp[0];
        data.record.clue_no = tmp[1];
    }
    else if(tmp.size()==1 && tmp[0] != alias)
        data.uuid = tmp[0];
    else
        data.record.clue_no = alias;
}

std::string MessageProcess::GetCallRecordFromCm(std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{

//     std::string command = "select server_ip,https_port,enterprise_id from ai.enterprise_info where id = " + data.eid + " limit 1;";
//     std::string server_ip = "";
//     ushort https_port = 0;
//     std::string enterprise_id = "";
//     auto ip_port = mysqlclient.query<std::tuple<std::string, std::string, std::string>>(command);
//     LOGGER->info("command is {}", command);
//     if (data.cc_number == "")
//     {
//         std::string command = "select cc_number from calllog where id = " + data.calllog_id + " limit 1;";
//         LOGGER->info("command is {}", command);
//         auto res = mysqlclient.query<std::tuple<std::string>>(command);
//         if (res.size())
//             data.cc_number = std::get<0>(res[0]);
//         else
//             LOGGER->info("cannot find cc_number");
//     }
//     if (ip_port.size())
//     {
//         server_ip = std::get<0>(ip_port[0]);
//         https_port = static_cast<ushort>(stoi_s(std::get<1>(ip_port[0])));
//         enterprise_id = std::get<2>(ip_port[0]);
//         std::string url = "https://" + server_ip + ":" + std::to_string(https_port);
//         LOGGER->info("We prepare to get into the new program and ccgeid is{}", enterprise_id);
//         std::string post_param = "ccgeid=" + enterprise_id + "&cc_number=" + data.cc_number;
//         url = url + "/robot/call_record?" + post_param;

//         std::string key_name = "robot_cm_default_secret";
//         std::string signatureKey = "robot.cm.default.secret.da123456780e6fy5";
//         auto pair = GetSignkey(key_name, signatureKey, stoi_s(data.eid), mysqlclient);
//         if (!pair.first.empty())
//         {
//             std::string response = PostUrl2(pair.first, pair.second, url, post_param, false);
//             std::string real_response;
//             WebOcApiData result;
//             rapidjson::Document doc;
//             rapidjson::Value root(rapidjson::kObjectType);
//             doc.Parse(response.c_str());

//             if (doc.IsObject() && doc.HasMember("data"))
//             {
//                 rapidjson::StringBuffer str_buf;
//                 rapidjson::Writer<rapidjson::StringBuffer> writer(str_buf);
//                 doc["data"].Accept(writer);
//                 real_response = std::move(str_buf.GetString());
//             }

//             LOGGER->info("cm response is {}  parse data is {}", response, real_response);
//             return real_response;
//         }
//     }
//     LOGGER->info("no response");
//     return "";
}