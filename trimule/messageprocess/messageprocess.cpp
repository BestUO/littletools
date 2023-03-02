#include "messageprocess.h"
#include "dbupdate.h"
#include "httpclient/HttpRequester.h"
#include "global.hpp"
#include "json/json.h"
#include "tinyxml2/tinyxml2.h"
#include "rapidjson/rapidjson.h"
#include "rapidjson/writer.h"
#include "rapidjson/document.h"

CallInfo MessageProcess::GetCallRecord(std::string_view real_data, int framework_class)
{
    CallInfo result;
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(real_data.data(), root))
    {
        auto enterprise_type = (root)["enterprise_type"];
        auto record_url = (root)["record_url"];
        auto records = (root)["records"];

        if (record_url.isString() && !record_url.isNull())
            result.record_url = record_url.asString();
        if (enterprise_type.isInt())
            result.enterprise_type = enterprise_type.asInt();

        int duration = 0;
        int now_flow = 0;
        if (records.isArray())
        {
            for (unsigned int i = 0; i < records.size(); ++i)
            {
                auto &record = records[i];
                if (!record.isObject())
                    return result;

                auto duration_time = stoi(record["duration_time"].asString());
                auto end_time = record["end_time"].asString();
                auto call_state = record["call_state"].asString();

                result.cc_number = record["cc_number"].asString();
                result.customer_fail_reason = stoi(record["customer_fail_reason"].asString());
                result.stop_reason = record["stop_reason"].asInt();
                now_flow = record["flow_number"].asInt();
                
                if (now_flow != 0) // transfer_manual
                {
                    auto conversation_time = record["conversation_time"].asString();
                    auto initiate_time = record["initiate_time"].asString();

                    result.manual_type = GetManualType(result.stop_reason);
                    result.transfer_manual_cost += std::stoi(record["seat_ring_duration"].asString());//转人工振铃时长
                    result.transfer_duration += duration_time; 
                    result.hangup_type = GetHangupType(result.stop_reason);

                    if(now_flow > result.flow_number)
                    {
                        result.flow_number = now_flow;
                        result.transfer_number = record["dialing"].asString();
                        result.transfer_end_time = end_time;
                        result.transfer_call_state = std::stoi(call_state);
                    }
                }
                else if(now_flow == 0 )// ai_
                {
                    result.call_result = GetCallResult(result.stop_reason, result.customer_fail_reason);
                    
                    result.call_type = std::stoi(record["call_type"].asString());
                    result.ring_time = std::stoi(record["customer_ring_duration"].asString());
                    result.confirm_time = record["conversation_time"].asString();
                    result.end_time = end_time;
                    result.duration_time = duration_time;
                    result.call_state = std::stoi(call_state);
                    result.start_time = std::to_string(std::stoi(end_time)-result.duration_time-result.ring_time);
                    result.switch_number = !record["switch_number"].isNull() ? record["switch_number"].asString() : "";
                    if(result.hangup_type==0)
                        result.hangup_type = GetHangupType(result.stop_reason);
                }
                if (framework_class == 2)
                {
                    auto str_associaction = [=](std::string str, int class_judge) -> std::string
                    {
                        if (std::stoi(str) >= 0 && std::stoi(str) <= 9)
                        {
                            std::string str1 = class_judge == 2 ? "20" : "10";
                            return str1 + str;
                        }
                        else
                        {
                            std::string str1 = class_judge == 2 ? "2" : "1";
                            return str1 + str;
                        }
                    };
                    if (result.customer_fail_reason != 0)
                    {
                        result.call_state = std::stoi(str_associaction(std::to_string(result.customer_fail_reason), 2));
                    }
                    else if (result.stop_reason != 0)
                    {
                        result.call_state = std::stoi(str_associaction(std::to_string(result.stop_reason), 1));
                    }
                }
            }
            if(result.flow_number>0)
            {
                result.transfer_start_time = std::to_string(std::stoi(result.transfer_end_time)-result.transfer_duration-result.transfer_manual_cost);
                result.transfer_confirm_time = std::to_string(std::stoi(result.transfer_end_time)-result.transfer_duration);
            }
        }
    }
    return result;
}

int MessageProcess::GetManualType(int &stop_reason)
{
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
        case 26:
            return ManualType::ManualRefuse;
        case 25:
        case 31:
        case 29:
        case 30:
            return ManualType::Answered;
        default:
            return ManualType::NoType;
    }
}

int MessageProcess::GetHangupType(int &stop_reason)
{
    switch (stop_reason)
    {
        case 7:
        case 25:
            return HangUpType::UserHangUp;
        default:
            return HangUpType::AiHangUp;
    }
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

std::tuple<std::string,std::string,std::string> MessageProcess::UpdateAllInfo(std::string_view message, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
	// LOGGER->info("UpdateAllInfo message {}", message);
	CallInfo callog = GetCallRecord(message, 2);

    auto [calllog_id,clue_id,task_id,eid,call_count,caller_phone] = GetIdsWithCCNumber(mysqlclient,callog.cc_number);
    UpdateMessage::UpdateCalllog(callog, calllog_id, mysqlclient);
    UpdateMessage::UpdateOutCallClue(callog, clue_id, mysqlclient);
    UpdateMessage::UpdateAiCalllogExtension(callog, calllog_id, mysqlclient);
    LOGGER->info("calllog_id is {},clue_id is {},task_id is {},eid is {}", calllog_id, clue_id, task_id, eid);
    return {calllog_id,task_id,eid};
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

std::string MessageProcess::GenerateCallBackString(CallBackData &data)
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
    CallBackData data;
    GetLabelInfo(data,calllog_id,mysqlclient);
    GetCallLogInfo(data,calllog_id,mysqlclient);
    GetCallLogExtensionInfo(data,calllog_id,mysqlclient);
    GetTaskInfo(data,mysqlclient);
    GetClueInfo(data,mysqlclient);
    return GenerateCallBackString(data);
}

void MessageProcess::GetLabelInfo(CallBackData &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    std::string sql = "select label from aicall_calllog_label where calllog_id=?";
    auto results = mysqlclient.query<std::tuple<std::string>>(sql,calllog_id.data());
    for(auto &&result:results)
        data.record.label += std::get<0>(result) + ",";
    if(!data.record.label.empty())
        data.record.label.pop_back();
}

void MessageProcess::GetCallLogInfo(CallBackData &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    auto tmp = MakeMyTuple("task_id", "script_name", "callee_phone", "cc_number", "calllog_txt", "call_result", "call_time", "duration", "intention_type", "manual_status",
        "call_count", "call_record_url", "answer_time", "hangup_time", "match_global_keyword", "transfer_duration", "transfer_number","buttons","caller_phone","clue_id");
    auto results = mysqlclient.query<decltype(tmp)>("select " + TupleToString(tmp) + " from calllog where id=?",calllog_id.data());
    if(!results.empty())
    {
        data.record.calllog_id = std::stoi(calllog_id.data());
        data.record.task_id = std::stoi(std::get<0>(results[0]));
        data.record.script_name = std::get<1>(results[0]);
        data.record.callee_phone = std::get<2>(results[0]);
        data.record.cc_number = std::get<3>(results[0]);
        data.record.calllog_txt = std::get<4>(results[0]);
        data.record.call_result = std::stoi(std::get<5>(results[0]));
        data.record.call_time = std::stoi(std::get<6>(results[0]));
        data.record.duration = std::stoi(std::get<7>(results[0]));
        data.record.intention_type = std::stoi(std::get<8>(results[0]));
        data.record.manual_status = std::stoi(std::get<9>(results[0]));
        data.record.call_count = std::stoi(std::get<10>(results[0]));
        data.record.record_url = std::get<11>(results[0]);
        data.record.answer_time = std::stoi(std::get<12>(results[0]));
        data.record.hangup_time = std::stoi(std::get<13>(results[0]));
        data.record.match_global_keyword = std::get<14>(results[0]);
        data.record.transfer_duration = std::stoi(std::get<15>(results[0]));
        data.record.transfer_number = std::get<16>(results[0]);
        data.record.buttons = std::get<17>(results[0]);
        data.record.caller_phone = std::get<18>(results[0]);
        data.clue_id = std::get<19>(results[0]);
    }
}

void MessageProcess::GetCallLogExtensionInfo(CallBackData &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    auto tmp = MakeMyTuple("switch_number", "hangup_type", "call_state");
    auto results = mysqlclient.query<decltype(tmp)>("select " + TupleToString(tmp) + " from aicall_calllog_extension where calllog_id=?",calllog_id.data());
    if(!results.empty())
    {
        data.record.switch_number = std::get<0>(results[0]);
        data.record.hangup_type = std::stoi(std::get<1>(results[0]));
        data.record.call_state = std::stoi(std::get<2>(results[0]));
    }
}

void MessageProcess::GetTaskInfo(CallBackData &data, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    std::string sql = "select uuid from outcall_task where id=?";
    auto results = mysqlclient.query<std::tuple<std::string>>(sql,data.record.task_id);
    if(!results.empty())
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

void MessageProcess::GetClueInfo(CallBackData &data, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    std::string sql = "select alias from outcall_clue where id=?";
    auto results = mysqlclient.query<std::tuple<std::string>>(sql, data.clue_id);
    if(!results.empty())
    {
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
}

std::pair<std::string, std::string> MessageProcess::GetSignkey(std::string_view url, std::string keyname, std::string signatureKey)
{
    std::pair<std::string, std::string> pair{"", ""};

    rapidjson::Document doc;
    std::string real_data = HttpRequester::PostUrl2(keyname, signatureKey, url.data(), "", true);
    doc.Parse(real_data.c_str());
    if (doc.IsObject())
    {
        std::string str = "";
        if (doc["info"].IsString())
            str = doc["info"].GetString();

        if (doc["info"].IsString() && str == "Success")
        {
            pair.first = doc["data"]["name"].GetString();
            pair.second = doc["data"]["value"].GetString();
        }
    }

    return pair;
}

std::tuple<std::string,std::string> MessageProcess::GetUrlAndCCgeid(std::string_view cc_number, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    std::string sql = get_sql("select server_ip,https_port,enterprise_id from ai.enterprise_info where id = (select enterprise_uid from calllog where cc_number= ?);",cc_number.data());
    auto results = mysqlclient.query<std::tuple<std::string, std::string, std::string>>(sql);
    LOGGER->info(sql);
    if(!results.empty())
    {
        auto [server_ip,https_port,ccgeid] = results[0];

        std::string url = "https://" + server_ip + ":" + https_port;
        LOGGER->info("GetCallRecordFromCm ccgeid is {}", ccgeid);
        return {url,ccgeid};
    }
    else
        return {"",""};
}

std::string MessageProcess::UpdateCallRecord(std::string_view cc_number, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    auto [url,ccgeid] = GetUrlAndCCgeid(cc_number,mysqlclient);
    if(!ccgeid.empty())
    {
        std::string key_name = "robot_cm_default_secret";
        std::string signatureKey = "robot.cm.default.secret.da123456780e6fy5";
        auto pair = GetSignkey(url+"/server/signkey", key_name, signatureKey);
        if (!pair.first.empty())
        {
            std::string post_param = "ccgeid=" + ccgeid + "&cc_number=" + cc_number.data();
            std::string response = HttpRequester::PostUrl2(pair.first, pair.second, url + "/robot/call_record?" + post_param, post_param, false);
            rapidjson::Document doc;
            doc.Parse(response.c_str());

            if (doc.IsObject() && doc.HasMember("data"))
            {
                rapidjson::StringBuffer str_buf;
                rapidjson::Writer<rapidjson::StringBuffer> writer(str_buf);
                doc["data"].Accept(writer);
                return str_buf.GetString();
            }
        }
    }
    return "";
}

std::tuple<Response,std::string> MessageProcess::CheckCCNumber(std::string_view body)
{
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(body.data(), root))
    {
        auto records = root["records"];
        if (records.isArray() && records.size() > 0)
        {
            auto record = records[0];
            if (!record["cc_number"].isNull() && record["cc_number"].asString() != "")
                return {Response::SUCCESS, record["cc_number"].asString()};
        }
    }
    return {Response::FAIL,""};
}

std::tuple<Response,std::string,std::string,std::string> MessageProcess::CheckFromOC(std::string_view body)
{
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(body.data(), root))
    {
        auto records = root["records"];
        if (records.isArray() && records.size() > 0)
        {
            auto record = records[0];
            if (!record["calllog_id"].isNull() && record["calllog_id"].asString() != "" &&
                !record["cc_number"].isNull() && !record["url"].isNull())
                return {Response::SUCCESS,record["calllog_id"].asString(),record["cc_number"].asString(),record["url"].asString()};
        }
    }
    return {Response::FAIL,"","",""};
}

std::string MessageProcess::GetCallBackUrl(std::string_view eid, ormpp::dbng<ormpp::mysql> &mysqlclient)
{
    auto url = mysqlclient.query<std::tuple<std::string>>("select value from aicall_config where `key` = 'api_callback_domain' and eid = ?",eid.data());
    LOGGER->info("select value from aicall_config where `key` = 'api_callback_domain' and eid = {}", eid);
    if (url.size())
    {
        std::string real_url = std::get<0>(url[0]) + "/callRecordDetail";
        LOGGER->info("url is {}", real_url);
        return real_url;
    }
    else
        return "";
}

std::tuple<std::string,std::string> MessageProcess::GetInfoFromSubsidiary(std::string_view cc_number)
{
    GETMYSQLCLIENT
    std::string sql = get_sql("select a.calllog_id,a.url from aicall_calllog_subsidiary as a,calllog as b where a.calllog_id=b.id and b.cc_number=?;",cc_number.data());
    LOGGER->info(sql);
    auto result = mysqlclient.query<std::tuple<std::string,std::string>>(sql);
    if (result.size())
        return result[0];
    else
        return {"",""};
}

void MessageProcess::UpdateCalllogSubsidiary(ormpp::dbng<ormpp::mysql> &mysqlclient,int status, const std::string &calllog_id)
{
    std::string sql = get_sql("update aicall_calllog_subsidiary set update_status=? where calllog_id=?",status,calllog_id);
    LOGGER->info(sql);
    mysqlclient.execute(sql);
}