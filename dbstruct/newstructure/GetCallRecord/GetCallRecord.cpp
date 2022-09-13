#include "GetCallRecord.h"
#include <iostream>
#include <json/json.h>
#include <string>
using namespace std;
CallInfo CallRecord::GetCallRecord(std::string &s, int &framework_class)
{
    CallInfo result;
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(s, root))
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

                if (result.flow_number != 0 && result.flow_number >= now_flow) // transfer_manual
                {
                    now_flow = result.flow_number;

                    auto &dialing = record["dialing"];
                    auto &transfer_confirm_time = record["confirm_timestamp"];
                    auto &send_query_msg_timestamp = record["query_msg_time"];
                    auto &send_invite_timestamp = record["invite_time"];
                    auto &transfer_manual_cost = record["seat_ring_duration"];
                    auto &conversation_time = record["conversation_time"];

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
                        result.transfer_start_time = start_time.asString();
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
                    result.hangup_type = GetHangupType(result.stop_reason, result.customer_fail_reason);

                    auto &confirm_time = record["confirm_timestamp"];
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

std::string CallRecord::CheckInfo(std::string &info)
{
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(info, root))
    {

        auto records = root["records"];
        if (records.isArray() && records.size() > 0)
        {
            auto record = records[0];
            if (!record["cc_number"].isNull() && record["cc_number"].asString() != "")
            {

                if (!record["switch_number"].isNull() || record["cc_number"].asString() == "")
                    return "902";
                else
                    return "903"; // switch_number为空
            }
            else
                return "901"; // cc_number空或不存在
        }
    }
    return "900"; // style error
}

std::string CallRecord::CheckUnSync(const std::string &info)
{
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(info, root))
    {

        auto records = root["data"];
        if (!root["eid"].isNull())
        {
            return "902";
        }
    }
    return "900"; // style error
}

std::string CallRecord::CheckWebOcInfo(const std::string &info)
{
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(info, root))
    {

        auto records = root["data"];
        if (records.isArray() && records.size() > 0 && !root["type"].isNull())
        {
            return "902";
        }
    }
    return "900"; // style error
}

std::vector<std::string> CallRecord::ParseUnSync(const std::string &info)
{
    Json::Reader reader;
    Json::Value root;
    std::vector<std::string> vec;
    if (reader.parse(info, root))
    {
        vec.emplace_back(std::to_string(root["eid"].asInt()));
        vec.emplace_back(std::to_string(root["min_time"].asInt()));
        vec.emplace_back(std::to_string(root["max_time"].asInt()));
    }
    return vec;
}

int CallRecord::GetManualType(int &stop_reason, int &customer_fail_reason, int &conversation_type)
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

int CallRecord::GetHangupType(int &stop_reason, int &customer_fail_reason)
{
    if (stop_reason == 25)
        return HangUpType::UserHangUp;
    else
        return HangUpType::AiHangUp;
}
int CallRecord::GetCallResult(int &stop_reason, int &customer_fail_reason)
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
