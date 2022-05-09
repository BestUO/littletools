#include "GetCallRecord.h"
#include <iostream>
#include <json/json.h>
#include <string>
using namespace std;
CallInfo CallRecord::GetCallRecord(std::string s, int framework_class)
{
    CallInfo result;
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(s, root))
    {

        Json::Value data = root["data"];

        auto remove = [](std::string str) -> int
        {
            std::string::size_type pos = str.find("秒");
            int judge = 0;
            for (unsigned int nos = 0; nos != pos; nos++)
            {
                if ((str[nos]) > '0' && str[nos] <= '9')
                {
                    return 2;
                }
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
        if (!data.isObject() || data["enterprise_type"].isNull() || data["record_url"].isNull() ||
            data["records"].isNull())
            return result;
        auto enterprise_type = (data)["enterprise_type"];
        auto record_url = (data)["record_url"];
        auto records = (data)["records"];

        if (record_url.isString()&&!record_url.isNull())
            result.record_url = record_url.asString();

        if (enterprise_type.isInt())
            result.enterprise_type = enterprise_type.asInt();
        int duration = 0;
        // when transfer manual happened, records size will be 2 or more
        if (records.isArray() && records.size() > 0)
        {
            for (unsigned int i = 0; i < records.size(); ++i)
            {
                auto &record = records[i];
                if (!record.isObject() || record["confirm_timestamp"].isNull() ||
                    record["end_time"].isNull() ||
                    record["duration_time"].isNull() ||
                    record["call_type"].isNull() ||
                    (record["call_state"].isNull() && record["stop_reason"].isNull() &&
                     record["customer_fail_reason"].isNull()) ||
                    record["dialing"].isNull())
                    return result;
                auto &call_type = record["call_type"];
                int type = call_type.isString() && !call_type.isNull() ? stoi(call_type.asString()) : 1;

                auto &duration_time = !record["valid_duration"].isNull() == true ? record["valid_duration"] : record["duration_time"];

                if (type == 3 ||
                    type == 2)
                {
                    auto &dialing = record["dialing"];
                    auto &end_time = record["end_time"];
                    auto &call_state = record["call_state"];
                    if (dialing.isString())
                        result.transfer_number = dialing.asString();
                    if (duration_time.isString())
                    {
                        result.transfer_duration = remove_Chinese(duration_time.asString());
                    }

                    if (end_time.isString())
                        result.transfer_end_time = std::string(end_time.asString());
                    if (call_state.isString())
                        result.transfer_call_state = stoi(call_state.asString(), 0);
                    if (!record["valid_duration"].isNull())
                    {
                        result.manual_type = remove(record["valid_duration"].asString());
                        // LOG(INFO) << "manual_type is " << result.manual_type;
                    }
                    //                result.switch_number = record.HasMember("switch_number") ? record["switch_number"].asString() : "";
                }
                else
                {
                    auto &confirm_time = record["confirm_timestamp"];
                    auto &end_time = record["end_time"];
                    auto &call_state = record["call_state"];
                    auto &start_time = record["start_time"];
                    auto &call_type = record["call_type"];
                    if (confirm_time.isString())
                        result.confirm_time = std::string(confirm_time.asString());
                    if (end_time.isString() )
                        result.end_time = std::string(end_time.asString());

                    if (duration_time.isString())
                    {
                        result.duration_time = remove_Chinese(duration_time.asString());
                    }

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
                    string str;
                    int reason_judge = 0;
                    if (!record["customer_fail_reason"].isNull())
                    {
                        auto &customer_fail_reason = record["customer_fail_reason"];
                        string str = customer_fail_reason.asString();
                        if (str != "")
                        {
                            result.call_state = stoi(str_associaction(customer_fail_reason.asString(), 2));
                            result.customer_fail_reason = stoi(customer_fail_reason.asString());
                        }
                        else
                            reason_judge = 1;
                    }
                    if (!record["stop_reason"].isNull() && reason_judge == 1)
                    {
                        auto &stop_reason = record["stop_reason"];
                        int num = stop_reason.asInt();
                        if (num != 0)
                        {
                            result.call_state = stoi(str_associaction(to_string(stop_reason.asInt()), 1));
                            result.stop_reason = stop_reason.asInt();
                        }
                    }
                }
            }
        }
    }
    return result;
}