#include "GetCallRecord.h"
#include "../dbstruct/dbstruct.h"
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
        if (!data.isObject() || data["enterprise_type"].isNull() || data["record_url"].isNull() ||
            data["records"].isNull())
            return result;
        auto enterprise_type = (data)["enterprise_type"];
        auto record_url = (data)["record_url"];
        auto records = (data)["records"];

        if (record_url.isString() && !record_url.isNull())
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
                auto &end_time = record["end_time"];
                auto &call_state = record["call_state"];
                auto &start_time = record["start_time"];

                if (record["cc_number"].isString())
                    result.cc_number = record["cc_number"].asString();

                result.flow_number = record["flow_number"].isNull()?-1 :record["flow_number"].asInt();
                if(record["customer_fail_reason"].asString()!="0")
                    result.customer_fail_reason = stoi(record["customer_fail_reason"].asString());
                result.stop_reason = record["stop_reason"].asInt();

                if(result.flow_number==1)
                {

                    result.manual_type = GetManualType(result.stop_reason,result.customer_fail_reason);
                } else if(result.flow_number==0){
                    result.call_result = GetCallResult(result.stop_reason,result.customer_fail_reason);
                    result.hangup_type = GetHangupType(result.stop_reason,result.customer_fail_reason);
                }



                if (type == 3 || type == 2)//transfer_manual
                {
                    auto &dialing = record["dialing"];
                    auto &transfer_confirm_time = record["confirm_timestamp"];
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
                   
                }
                else//ai_
                {
                    auto &confirm_time = record["confirm_timestamp"];
                    auto &call_type = record["call_type"];
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
                    if (result.customer_fail_reason!=0)
                    {
                        result.call_state = stoi(str_associaction(std::to_string(result.customer_fail_reason), 2));
                    }
                    else if (result.stop_reason!=0)
                    {
                        result.call_state = stoi(str_associaction(to_string(result.stop_reason), 1));
                    }
                
                }
            }
        }
    }
    return result;
}




std::string CallRecord::CheckInfo(std::string info)
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
                    return "903";//switch_number为空
            }
            else
                return "901"; // cc_number空或不存在
        }
    }
    return "900"; // style error
}

 int CallRecord::GetManualType(int stop_reason,int customer_fail_reason)
 {
        if(customer_fail_reason==9)
           
            return 5;

        switch (stop_reason)
        {
        case 9:
            return 1;
        case 43:
            return 3;
        case 13:
        case 35:
        case 7:
            return 4;
        case 25:
        case 26:
        case 31:
        case 29:
        case 30:
            return 2;
        default:
            return 0;
        }
 }

int CallRecord::GetHangupType(int stop_reason,int customer_fail_reason)
{
    if(stop_reason==25)
        return 2;
    else 
        return 1;
}
int CallRecord::GetCallResult(int stop_reason,int customer_fail_reason)
{

        switch (customer_fail_reason)
    {
        case 9:
            return 5;
        case 8:
            return 6;
        case 2:
            return 7;
        case 11:
            return 8;
        case 13:
            return 9;
        case 30:
        case 31:
            return 11;
        default:
          return 0;
    }
    switch(stop_reason)
    {
        case 25:
        case 26:
        case 31:
            return 2;
        case 19:
            return 3;
        case 18:
        case 42:
            return 4;
        default :
            break;
    }

}