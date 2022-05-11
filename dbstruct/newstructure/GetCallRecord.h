#ifndef CALL_H
#define CALL_H

#include <iostream>
#include <string>
#include <json/json.h>
#include "../dbstruct/dbstruct.h"


typedef struct CallInfo {
        std::string record_url;
        std::string confirm_time;
        std::string end_time;
        int duration_time;
        int enterprise_type;// 1 is call center, 0 is cloud switch
        int call_state;
        std::string transfer_number;
        int transfer_duration;
        std::string transfer_end_time;
        int transfer_call_state;
        std::string start_time;
        int call_type;
        std::string switch_number;
        int stop_reason;
        int customer_fail_reason;
        int manual_type;
        std::string cc_number;
        CallInfo() : confirm_time(""), end_time(""), duration_time(0), enterprise_type(1), call_state(0),
                     transfer_duration(-1), transfer_call_state(0), call_type(-1) ,
                     stop_reason(0), customer_fail_reason(0),manual_type(0),cc_number(""){}
    };
    
class CallRecord{
    public:
      
 CallInfo GetCallRecord(std::string s,int framework_class);
 
 };

 #endif