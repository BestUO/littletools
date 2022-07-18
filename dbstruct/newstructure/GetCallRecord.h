#ifndef CALL_H
#define CALL_H

#include <iostream>
#include <string>
#include <json/json.h>
#include "../dbstruct/dbstruct.h"

    struct CallInfo {
        std::string record_url;
        std::string confirm_time;//sql confirm_timestamp
        std::string transfer_confirm_time;//sql manual_confirm
        std::string end_time;//end_time
        std::string transfer_end_time;//manual_disconnect
        int duration_time;
        int enterprise_type;// 1 is call center, 0 is cloud switch
        int call_state;
        std::string transfer_number;
        int transfer_duration;
        int transfer_call_state;
        std::string start_time;
        std::string transfer_start_time;//manual_incoming
        int call_type;
        std::string switch_number;
        int stop_reason;
        int customer_fail_reason;
        int manual_type;
        std::string cc_number;
        int call_result;
        int hangup_type;
        int flow_number;
        std::string send_query_msg_timestamp;
        std::string send_invite_timestamp;
        std::string ring_time;
        std::string transfer_manual_cost;
        CallInfo() : record_url(""),confirm_time(""),transfer_confirm_time(""), end_time(""), transfer_end_time(""),duration_time(0), enterprise_type(1), call_state(0),
                     transfer_duration(0), transfer_call_state(0),start_time(""), transfer_start_time(""),call_type(0) ,
                     stop_reason(0), customer_fail_reason(0),manual_type(0),cc_number(""),call_result(0),hangup_type(0),
                     flow_number(-1),send_query_msg_timestamp(""),send_invite_timestamp(""),ring_time(""),transfer_manual_cost(""){}
    };

enum ManualType{
    NoType = 0,
    NoFreeSeats = 1,
    Answered,
    HangUpDuringTransfer,
    ManualNotAnswered,
    ManualRefuse
};

enum HangUpType{
    AiHangUp = 1,
    UserHangUp
};

enum CallResult
{
    HANGUP_WITH_NO_INPUT = 1,
    CALL_SUCCESS,
    UNCONNECTED,
    CALL_FAIL,
    CALL_REJECT,
    NOT_EXIST,
    POWER_OFF,
    OUT_OF_SERVICE,
    NOT_IN_SERVICE,
    BUSY,
    SEAT_ARREARAGE 
};

class CallRecord{
    public:     
         CallInfo GetCallRecord(std::string s,int framework_class);
         std::string CheckInfo(std::string info);

    private: 
         int GetManualType(int stop_reason,int customer_fail_reason);
         int GetHangupType(int stop_reason,int customer_fail_reason);
         int GetCallResult(int stop_reason,int customer_fail_reason);
         
 };

#endif