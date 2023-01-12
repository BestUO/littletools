#pragma once
#include <string>

struct Record
{
    int task_id=0;
    std::string script_name="";
    std::string callee_phone="";
    std::string cc_number="";
    std::string calllog_txt="";
    int call_result=0;
    int call_time=0;
    int duration=0;
    int intention_type=0;
    int manual_status=0;
    int call_count=0;
    std::string record_url="";
    int answer_time=0; // sql confirm_timestamp
    int hangup_time=0; // end_time
    std::string match_global_keyword="";
    int transfer_duration=0;
    std::string transfer_number="";
    std::string buttons="";
    int calllog_id=0;
    std::string caller_phone="";
    std::string switch_number="";
    int hangup_type=0;
    std::string label="";
    std::string clue_no="";
    int call_state = 0;
};

struct CallBackData
{
    std::string uuid="";
    Record record;
    std::string clue_id="";
};

struct CallInfo 
{
    std::string record_url="";
    std::string confirm_time="";//sql confirm_timestamp
    std::string transfer_confirm_time="";//sql manual_confirm
    std::string end_time="";//end_time
    std::string transfer_end_time="";//manual_disconnect
    int duration_time=0;
    int enterprise_type=1;// 1 is call center, 0 is cloud switch
    int call_state=0;
    std::string transfer_number="";
    int transfer_duration=0;
    int transfer_call_state=0;
    std::string start_time="";
    std::string transfer_start_time="";//manual_incoming
    int call_type=0;
    std::string switch_number="";
    int stop_reason=0;
    int customer_fail_reason=0;
    int manual_type=0;
    std::string cc_number="";
    int call_result=0;
    int hangup_type=0;
    int flow_number=-1;
    std::string send_query_msg_timestamp="";
    std::string send_invite_timestamp="";
    std::string ring_time="";
    std::string transfer_manual_cost="";
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

enum Response{SUCCESS,FAIL};