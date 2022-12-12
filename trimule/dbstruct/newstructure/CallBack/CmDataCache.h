#pragma once
#include "CallBack.h"
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <chrono>
#include <sstream>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include "../UpdateCalllog/UpdateCalllog.h"
#include "HttpRequester.h"
#include "global.hpp"

struct IdMuster{
    std::string calllog_id;
    std::string eid;
    std::string task_id;
    std::string time;
    std::string url;

};

class DataCache:public CallBackManage{

public: 

    void PollingQueue();
    void OcWebPollingQueue();
    void CallBackActionQueue();
    void CheckUnUpdateId(const std::string &eid,const std::string &start_time,const std::string &end_time);
    bool CheckTimeOut(const IdMuster &muster);
    std::string GetCmDataId(const std::string &id);
    IdMuster ParseCmId(std::string_view key);
   
};

class MessageProcess
{
public:
    static std::string UpdateAllInfo(std::string_view message, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static std::string GetCallBackString(std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static std::string GetCallRecordFromCm(std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient);
private:
    static CallInfo GetCallRecord(std::string_view real_data, int framework_class);
    static int GetManualType(int &stop_reason, int &customer_fail_reason,int &conversation_type);
    static int GetHangupType(int &stop_reason,int &customer_fail_reason);
    static int GetCallResult(int &stop_reason,int &customer_fail_reason);
    static auto GetIdsWithCCNumber(ormpp::dbng<ormpp::mysql> &mysqlclient,const std::string &cc_number);

    static void GetLabelInfo(CallBackDataNew &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void GetCallLogInfo(CallBackDataNew &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void GetCallLogExtensionInfo(CallBackDataNew &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void GetClueInfo(CallBackDataNew &data, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void GetTaskInfo(CallBackDataNew &data, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static std::string GenerateCallBackString(CallBackDataNew &data);
    static std::string CollectInfoXML2JSON(const std::string &xml);
    static std::vector<std::string> Split(const std::string& str, const std::string& delims);
};