#pragma once
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "messageprocessstruct.hpp"

#include <string_view>
#include <vector>

class MessageProcess
{
public:
    static std::tuple<Response,std::string> CheckCCNumber(std::string_view body);
    static std::tuple<Response,std::string,std::string> CheckEidCalllogId(std::string_view body);
    static std::tuple<std::string,std::string,std::string> UpdateAllInfo(std::string_view message, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static std::string GetCallBackString(std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static std::string UpdateCallRecord(std::string_view cc_number, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static std::string GetCallBackUrl(std::string_view eid, ormpp::dbng<ormpp::mysql> &mysqlclient);
private:
    static CallInfo GetCallRecord(std::string_view real_data, int framework_class);
    static int GetManualType(int &stop_reason, int &customer_fail_reason,int &conversation_type);
    static int GetHangupType(int &stop_reason,int &customer_fail_reason);
    static int GetCallResult(int &stop_reason,int &customer_fail_reason);
    static auto GetIdsWithCCNumber(ormpp::dbng<ormpp::mysql> &mysqlclient,const std::string &cc_number);

    static void GetLabelInfo(CallBackData &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void GetCallLogInfo(CallBackData &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void GetCallLogExtensionInfo(CallBackData &data, std::string_view calllog_id, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void GetClueInfo(CallBackData &data, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static void GetTaskInfo(CallBackData &data, ormpp::dbng<ormpp::mysql> &mysqlclient);
    static std::string GenerateCallBackString(CallBackData &data);
    static std::string CollectInfoXML2JSON(const std::string &xml);
    static std::vector<std::string> Split(const std::string& str, const std::string& delims);
    static std::pair<std::string, std::string> GetSignkey(std::string_view url, std::string keyname, std::string signatureKey);
    static std::tuple<std::string,std::string> GetUrlAndCCgeid(std::string_view cc_number, ormpp::dbng<ormpp::mysql> &mysqlclient);
};