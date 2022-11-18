#ifndef DATACACHE_H
#define DATACACHE_H
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
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

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
    IdMuster ParseCmId(const std::string &cm_id);
   
};

#endif