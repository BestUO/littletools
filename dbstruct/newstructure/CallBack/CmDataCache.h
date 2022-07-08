#ifndef DATACACHE_H
#define DATACACHE_H
#include "CallBack.h"
#include "../../dbstruct/dbstruct.h"
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <chrono>
#include <sstream>

struct IdMuster{
    std::string calllog_id;
    std::string eid;
    std::string task_id;
    int time;
};

class DataCache:public CallBackManage{

public: 

    void PollingQueue();
    void PushQueue(std::deque<std::string> &que,std::vector<std::string> &set);
    // void MoveQueue(std::deque<std::string> &que);
    void BackQueue(std::deque<std::string> &que,std::string list_name);
    // bool DataCache::CheckQueue(const std::deque<std::string> &que, const std::time_t &time_pre, const std::time_t &time_now);
    void ManageIdTime( std::vector<std::string> &cm_id);
    bool CheckIdTime(const IdMuster &muster);
    std::string GetCmDataId(const std::string &id);
    IdMuster ParseCmId(const std::string &cm_id);
};

#endif