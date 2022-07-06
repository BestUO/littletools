#ifndef DATACACHE_H
#define DATACACHE_H
#include "CallBack.h"
#include "../../dbstruct/dbstruct.h"
#include <unordered_set>
#include <algorithm>
#include <deque>
#include <chrono>
#include <sstream>

struct IdMuster{
    std::string calllog_id;
    std::string eid;
    std::string task_id;
};

class DataCache:public CallBackManage{

public: 

    void PollingQueue();
    void PushQueue(std::deque<std::pair<std::string,int>> &que,std::unordered_set<std::string> &set);
    void MoveQueue(std::deque<std::pair<std::string,int>> &que);
    bool CheckQueue(std::deque<std::pair<std::string,int>> &que,std::time_t &time_pre,std::time_t &time_now);
    IdMuster ParseCmId(const std::string &cm_id);
};

#endif