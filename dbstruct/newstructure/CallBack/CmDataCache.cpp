#include <iostream>
#include "CmDataCache.h"
#include <ctime>
void DataCache::PollingQueue()
{
    std::unordered_set<std::string> set;
    RedisOperate *instance = RedisOperate::getInstance();
    std::string set_name = "cm_id_cluster";
    set = instance->GetSetFromRedis(set_name);
    std::deque<std::pair<std::string, int>> que; // pair: cm_id , is not _sync?

    time_t now_time, pre_time;
    now_time = time(NULL);
    pre_time = time(NULL);
    while (true)
    {
        now_time = time(NULL);
        if (CheckQueue(que, now_time, pre_time))
        {
            // continue
            
            std::string now_id = que.front().first;
            IdMuster muster = ParseCmId(now_id);
            
        }
        else
        {
            set = instance->GetSetFromRedis(set_name);
            PushQueue(que,set);
            now_time = time(NULL);
            pre_time = time(NULL);


        }
    }
}

void DataCache::PushQueue(std::deque<std::pair<std::string, int>> &que, std::unordered_set<std::string> &sets)
{

    int num = que.size();
    LOGGER->info("push queue {},size is ", num);

    int i = 0;
    for (std::unordered_set<std::string>::iterator it = sets.end(); it != sets.end(); it++, i++)
    {
        if (i >= num)
            que.push_front({*it, 0}); // reset  pair judge
        else
            que[i].second = 0;
    }
}

void DataCache::MoveQueue(std::deque<std::pair<std::string, int>> &que)
{
    if (que.size())
    {
        auto bottom = que.front();
        que.pop_front();
        que.push_back(bottom);
    }
}

bool CheckQueue(const std::deque<std::pair<std::string, int>> &que, const std::time_t &time_pre, const std::time_t &time_now)
{
    std::stringstream sstream;
    sstream << time_pre;
    std::string time_pre_str = sstream.str();

    sstream << time_now;
    std::string time_now_str = sstream.str();
    int duration = stoi(time_now_str) - stoi(time_pre_str);

    return ((que.size() == 0) || (duration >= 10) || (que.front().second == 1)) ? 0 : 1;
}

IdMuster ParseCmId(const std::string &cm_id)
{
    IdMuster muster;
    int id_num=0;
    int pos1=0,pos2=0;
    for(int i=0;i<cm_id.size();i++)
    {
        if(cm_id[i]=='-')
        {
           if(pos1==0)
                pos1 = i;
            else {
                pos2 = i;
                break;
            }
        }
    }

    muster.eid = cm_id.substr(0,pos1+1);
    muster.task_id = cm_id.substr(pos1+1,(pos2-pos1-1));
    muster.calllog_id = cm_id.substr(pos2+1,(cm_id.size()-pos2-2));
    
    return muster;
}