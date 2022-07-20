#include <iostream>
#include "CmDataCache.h"
#include <ctime>
#include "../../../redispool/redisclient.h"
void DataCache::PollingQueue()
{

    std::vector<std::string> list;
    RedisOperate instance;
    std::string list_name = "cm_id_cluster";

    int sleep_judge = 0;
    while (true)
    {
        if (list.empty())
        {
            list = instance.GetListFromRedis(list_name);
            if (list.empty())
            {
                LOGGER->info("no cache data,PollingQueue  sleep 300s");
                sleep(300);
                LOGGER->info("PollingQueue  wakeup");
                continue;
            }
        }
        for (auto &now_id : list)
        {
            IdMuster muster = ParseCmId(now_id);
            if (muster.time != "oc"&&muster.time != "web" && muster.time != "now")
            {
                CallBackRules rule;
                CallBackData data;
                data.task_id = muster.task_id;
                data.eid = muster.eid;
                data.calllog_id = muster.calllog_id;
                std::string cm_data_cache = instance.SearchRules(now_id);
                if (cm_data_cache != "null")
                {
                    data = CacheCmJsonSwitch(cm_data_cache);
                    GetRulesFromRedis(rule);
                    if (OC_sync_judge(muster.calllog_id) || CheckTimeOut(muster))
                    {
                        GetOCSyncData(data);
                        if (CallBackJudge(rule, data))
                        {
                            // callback
                            instance.DelKey(now_id);
                            instance.LREMForList(list_name, {now_id});
                            std::string caback_data = MergeCacheJson(data, cm_data_cache);
                        }
                    }
                    else
                    {
                        LOGGER->info("oc not sync,back to redis,and sleep 300s");
                        sleep(300);
                    }
                }
            }
        }
        list.clear();
    }
}

void DataCache::OcWebPollingQueue()
{
    std::vector<std::string> list;
    RedisOperate instance;
    std::string list_name = "cm_id_cluster";

    while (true)
    {
        if (list.empty())
        {
            list = instance.GetListFromRedis(list_name);
            if (list.empty())
            {
                LOGGER->info("no cache data,OcWebPollingQueue  sleep 300s");
                sleep(300);
                LOGGER->info("OcWebPollingQueue  wakeup");
                continue;
            }
        }
        for (auto &now_id : list)
        {
            IdMuster muster = ParseCmId(now_id);
            
            if (muster.time == "oc"||muster.time == "web")
            {
                std::string data = "";
                if (muster.time == "web")
                data = GetCallRecordFromCm(muster.url);
                UpdateMessage update_action;
                bool class_judge = 5;
                update_action.HandleSQL(data,class_judge,muster.calllog_id);
                instance.DelKey(now_id);
                instance.LREMForList(list_name, {now_id});
            }
        }
        list.clear();
    }
}

void DataCache::CallBackActionQueue()
{
    std::vector<std::string> list;
    RedisOperate instance;
    std::string list_name = "cm_id_cluster";
    
    while (true)
    {
        if (list.empty())
        {
            list = instance.GetListFromRedis(list_name);
            if (list.empty())
            {
                LOGGER->info("no cache data,CallBackActionQueue  sleep 300s");
                sleep(300);
                LOGGER->info("CallBackActionQueue  wakeup");
                continue;
            }
        }
        for (auto &now_id : list)
        {
            IdMuster muster = ParseCmId(now_id);
            
            if (muster.time == "now")
            {
                std::string data_cache = instance.SearchRules(now_id);
                CallBackAction(data_cache,muster.url);
                instance.DelKey(now_id);
                instance.LREMForList(list_name, {now_id});
            }
        }
        list.clear();
    }
}

IdMuster DataCache::ParseCmId(const std::string &cm_id)
{
    IdMuster muster;
    int id_num = 0;
    int pos1 = 0, pos2 = 0, pos3=0,pos4=0;
    for (int i = 0; i < cm_id.size(); i++)
    {
        if (cm_id[i] == '-')
        {
            if (pos1 == 0)
            {
                pos1 = i;
            }
            else if (pos2 == 0)
            {
                pos2 = i;
            }
            else if (pos3 == 0)
            {
                pos3 = i;
            }
            else if (pos4 == 0)
            {
                pos4 = i;
                break;
            }
        }
    }

    muster.eid = cm_id.substr(0, pos1 + 1);
    muster.task_id = cm_id.substr(pos1 + 1, (pos2 - pos1 - 1));
    muster.calllog_id = cm_id.substr(pos2 + 1, pos3 - pos2 - 1);
    muster.time = cm_id.substr(pos3 + 1, pos4 - pos3 - 1);
    muster.url = cm_id.substr(pos3 + 1, (cm_id.size() - pos4 - 2));
    return muster;
}

bool DataCache::CheckTimeOut(const IdMuster &muster)
{
    auto now = time(NULL);
    std::stringstream sstream;
    sstream << now;
    std::string time_ = sstream.str();
    if (stoi(time_) - stoi(muster.time) >= 600)
    {
        LOGGER->info("Callback_data is timeout,force callback");
        return 1;
    }
    else
        return 0;
}

// void DataCache::MoveQueue(std::deque<std::string> &que)
// {
//     if (que.size())
//     {
//         auto bottom = que.front();
//         que.pop_front();
//         que.push_back(bottom);
//     }
// }

// bool DataCache::CheckQueue(const std::deque<std::string> &que, const std::time_t &time_pre, const std::time_t &time_now)
// {
//     std::stringstream sstream;
//     sstream << time_pre;
//     std::string time_pre_str = sstream.str();

//     sstream << time_now;
//     std::string time_now_str = sstream.str();
//     int duration = stoi(time_now_str) - stoi(time_pre_str);

//     return ((que.size() == 0) || (duration >= 10) || (que.front().second == 1)) ? 0 : 1;
// }

//  void DataCache::ManageIdTime( std::vector<std::string> &cm_id)
// {
//     time_t now = time(NULL);
//     for(int i=0;i<cm_id.size();i++)
//     {
//         int num_ =0;
//         std::string rep;
//         rep = GetCmDataId(cm_id[i]);
//         rep+='-';
//         std::stringstream sstream;
//         sstream << now;
//         std::string time_ = sstream.str();
//         rep+=time_;
//     }
// }

// std::string DataCache::GetCmDataId(const std::string &id)
// {
//     std::string rep;
//     int num_ =0;
//     for(int j=0;j<id.size();j++)
//     {

//         if(id[j]=='-')
//         {
//             num_++;
//         }
//         if(num_==3)
//         {
//             return rep;
//         }
//         rep+=id[j];
//     }

// }