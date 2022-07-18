#include <iostream>
#include "CmDataCache.h"
#include <ctime>
#include "../../../redispool/redisclient.h"
void DataCache::PollingQueue()
{
    
    std::vector<std::string> list;
    std::deque<std::string> que; // pair: cm_id , is not _sync?
    // RedisOperate *instance = RedisOperate::getInstance();
    RedisOperate instance;
    // instance->RedisConnect();
    std::string list_name = "cm_id_cluster";

    int sleep_judge=0;
    while(true)
    {
        if(!que.size())
        {

            list = instance.GetListFromRedis(list_name);
            if(list.size())
                PushQueue(que,list);
            else 
            {
                LOGGER->info("PollingQueue  sleep 300s");
                // sleep(300);
                LOGGER->info("PollingQueue  wakeup");
                continue;
            }
        }
        
        std::string now_id = que.front();
        IdMuster muster = ParseCmId(now_id);
        // std::string rule_id = muster.eid+'-'+muster.task_id;
        // std::string rule = instance->SearchRules(rule_id);
        CallBackRules rule;
        CallBackData data;
        std::string cm_data_cache = instance.SearchRules(now_id);
        if(cm_data_cache!="null")
        {
            data = CacheCmJsonSwitch(cm_data_cache);
            GetRulesFromRedis(rule);
            if(CallBackJudge(rule,data))
            {
                //callback
                if(OC_sync_judge(muster.calllog_id)||CheckIdTime(muster))
                {
                    GetOCSyncData(data);
                    instance.DelKey(now_id);
                    instance.LREMForList(list_name,{now_id});
                    std::string caback_data = MergeCacheJson(data,cm_data_cache);
                    que.pop_front();
                }
                else
                { 
                    BackQueue(que,list_name);
                }
            }
        }
 
    }

}

void DataCache::PushQueue(std::deque<std::string> &que, std::vector<std::string> &list)
{
    for (std::vector<std::string>::iterator it = list.begin(); it != list.end(); it++)// reset  queue 
    {
        que.push_front(*it); 
    }
    int num = que.size();
    LOGGER->info("push queue {},size is ", num);
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

IdMuster DataCache::ParseCmId(const std::string &cm_id)
{
    IdMuster muster;
    int id_num=0;
    int pos1=0,pos2=0,pos3;
    for(int i=0;i<cm_id.size();i++)
    {
        if(cm_id[i]=='-')
        {
           if(pos1==0)
            {
                pos1 = i;
            }
            else if(pos2==0)
            {
                pos2 = i;
            }
            else if(pos3==0)
            {
                pos3 = i;
                break;
            }
        }
    }

    muster.eid = cm_id.substr(0,pos1+1);
    muster.task_id = cm_id.substr(pos1+1,(pos2-pos1-1));
    muster.calllog_id = cm_id.substr(pos2+1,pos3-pos2-1);
    muster.time = stoi(cm_id.substr(pos3+1,(cm_id.size()-pos3-2)));
    return muster;
}

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

bool DataCache::CheckIdTime(const IdMuster &muster)
{
    auto now = time(NULL);    
    std::stringstream sstream;
    sstream << now;
    std::string time_ = sstream.str();
    if(stoi(time_)-muster.time>=600)
        return 1;
    else 
        return 0;
}

void DataCache::BackQueue(std::deque<std::string> &que,std::string list_name)
{
    std::vector<std::string> vec(que.begin(),que.end());
    // std::shared_ptr<RedisOperate> instance = std::make_shared<RedisOperate>();
    // instance->RedisConnect();
    RedisOperate instance;
    instance.Rpush(list_name,vec);
    que.clear();
    LOGGER->info("oc not sync,back to redis,and sleep 300s");
    sleep(300);
}


    // while (true)
    // {
    //     now_time = time(NULL);
    //     if (CheckQueue(que, now_time, pre_time))
    //     {
    //         // continue
    //         std::string now_id = que.front().first;
    //         IdMuster muster = ParseCmId(now_id);
    //         // std::string rule_id = muster.eid+'-'+muster.task_id;
    //         // std::string rule = instance->SearchRules(rule_id);
    //         CallBackRules rule;
    //         CallBackData data;
    //         std::string cm_data_cache = instance->SearchRules(now_id);
    //         if(cm_data_cache!="null")
    //         {
    //             data = CacheCmJsonSwitch(cm_data_cache);
    //             GetRulesFromRedis(rule);
    //             if(CallBackJudge(rule,data))
    //             {
    //                 //callback
    //                 if(OC_sync_judge(muster.calllog_id))
    //                 {
    //                     GetOCSyncData(data);
    //                     std::string caback_data = MergeCacheJson(data,cm_data_cache);

    //                 }else
    //                 { 
    //                     MoveQueue(que);
    //                 }
    //             }
    //         }

    //     }
    //     else
    //     {
    //         set = instance->GetSetFromRedis(set_name);
    //         PushQueue(que,set);
    //         now_time = time(NULL);
    //         pre_time = time(NULL);
    //     }
    // }