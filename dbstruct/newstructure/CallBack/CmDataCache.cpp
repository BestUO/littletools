#include <iostream>
#include "CmDataCache.h"
#include <ctime>
#include "../../../redispool/redisclient.h"
// int stoi_s(const std::string &str)
// {
//     int i = 0;
//     try
//     {
//         i = std::stoi(str);
//     }
//     catch (...)
//     {
//     }
//     return i;
// }
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
            LOGGER->info("PollingQueue  id is {}",now_id);
            IdMuster muster = ParseCmId(now_id);
            if (muster.time != "oc" && muster.time != "web" && muster.time != "now")
            {
                CallBackRules rule;
                CallBackData data;
                std::string cm_data_cache = instance.SearchRules(now_id);
                int calllog_or_cc_number = 1;
                PrepareId(data,rule,calllog_or_cc_number,muster.calllog_id);

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
    std::string list_name = "cm_id_cluster_ocweb";

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
            CallBackRules rule;
            CallBackData data;

            IdMuster muster = ParseCmId(now_id);
            int calllog_or_cc_number = 1;
            PrepareId(data,rule,calllog_or_cc_number,muster.calllog_id);
            if (muster.time == "oc" || muster.time == "web")
            {

                bool class_judge = 0; // whatever just not  0
                std::string data = "";
                if (muster.time == "web")
                {    
                    data = GetCallRecordFromCm(muster.url);
                    class_judge = 1;
                }
                else  
                    class_judge = 3;
                UpdateMessage update_action;
                
                update_action.HandleSQL(data, class_judge, muster.calllog_id);
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
    std::string list_name = "cm_id_cluster_now";

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
                LOGGER->info("begin call back!!!  data is {}",data_cache);
                CallBackAction(data_cache, muster.url);
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
    int pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
    // for (int i = 0; i < cm_id.size(); i++)
    // {
    //     if (cm_id[i] == '-')
    //     {
    //         if (pos1 == 0)
    //         {
    //             pos1 = i;
    //             break;
    //         }
    //         else if (pos2 == 0)
    //         {
    //             pos2 = i;
    //             break
    //         }
    //     }
    // }
    if (cm_id.find('-') != std::string::npos)
    {
        pos1 = cm_id.find('-');
        muster.calllog_id = cm_id.substr(0, pos1);
        muster.time = cm_id.substr(pos1 + 1, (cm_id.size() - pos1 - 1));
        // LOGGER->info("calllog_id is {},type is {}", muster.calllog_id, muster.time);
    }
    else
    {
        LOGGER->info("parse error ,redis data have problem");
    }
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

