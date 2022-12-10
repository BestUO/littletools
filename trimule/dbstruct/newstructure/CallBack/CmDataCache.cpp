#include <iostream>
#include "CmDataCache.h"
#include <ctime>
#include "../../../redispool/redisclient.h"

void DataCache::PollingQueue()
{
    std::vector<std::string> list;
    RedisOperate instance;
    std::string list_name = "cm_id_cluster";
    ormpp::dbng<ormpp::mysql> mysqlclient;
    sqlconnect conne = SettingParser::GetSettinghParser("conf/trimule_config.json");
    std::string port = to_string(conne.db_port);
    try
    {
        if(!mysqlclient.connect(conne.host.c_str(), conne.user.c_str(), conne.password.c_str(), conne.db.c_str(), conne.db_timeout, conne.db_port))
        throw 1;
    }
    catch (int i)
    {
        if(i)
            LOGGER->info("mysql maybe error ,please take a check ");
        return;
    }
    
    int sleep_judge = 0;
    int time = SettingParser::GetSleepTime("conf/trimule_config.json");
    LOGGER->info("PollingQueue sleep time is {} second", time);
    while (true)
    {
        if (list.empty())
        {
            list = instance.GetListFromRedis(list_name);
            if (list.empty())
            {
                sleep(time);
                continue;
            }
        }
        for (auto &key : list)
        {
            LOGGER->info("from {} key is {}", list_name, key);
            IdMuster muster = ParseCmId(key);
            if (muster.time != "oc" && muster.time != "web" && muster.time != "now")
            {
                std::string cm_data_cache = instance.GetValue(key);
                
                if (!cm_data_cache.empty())
                {
                    CallBackData data = CacheCmJsonSwitch(cm_data_cache);
                    CallBackRules rule = GetRulesFromRedis(data.eid, data.task_id);
                    std::string wherecondition = R"(id = ')" + muster.calllog_id + R"(')";
                    PrepareId(data, wherecondition, mysqlclient);

                    if (OC_sync_judge(muster.calllog_id, mysqlclient) || CheckTimeOut(muster))
                    {
                        instance.DelKey(key);
                        instance.LREMForList(list_name, {key});
                        GetOCSyncData(data, mysqlclient);
                        if (CallBackJudge(rule, data))
                        {
                            // callback
                            std::string caback_data = MergeCacheJson(data, cm_data_cache);
                            CallBackManage::CacheCmData(data, caback_data, 2, mysqlclient);
                        }
                    }
                    else
                        sleep(time);
                }
                else
                {
                    instance.LREMForList(list_name, {key});
                    LOGGER->info("null cm_data_cache ,donot callback");
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
    ormpp::dbng<ormpp::mysql> mysqlclient;
    sqlconnect conne = SettingParser::GetSettinghParser("conf/trimule_config.json");

    try
    {
        if(!mysqlclient.connect(conne.host.c_str(), conne.user.c_str(), conne.password.c_str(), conne.db.c_str(), conne.db_timeout, conne.db_port))
        throw 1;
    }
    catch (int  i)
    {
        if(i)
        {
            LOGGER->info("mysql maybe error ,please take a check ");
            cout<<"mysql error"<<endl;
        }
        return;
    }

    int time = SettingParser::GetSleepTime("conf/trimule_config.json");

    LOGGER->info("OcWebPollingQueue  sleep time is {} second", time);
    while (true)
    {
        if (list.empty())
        {
            list = instance.GetListFromRedis(list_name);
            if (list.empty())
            {
                sleep(time);
                continue;
            }
        }
        for (auto &key : list)
        {
            LOGGER->info("from {} key is {}", list_name, key);
            IdMuster muster = ParseCmId(key);

            std::string wherecondition = "";
            if (muster.time == "cm_whole")
                wherecondition = R"(cc_number = ')" + muster.calllog_id + R"(')";
            else
                wherecondition = R"(id = ')" + muster.calllog_id + R"(')";

            if (muster.calllog_id != "")
            {
                CallBackData data;
                PrepareId(data, wherecondition, mysqlclient); // muster.calllog_id  maybe  cc_number
                if (!OC_sync_judge(data.calllog_id, mysqlclient) && !CheckTimeOut(muster))
                {
                    LOGGER->info("calllog {} not sync ", data.calllog_id);
                    sleep(time);
                    continue;
                }

                std::string message_from_cm = "";
                if (muster.time == "web")
                    message_from_cm = GetCallRecordFromCm(data, mysqlclient);
                else if (muster.time == "oc")
                {}
                else if (muster.time == "cm_whole")
                    message_from_cm = instance.GetValue(key);

                if (data.calllog_id != "" && !message_from_cm.empty())
                    UpdateMessage::HandleSQL(message_from_cm, mysqlclient, wherecondition, data.calllog_id);
            }
            else
                LOGGER->info("cc_number or calllog_id is null ,key is {} ,delete it",key);
            instance.DelKey(key);
            instance.LREMForList(list_name, {key});
        }
        list.clear();
    }
}

void DataCache::CallBackActionQueue()
{
    std::vector<std::string> list;
    RedisOperate instance;
    std::string list_name = "cm_id_cluster_now";
    int time = SettingParser::GetSleepTime("conf/trimule_config.json");
    LOGGER->info("CallBackActionQueue  sleep time is {} second", time);
    while (true)
    {
        if (list.empty())
        {
            list = instance.GetListFromRedis(list_name);
            if (list.empty())
            {
                sleep(time);
                continue;
            }
        }
        for (auto &key : list)
        {
            LOGGER->info("from {} key is {}", list_name, key);
            IdMuster muster = ParseCmId(key);
            if (muster.time == "now")
            {
                std::string data_cache = instance.GetValue(key);
                LOGGER->info("begin call back!!!  data is {}", data_cache);
                if(!data_cache.empty())
                    CallBackAction(data_cache, muster.url);
            }
            instance.DelKey(key);
            instance.LREMForList(list_name, {key});
        }
        list.clear();
    }
}

IdMuster DataCache::ParseCmId(std::string_view key)
{
    IdMuster muster;
    int pos1 = 0, pos2 = 0;

    int delimcount = count(key.begin(), key.end(), '-');

    if (delimcount == 1)
    {

        pos1 = key.rfind("-");
        muster.calllog_id = key.substr(0, pos1);
        muster.time = key.substr(pos1 + 1, (key.size() - pos1 - 1));

    }
    else
    {
        pos1 = key.rfind("-");
        int pos2 = key.substr(0,pos1).rfind("-");
        muster.calllog_id = key.substr(0, pos2);
        muster.url = key.substr(pos2 + 1, pos1-pos2-1);
        muster.time = key.substr(pos1 + 1, pos2 - pos1 - 1);
    }

    return muster;
}

int stoi_l(const std::string &str)
{
    int i = 0;
    try
    {
        i = std::stoi(str);
    }
    catch (...)
    {
    }
    return i;
}


bool DataCache::CheckTimeOut(const IdMuster &muster)
{
    auto now = time(NULL);
    std::stringstream sstream;
    sstream << now;
    std::string time_ = sstream.str();
    if (stoi(time_) - stoi_l(muster.time) >= 600)
    {
        LOGGER->info("Callback_data is timeout,force callback");
        return 1;
    }
    else
        return 0;
}

void DataCache::CheckUnUpdateId(const std::string &eid, const std::string &mini_time, const std::string &max_time)
{
    ormpp::dbng<ormpp::mysql> mysqlclient;
    sqlconnect conne = SettingParser::GetSettinghParser("conf/trimule_config.json");
    try
    {
        if(!mysqlclient.connect(conne.host.c_str(), conne.user.c_str(), conne.password.c_str(), conne.db.c_str(), conne.db_timeout, conne.db_port))
        throw 1;
    }
    catch (int  i)
    {
        if(i)
        {
            LOGGER->info("mysql maybe error ,please take a check ");
            cout<<"mysql error"<<endl;
        }
        return;
    }
    // mysqlclient.connect(conne.host.c_str(), conne.user.c_str(), conne.password.c_str(), conne.db.c_str(), conne.db_timeout, conne.db_port);

    auto res = mysqlclient.query<std::tuple<std::string>>("SELECT id from calllog WHERE enterprise_uid = " + eid + " and create_time >= " + mini_time + " and create_time <= " + max_time + " and id in (select calllog_id from aicall_calllog_subsidiary where update_status = 0) ");

    LOGGER->info("code is SELECT id from calllog WHERE enterprise_uid =  {}  and create_time >= {} and create_time <= {} and id in (select calllog_id from aicall_calllog_subsidiary where update_status = 0) ", eid, mini_time, max_time);

    LOGGER->info("Trimule may has core,so we has found {} calllog has not update now.", res.size());

    for (int i = 0; i < res.size(); i++)
    {
        CallBackData data;
        data.calllog_id = std::get<0>(res[i]);
        LOGGER->info("calllog is {}", data.calllog_id);
        CacheCmData(data, "", 1, mysqlclient);
    }
}