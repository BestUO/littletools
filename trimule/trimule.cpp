#include "cinatra/cinatra.hpp"
#include "queue/ringqueue.hpp"
#include "tools/threadpool.hpp"
#include "tools/threadpool2.hpp"
#include "tools/jsonwrap.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include "settingParser/settingParser.h"
#include "dbstruct/newstructure/UpdateCalllog/UpdateCalllog.h"
#include "dbstruct/newstructure/CallBack/CallBack.h"
#include "dbstruct/newstructure/CallBack/CmDataCache.h"
#include <vector>
#include <string>
#include <thread>
#include <future>

#include <iostream>
#include <iostream>
#include <functional>
#include <thread>
#include <condition_variable>
#include <future>
#include <atomic>
#include <vector>
#include <queue>


#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

void initspdlog()
{
    spdlog::flush_every(std::chrono::seconds(5));
    auto file_logger = spdlog::rotating_logger_mt<spdlog::async_factory>(SPDLOGGERNAME, SPDLOG_FILENAME, 1024 * 1024 * 200, 5);
    LOGGER->set_level(spdlog::level::info); // Set global log level to info
    LOGGER->set_pattern("[%Y-%m-%d %H:%M:%S.%e %^%L%$ %t] %v");
}

template <class T>
class WorkerForHttp : public Worker<T>
{
public:
    WorkerForHttp(std::shared_ptr<T> queue) : Worker<T>(queue){};

protected:
    virtual void WorkerRun(bool original) final
    {
        while (!Worker<T>::_stop)
        {
            auto e = Worker<T>::_queue->GetObjBulk();
            if (e)
            {
                while (!e->empty())
                {
                    DealElement(std::move(e->front()));
                    e->pop();
                }
            }
            else
            {
                if (!original)
                    break;
                Worker<T>::_queue->WaitComingObj();
            }
        }
    }

    virtual void DealElement(std::string &&s) final
    {
        if (s.size() > 1)
        {
            std::string real_data = s.substr(1, s.size() - 1);

            if (s[0] == '0') // cm ctive
            {
                ormpp::dbng<ormpp::mysql> mysqlclient;
                CallBackData callog;
                CallBackManage cache_action;
                std::string str = real_data;
                int class_judge = 4;
                CallRecord record;
                int a=2;
                CallInfo data = record.GetCallRecord(real_data, a);
        
                callog.cc_number = data.cc_number;
                cache_action.CacheCmData(callog, str, class_judge,mysqlclient);
            }
            else // oc  web
            {
                CallBackManage cache_action;
                cache_action.MakeQueueCache(real_data);
            }
        }
    }
};

int CheckUnUpdateId(const std::vector<std::string> &vec)
{
    DataCache cache;
    cache.CheckUnUpdateId(vec[0],vec[1],vec[2]);
    return 0;
}

template <class T>
void SetApiCallBackHandler(cinatra::http_server &server, T threadpool)
{
    server.set_http_handler<cinatra::GET, cinatra::POST>("/", [threadpool = threadpool](cinatra::request &req, cinatra::response &res)
    {
        LOGGER->info("message is {}",std::string(req.body()));
        CallRecord check;
        std::string check_info = std::string(req.body());
        std::string check_res = check.CheckInfo(check_info);

        std::string que_str ='0'+check_info;
        int type = 0;
        if(check_res!="900"&&check_res!="901")
        {threadpool->EnqueueStr(que_str);}
        res.set_status_and_content(cinatra::status_type::ok, "{\"code\":200,\"info\":\""+check_res+"\"}");
    });

    server.set_http_handler<cinatra::GET, cinatra::POST>("/GetCallRecord/", [threadpool = threadpool](cinatra::request &req, cinatra::response &res)
    {

        //checkweboc data
        LOGGER->info("message is {}",std::string(req.body()));
        CallRecord check;
        std::string check_info = std::string(req.body());
        std::string check_res = check.CheckWebOcInfo(check_info);

        std::string que_str ='1'+check_info;
        int type = 1;
        threadpool->EnqueueStr(que_str);
		res.set_status_and_content(cinatra::status_type::ok, "{\"code\":200,\"info\":\""+check_res+"\"}"); 
    });

    server.set_http_handler<cinatra::GET, cinatra::POST>("/CheckUnSync/", [threadpool = threadpool](cinatra::request &req, cinatra::response &res)
    {
        LOGGER->info("message is {}",std::string(req.body()));
        CallRecord check;
        std::string check_info = std::string(req.body());
        std::string check_res = check.CheckUnSync(check_info);

        if(check_res=="902")
        {
            std::vector<std::string> vec = check.ParseUnSync(check_info);
            if(vec.size()==3)
            {
                std::thread aicall_calllog_subsidiary(CheckUnUpdateId,vec);
                aicall_calllog_subsidiary.detach();
            }
        }
		res.set_status_and_content(cinatra::status_type::ok, "{\"code\":200,\"info\":\""+check_res+"\"}"); 
    });
}

int PollingQueue()
{
    DataCache cache;
    cache.PollingQueue();
    return 0;
}
int OcWebPollingQueue()
{
    DataCache cache;
    cache.OcWebPollingQueue();
    return 0;
}
int CallBackActionQueue()
{
    DataCache cache;
    cache.CallBackActionQueue();
    return 0;
}

template<class QueueType>
class TrimuleThreadPool:public ThreadPool2<QueueType>
{
public:
    TrimuleThreadPool(std::shared_ptr<QueueType> queue, size_t minsize, size_t maxsize=10):ThreadPool2<QueueType>(queue,minsize,maxsize){}

    virtual void WorkerRun(bool original)
    {
        while (!ThreadPool2<QueueType>::_stop)
        {
            auto e = ThreadPool2<QueueType>::_queue->GetObjBulk();
            if (e)
            {
                while (!e->empty())
                {
                    DealElement(std::move(e->front()));
                    e->pop();
                }
            }
            else
            {
                if (!original)
                    break;
                ThreadPool2<QueueType>::_queue->WaitComingObj();
            }
        }
    }

    virtual void DealElement(std::string &&s) final
    {
        if (s.size() > 1)
        {
            std::string real_data = s.substr(1, s.size() - 1);

            if (s[0] == '0') // cm ctive
            {
                ormpp::dbng<ormpp::mysql> mysqlclient;
                CallBackData callog;
                CallBackManage cache_action;
                std::string str = real_data;
                int class_judge = 4;
                CallRecord record;
                int a=2;
                CallInfo data = record.GetCallRecord(real_data, a);
        
                callog.cc_number = data.cc_number;
                cache_action.CacheCmData(callog, str, class_judge,mysqlclient);
            }
            else // oc  web
            {
                CallBackManage cache_action;
                cache_action.MakeQueueCache(real_data);
            }
        }
    }
};

int main()
{
    initspdlog();
    auto config = JsonSimpleWrap::GetPaser("conf/trimule_config.json");
    int max_thread_num = 1;
    cinatra::http_server server(max_thread_num);
    server.listen((*config)["httpserver_setting"]["host"].GetString(), (*config)["httpserver_setting"]["port"].GetString());


    using QueueType = std::conditional_t<false, LockQueue<std::string>, FreeLockRingQueue<std::string>>;
    auto queuetask = std::shared_ptr<QueueType>(new QueueType);
    std::shared_ptr<Worker<QueueType>> worker = std::make_shared<WorkerForHttp<QueueType>>(queuetask);
    std::shared_ptr<ThreadPool<QueueType>> threadpool(new ThreadPool(queuetask, worker, 2, 2));
    //or use
    // std::shared_ptr<TrimuleThreadPool<QueueType>> threadpool(new TrimuleThreadPool(queuetask,2,2));
    SetApiCallBackHandler(server, threadpool);

    std::thread polling_queue(PollingQueue);
    std::thread oc_web_polling_queue(OcWebPollingQueue);
    std::thread call_back_action_queue(CallBackActionQueue);

    polling_queue.detach();
    oc_web_polling_queue.detach();
    call_back_action_queue.detach();

    server.run();
   

  
    return 0;
}