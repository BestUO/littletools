#include "synicmanager.hpp"
#include "messageprocess/messageprocess.h"
#include "cinatra/cinatra.hpp"
#include "queue/ringqueue.hpp"
#include "tools/threadpool.hpp"
#include "tools/jsonwrap.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include "httpclient/HttpRequester.h"
#include "global.hpp"

struct InfoBase
{
    enum Status{NOTREADY,OCREADY,CMREADY};

    std::string message="";
    std::string url="";
    Status status=NOTREADY;

    InfoBase(std::string message,std::string url,Status status):message(std::move(message)),url(std::move(url)),status(status){};
};

template <class T>
class UpdateMySQLWorker : public Worker<T>
{
public:
    UpdateMySQLWorker(std::shared_ptr<T> queue) : Worker<T>(queue){};

protected:
    virtual void WorkerRun(bool original)
    {
        GETMYSQLCLIENT
        while (!Worker<T>::_stop)
        {
            auto e = Worker<T>::_queue->GetObjBulk();
            if (e)
            {
                while (!e->empty())
                {
                    DealElement(mysqlclient, e->front());
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
    void DealElement(ormpp::dbng<ormpp::mysql> &mysqlclient, typename T::Type infoBase)
    {
        auto [calllog_id,task_id,eid] = std::move(MessageProcess::UpdateAllInfo(infoBase->message,mysqlclient));
        if(!infoBase->url.empty())
        {
            std::string cbstring = MessageProcess::GetCallBackString(std::move(calllog_id), mysqlclient);
            HttpRequester::PostUrl(infoBase->url,cbstring,1,true);
        }
        std::string sql = "update aicall_calllog_subsidiary set update_status=2 where calllog_id="+calllog_id;
        mysqlclient.execute(sql);
    }
};

template <class T>
void SetHttpHandler(cinatra::http_server &server, T threadpool)
{
    server.set_http_handler<cinatra::GET, cinatra::POST>("/", [threadpool = threadpool](cinatra::request &req, cinatra::response &res)
    {
        LOGGER->info("/ receive message is {}",std::string(req.body()));
        auto [check_res,ccnumber] = MessageProcess::CheckCCNumber(req.body());
        if(check_res == Response::SUCCESS)
        {
            auto instance = SynicManager<std::shared_ptr<InfoBase>>::GetInstance();
            if(auto opt=instance->GetFromSynicMap(ccnumber);opt != std::nullopt && opt.value()->status==InfoBase::Status::OCREADY)
            {
                opt.value()->message=std::move(req.body().data());
                if(instance->DeleteFromSynicMap(ccnumber))
                    threadpool->EnqueueStr(opt.value());
            }
            else
            {
                auto tmp = std::make_shared<InfoBase>(std::move(req.body().data()),"",InfoBase::Status::CMREADY);
                instance->InsertToSynicMap(ccnumber,tmp);
            }
        }
        res.set_status_and_content(cinatra::status_type::ok, "{\"code\":200,\"info\":\""+std::to_string(check_res)+"\"}"); 
    });

    server.set_http_handler<cinatra::GET, cinatra::POST>("/trimule/ocready/", [threadpool = threadpool](cinatra::request &req, cinatra::response &res)
    {   
        LOGGER->info("/trimule/ocready/ receive message is {}",std::string(req.body()));
        auto [check_res,ccnumber,url] = MessageProcess::CheckFromOC(req.body());

        auto instance = SynicManager<std::shared_ptr<InfoBase>>::GetInstance();
        if(check_res == Response::SUCCESS)
        {
            if(auto opt=instance->GetFromSynicMap(ccnumber);opt != std::nullopt && opt.value()->status==InfoBase::Status::CMREADY)
            {
                opt.value()->url=std::move(url);
                if(instance->DeleteFromSynicMap(ccnumber))
                    threadpool->EnqueueStr(std::move(opt.value()));
            }
            else
            {
                auto tmp = std::make_shared<InfoBase>("",std::move(url),InfoBase::Status::OCREADY);
                instance->InsertToSynicMap(ccnumber,tmp);
            }
        }

		res.set_status_and_content(cinatra::status_type::ok, "{\"code\":200,\"info\":\""+std::to_string(check_res)+"\"}"); 
    });


    server.set_http_handler<cinatra::GET, cinatra::POST>("/trimule/cmsynic/", [threadpool = threadpool](cinatra::request &req, cinatra::response &res)
    {
        //checkweboc data
        LOGGER->info("/trimule/cmsynic/ receive message is {}",std::string(req.body()));
        auto [check_res,ccnumber] = MessageProcess::CheckCCNumber(req.body());
        if(check_res==Response::SUCCESS)
        {
            SynicManager<std::shared_ptr<InfoBase>>::GetInstance()->DeleteFromSynicMap(ccnumber);
            
            GETMYSQLCLIENT

            std::string message = MessageProcess::UpdateCallRecord(ccnumber, mysqlclient);
            MessageProcess::UpdateAllInfo(message,mysqlclient);
        }

		res.set_status_and_content(cinatra::status_type::ok, "{\"code\":200,\"info\":\""+std::to_string(check_res)+"\"}"); 
    });

    server.set_http_handler<cinatra::GET, cinatra::POST>("/trimule/forcecallback/", [threadpool = threadpool](cinatra::request &req, cinatra::response &res)
    {
        //checkweboc data
        LOGGER->info("/trimule/forcecallback/ receive message is {}",std::string(req.body()));
        auto [check_res,eid,calllog_id, url] = MessageProcess::CheckForceCallBack(req.body());
        if(check_res==Response::SUCCESS)
        {
            GETMYSQLCLIENT
            // auto url = MessageProcess::GetCallBackUrl(eid, mysqlclient);
            if(!url.empty())
            {
                std::string cbstring = MessageProcess::GetCallBackString(std::move(calllog_id), mysqlclient);
                LOGGER->info("callbac info is {}",cbstring);
                HttpRequester::PostUrl(url,cbstring,1,true);
            }
        }

		res.set_status_and_content(cinatra::status_type::ok, "{\"code\":200,\"info\":\""+std::to_string(check_res)+"\"}"); 
    });
}

std::string GetFileSuffix()
{
    // std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    // std::string s(30, '\0');
    // std::strftime(&s[0], s.size(), ".%Y%m%d%H%M", std::localtime(&now));
    // return s;
    return "." + std::to_string(getpid());
}

void initspdlog()
{
    spdlog::flush_on(spdlog::level::info);
    // spdlog::flush_every(std::chrono::seconds(5));
    auto file_logger = spdlog::rotating_logger_mt<spdlog::async_factory>(SPDLOGGERNAME, SPDLOG_FILENAME+GetFileSuffix(), 1024 * 1024 * 200, 5);
    LOGGER->set_level(spdlog::level::info); // Set global log level to info
    LOGGER->set_pattern("[%Y-%m-%d %H:%M:%S.%e %^%L%$ %t] %v");
}

void recoverfun()
{
    GETMYSQLCLIENT
    auto time = (unsigned int)std::time(nullptr);
    while(true)
    {
        auto subsidiaryitem = mysqlclient.query<std::tuple<std::string,unsigned int,std::string,std::string>>(R"(select calllog_id,update_status, cc_number,url from aicall_calllog_subsidiary where (update_status=0 or url!="" and update_status=1) and unix_timestamp(create_time) < ? limit 100;)",time);
        LOGGER->info(R"(select cc_number,url from aicall_calllog_subsidiary where (update_status=0 or url!="" and update_status=1) and unix_timestamp(create_time) < {} limit 100;)", time);
        if(subsidiaryitem.empty())
            break;
        else
        {
            for(auto &&item:subsidiaryitem)
            {
                auto [calllog_id,update_status,cc_number,url] = item;
                if(update_status==0 && !cc_number.empty())//oc ocready
                {
                    std::string message = MessageProcess::UpdateCallRecord(cc_number, mysqlclient);
                    MessageProcess::UpdateAllInfo(message,mysqlclient);
                }
                if(!url.empty())
                {
                    std::string cbstring = MessageProcess::GetCallBackString(std::move(calllog_id), mysqlclient);
                    HttpRequester::PostUrl(url,cbstring,1,true);
                }
                std::string sql = "update aicall_calllog_subsidiary set update_status=2 where calllog_id="+calllog_id;
                mysqlclient.execute(sql);
            }
        }
    }

    LOGGER->info("recover finish");
}

int main()
{
    initspdlog();
    std::thread recover(recoverfun);
    recover.detach();
    auto config = JsonSimpleWrap::GetPaser("conf/trimule_config.json");
    int max_thread_num = 1;
    cinatra::http_server server(max_thread_num);
    server.listen((*config)["httpserver_setting"]["host"].GetString(), (*config)["httpserver_setting"]["port"].GetString());


    using QueueType = std::conditional_t<false, LockQueue<std::shared_ptr<InfoBase>>, FreeLockRingQueue<std::shared_ptr<InfoBase>>>;
    auto queuetask = std::shared_ptr<QueueType>(new QueueType);
    std::shared_ptr<Worker<QueueType>> worker = std::make_shared<UpdateMySQLWorker<QueueType>>(queuetask);
    std::shared_ptr<ThreadPool<QueueType>> threadpool(new ThreadPool(queuetask, worker, 2, 2));

    SetHttpHandler(server, threadpool);

    server.run();

    return 0;
}