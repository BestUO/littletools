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

template <class T>
class UpdateMySQLWorker : public threadpool::v1::Worker<T>
{
public:
    UpdateMySQLWorker(std::shared_ptr<T> queue)
        : threadpool::v1::Worker<T>(queue){};

protected:
    virtual void WorkerRun(bool original)
    {
        GETMYSQLCLIENT
        while (!threadpool::v1::Worker<T>::_stop)
        {
            auto e = threadpool::v1::Worker<T>::_queue->GetObjBulk();
            if (e)
            {
                while (!e->empty())
                {
                    DealElement(mysqlclient, std::move(e->front()));
                    e->pop();
                }
            }
            else
            {
                if (!original)
                    break;
                threadpool::v1::Worker<T>::_queue->WaitComingObj();
            }
        }
    }

    virtual typename std::enable_if<
        std::is_same<typename T::Type, std::string>::value>::type
    DealElement(ormpp::dbng<ormpp::mysql>& mysqlclient, std::string&& message)
    {
        auto [calllog_id, task_id, eid]
            = std::move(MessageProcess::UpdateAllInfo(message, mysqlclient));
        auto url = MessageProcess::GetCallBackUrl(eid, mysqlclient);
        if (1 && !url.empty())
        {
            std::string cbstring = MessageProcess::GetCallBackString(
                std::move(calllog_id), mysqlclient);
            LOGGER->info("callbac info is {}", cbstring);
            HttpRequester::PostUrl(url, cbstring, true);
        }
    }
};

template <class T>
void SetHttpHandler(cinatra::http_server& server, T threadpool)
{
    server.set_http_handler<cinatra::GET, cinatra::POST>("/",
        [threadpool = threadpool](
            cinatra::request& req, cinatra::response& res) {
            LOGGER->info("/ receive message is {}", std::string(req.body()));
            auto [check_res, ccnumber]
                = MessageProcess::CheckCCNumber(req.body());

            auto instance = SynicManager::GetInstance();
            if (auto opt = instance->GetFromSynicMap(ccnumber);
                opt != std::nullopt)
            {
                threadpool->EnqueueStr(std::move(std::move(req.body().data())));
                instance->DeleteFromSynicMap(ccnumber);
            }
            else
                instance->InsertToSynicMap(ccnumber, req.body());
            res.set_status_and_content(cinatra::status_type::ok,
                "{\"code\":200,\"info\":\"" + std::to_string(check_res)
                    + "\"}");
        });

    server.set_http_handler<cinatra::GET, cinatra::POST>("/trimule/ocready/",
        [threadpool = threadpool](
            cinatra::request& req, cinatra::response& res) {
            LOGGER->info("/trimule/ocready/ receive message is {}",
                std::string(req.body()));
            auto [check_res, ccnumber]
                = MessageProcess::CheckCCNumber(req.body());

            auto instance = SynicManager::GetInstance();
            if (auto opt = instance->GetFromSynicMap(ccnumber);
                opt != std::nullopt)
            {
                threadpool->EnqueueStr(std::move(opt.value().data()));
                instance->DeleteFromSynicMap(ccnumber);
            }
            else
                instance->InsertToSynicMap(ccnumber, req.body());
            res.set_status_and_content(cinatra::status_type::ok,
                "{\"code\":200,\"info\":\"" + std::to_string(check_res)
                    + "\"}");
        });

    server.set_http_handler<cinatra::GET, cinatra::POST>("/trimule/cmsynic/",
        [threadpool = threadpool](
            cinatra::request& req, cinatra::response& res) {
            // checkweboc data
            LOGGER->info("/trimule/cmsynic/ receive message is {}",
                std::string(req.body()));
            auto [check_res, ccnumber]
                = MessageProcess::CheckCCNumber(req.body());
            if (check_res == Response::SUCCESS)
            {
                SynicManager::GetInstance()->DeleteFromSynicMap(ccnumber);

                GETMYSQLCLIENT

                std::string message
                    = MessageProcess::UpdateCallRecord(ccnumber, mysqlclient);
                MessageProcess::UpdateAllInfo(message, mysqlclient);
            }

            res.set_status_and_content(cinatra::status_type::ok,
                "{\"code\":200,\"info\":\"" + std::to_string(check_res)
                    + "\"}");
        });

    server.set_http_handler<cinatra::GET, cinatra::POST>(
        "/trimule/forcecallback/",
        [threadpool = threadpool](
            cinatra::request& req, cinatra::response& res) {
            // checkweboc data
            LOGGER->info("/trimule/forcecallback/ receive message is {}",
                std::string(req.body()));
            auto [check_res, eid, calllog_id]
                = MessageProcess::CheckEidCalllogId(req.body());
            if (check_res == Response::SUCCESS)
            {
                GETMYSQLCLIENT
                auto url = MessageProcess::GetCallBackUrl(eid, mysqlclient);
                if (!url.empty())
                {
                    std::string cbstring = MessageProcess::GetCallBackString(
                        std::move(calllog_id), mysqlclient);
                    LOGGER->info("callbac info is {}", cbstring);
                    HttpRequester::PostUrl(url, cbstring, true);
                }
            }

            res.set_status_and_content(cinatra::status_type::ok,
                "{\"code\":200,\"info\":\"" + std::to_string(check_res)
                    + "\"}");
        });
}

void initspdlog()
{
    spdlog::flush_every(std::chrono::seconds(5));
    auto file_logger = spdlog::rotating_logger_mt<spdlog::async_factory>(
        SPDLOGGERNAME, SPDLOG_FILENAME, 1024 * 1024 * 200, 5);
    LOGGER->set_level(spdlog::level::info);  // Set global log level to info
    LOGGER->set_pattern("[%Y-%m-%d %H:%M:%S.%e %^%L%$ %t] %v");
}

int main()
{
    initspdlog();
    auto config        = JsonSimpleWrap::GetPaser("conf/trimule_config.json");
    int max_thread_num = 1;
    cinatra::http_server server(max_thread_num);
    server.listen((*config)["httpserver_setting"]["host"].GetString(),
        (*config)["httpserver_setting"]["port"].GetString());

    using QueueType = std::conditional_t<false,
        LockQueue<std::string>,
        FreeLockRingQueue<std::string>>;
    auto queuetask  = std::shared_ptr<QueueType>(new QueueType);
    std::shared_ptr<threadpool::v1::Worker<QueueType>> worker
        = std::make_shared<UpdateMySQLWorker<QueueType>>(queuetask);
    std::shared_ptr<threadpool::v1::ThreadPool<QueueType>> threadpool(
        new threadpool::v1::ThreadPool(queuetask, worker, 2, 2));

    SetHttpHandler(server, threadpool);

    server.run();

    return 0;
}