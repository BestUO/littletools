#include "cinatra.hpp"
#include "queue/ringqueue.hpp"
#include "tools/threadpool.hpp"
#include "tools/jsonwrap.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include "dbstruct/dbstruct.h"

#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

template<class T>
class WorkerForHttp:public Worker<T>
{
public:
    WorkerForHttp(std::shared_ptr<T> queue):Worker<T>(queue){};

protected:
    virtual void WorkerRun(bool original)
    {
        auto config = JsonSimpleWrap::GetPaser("conf/setting.conf");
        ormpp::dbng<ormpp::mysql> mysqlclient;
	    mysqlclient.connect((*config)["mysql_setting"]["mysql_host"].GetString(), (*config)["mysql_setting"]["mysql_user"].GetString(),
                                     (*config)["mysql_setting"]["mysql_password"].GetString(), (*config)["mysql_setting"]["mysql_db"].GetString());
        while(!Worker<T>::_stop)
        {
            auto e = Worker<T>::_queue->GetObjBulk();
            if(e)
            {
                while(!e->empty())
                {
                    DealElement(mysqlclient, std::move(e->front()));
                    e->pop();
                }
            }
            else
            {
                if(!original)
                    break;
                Worker<T>::_queue->WaitComingObj();
            }
        }

    }

    virtual typename std::enable_if<std::is_same<typename T::Type, std::string>::value>::type
    // typename std::enable_if<std::is_same<typename GetContainerType<T>::Type, Worker2Params>::value>::type
    DealElement(ormpp::dbng<ormpp::mysql> &mysql, std::string &&s)
    {
        LOGGER->info("message #{}", s);
        auto res = mysql.query<aicall_tts_file_cache>("id = 5659");
        for(auto& file : res)
            std::cout<<file.id<<" "<<file.TTS_text<<" "<<file.TTS_version_code<<std::endl;
    }

    virtual typename std::enable_if<std::is_same<typename T::Type, std::string>::value>::type
    DealElement(std::string &&s)
    {
        return;
    }
};

template<class T>
void SetApiCallBackHandler(cinatra::http_server &server, T threadpool)
{
	server.set_http_handler<cinatra::GET, cinatra::POST>("/", [threadpool=threadpool](cinatra::request& req, cinatra::response& res) {
        std::cout << req.body() << std::endl;
        threadpool->EnqueueStr(std::string(req.body()));
		res.set_status_and_content(cinatra::status_type::ok, "hello world");
	});
}

void initspdlog()
{
    spdlog::flush_every(std::chrono::seconds(5));
    auto file_logger = spdlog::rotating_logger_mt<spdlog::async_factory>(SPDLOGGERNAME, SPDLOG_FILENAME, 1024 * 1024 * 200, 5);
    LOGGER->set_level(spdlog::level::info); // Set global log level to info
    LOGGER->set_pattern("[%H:%M:%S:%e %z %^%L%$ %t] %v");
}

int main()
{
    initspdlog();

    auto config = JsonSimpleWrap::GetPaser("conf/setting.conf");
    int max_thread_num = 1;
	cinatra::http_server server(max_thread_num);
    server.listen((*config)["httpserver_setting"]["host"].GetString(), (*config)["httpserver_setting"]["port"].GetString());
    
    using QueueType = std::conditional_t<false, LockQueue<std::string>,  FreeLockRingQueue<std::string>>;
    auto queuetask = std::shared_ptr<QueueType>(new QueueType);
    std::shared_ptr<Worker<QueueType>> worker = std::make_shared<WorkerForHttp<QueueType>>(queuetask);
    std::shared_ptr<ThreadPool<QueueType>> threadpool(new ThreadPool(queuetask,worker,2,2));

    SetApiCallBackHandler(server, threadpool);

	server.run();
    spdlog::shutdown();
	return 0;
}