#include "cinatra.hpp"
#include "queue/ringqueue.hpp"
#include "tools/threadpool.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"

struct aicall_tts_file_cache
{
	int id;
	std::string TTS_text;
	int TTS_version_code;
    std::string tts_src;
    int tts_duration;
    int create_time;
    int access_time;
    int extension;
};
REFLECTION(aicall_tts_file_cache, id, TTS_text, TTS_version_code, tts_src, tts_duration, create_time, access_time, extension)

template<class T>
class WorkerForHttp:public Worker<T>
{
public:
    WorkerForHttp(std::shared_ptr<T> queue):Worker<T>(queue){};

protected:
    virtual void WorkerRun(bool original)
    {
        ormpp::dbng<ormpp::mysql> mysqlclient;
	    mysqlclient.connect("127.0.0.1", "db", "123", "ai");
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
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

    }

    virtual typename std::enable_if<std::is_same<typename T::Type, std::string>::value>::type
    // typename std::enable_if<std::is_same<typename GetContainerType<T>::Type, Worker2Params>::value>::type
    DealElement(ormpp::dbng<ormpp::mysql> &mysql, std::string &&s)
    {
        auto res = mysql.query<aicall_tts_file_cache>("id = 5622");
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

int main()
{
    int max_thread_num = 1;
	cinatra::http_server server(max_thread_num);
    server.listen("0.0.0.0", "8080");
    
    using QueueType = std::conditional_t<true, LockQueue<std::string>,  FreeLockRingQueue<std::string>>;
    auto queuetask = std::shared_ptr<QueueType>(new QueueType);
    std::shared_ptr<Worker<QueueType>> worker = std::make_shared<WorkerForHttp<QueueType>>(queuetask);
    std::shared_ptr<ThreadPool<QueueType>> threadpool(new ThreadPool(queuetask,worker,2));

    SetApiCallBackHandler(server, threadpool);

	server.run();
	return 0;
}