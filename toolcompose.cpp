#include "cinatra.hpp"
#include "queue/ringqueue.hpp"
#include "tools/threadpool.hpp"

template<class T>
void SetApiCallBackHandler(cinatra::http_server &server, T threadpool)
{
	server.set_http_handler<cinatra::GET, cinatra::POST>("/", [threadpool=threadpool](cinatra::request& req, cinatra::response& res) {
        std::cout << req.body() << std::endl;
        threadpool->EnqueueStr(std::string(req.body()));
		res.set_status_and_content(cinatra::status_type::ok, "hello world");
	});
}

template<class T>
class WorkerForHttp:public Worker<T>
{
public:
    WorkerForHttp(std::shared_ptr<T> queue):Worker<T>(queue){};

protected:
    virtual void WorkerRun(bool original)
    {
        while(!Worker<T>::_stop)
        {
            auto e = Worker<T>::_queue->GetObjBulk();
            if(e)
            {
                while(!e->empty())
                {
                    DealElement(std::move(e->front()));
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
    DealElement(std::string &&s)
    {
        std::cout << s << std::endl;
    }
};

int main()
{
    int max_thread_num = 1;
	cinatra::http_server server(max_thread_num);
    server.listen("0.0.0.0", "8080");
    
    using QueueType = std::conditional_t<false, LockQueue<std::string>,  FreeLockRingQueue<std::string>>;
    auto queuetask = std::shared_ptr<QueueType>(new QueueType);
    std::shared_ptr<Worker<QueueType>> worker = std::make_shared<WorkerForHttp<QueueType>>(queuetask);

    std::shared_ptr<ThreadPool<QueueType>> threadpool(new ThreadPool(queuetask,worker,2));
    // std::shared_ptr<ThreadPool<LockQueue<std::function<void()>>>> threadpool(new ThreadPool<LockQueue<std::function<void()>>>(2));

    SetApiCallBackHandler(server, threadpool);

	server.run();
	return 0;
}