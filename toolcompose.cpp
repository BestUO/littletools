#include "cinatra.hpp"
#include "queue/ringqueue.hpp"
#include "tools/threadpool.hpp"

void SetApiCallBackHandler(cinatra::http_server &server, std::shared_ptr<ThreadPool> threadpool)
{
	server.set_http_handler<cinatra::GET, cinatra::POST>("/", [threadpool=threadpool](cinatra::request& req, cinatra::response& res) {
        std::cout << req.body() << std::endl;
        threadpool->enqueue([strview=req.body()] 
        {
            std::cout << strview << std::endl;
        });
		res.set_status_and_content(cinatra::status_type::ok, "hello world");
	});
}

int main()
{
    int max_thread_num = 1;
	cinatra::http_server server(max_thread_num);
    server.listen("0.0.0.0", "8080");
    
    // std::shared_ptr<FreeLockRingQueue<std::string_view>> queue(new FreeLockRingQueue<std::string_view>);
    std::shared_ptr<ThreadPool> threadpool(new ThreadPool(2));

    SetApiCallBackHandler(server, threadpool);

	server.run();
	return 0;
}