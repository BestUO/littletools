#include <iostream>
#include "queue/rte_ring.h"
#include "queue/threadsafecontainer.hpp"
#include "tools/timermanager.hpp"
#include "tools/commandcenter.hpp"
#include <tuple>
#include <vector>
#include <chrono>
#include <thread>
#include <cinatra.hpp>
#include <simplewrapkafka.h>
#include "RWSeparate.hpp"
#include <json/json.h>

using namespace cinatra;



void testDealCommandCenter()
{
    struct TTT
    {
        int a;
        std::string b;
    };
    auto returnfun = [](int a, std::string b) -> TTT
    {
        return TTT{a,b};
    };
    auto printfun = [](int a, std::string b)
    {
        std::cout << "print fun:" << a << " " << b << std::endl;
    };

    DealCommandCenter *a = DealCommandCenter::GetInstance();

    TTT t;
    a->AsyncDealCommandAndGetResult(t, returnfun, 1, "1");
    std::cout << "return fun:" << t.a << " " << t.b << std::endl;

    a->AsyncDealCommand(printfun, 4, "456");
}


void testfreelock()
{
    auto aaa = RTE_Ring("Myring",1024,false);

    std::vector<void*> v1;
    for(int n = 0;n < 5000000;n++)
        v1.push_back(new int(n));

    std::vector<std::thread> vp;
    for(int a = 0;a<2;a++)
    {
        std::thread t([&]()
        {
            for (auto b:v1)
                while(!aaa.rte_ring_mp_enqueue_bulk(&b, 1, nullptr))
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
        });
        vp.emplace_back(std::move(t));
    }

    std::vector<std::thread> vc;
    for(int a = 0;a<4;a++)
    {
        std::thread t([&]()
        {
            int total = 0;
            while(true)
            {
                void *ptr = nullptr;
                int num = aaa.rte_ring_mc_dequeue_bulk(&ptr, 1, nullptr);
                if(num)
                    total += *(int*)ptr;
            }
            
        });
        vc.emplace_back(std::move(t));
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    for(auto t = vp.begin();t!=vp.end();t++)
        t->join();
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = endTime - startTime;
    std::cout << "3 " << fp_ms.count() << std::endl;

    for(auto t = vc.begin();t!=vc.end();t++)
        t->join();
    for(auto a:v1)
        delete (int*)a;
}

int cinatra_upload()
{
    http_server server(std::thread::hardware_concurrency());
    server.listen("0.0.0.0", "8080");

    //http upload(multipart)
    server.set_http_handler<GET, POST>("/upload_multipart", [](request& req, response& res) {
        assert(req.get_content_type() == content_type::multipart);
        
        auto& files = req.get_upload_files();
        for (auto& file : files) {
            std::cout << file.get_file_path() << " " << file.get_file_size() << std::endl;
        }

        res.set_status_and_content(status_type::ok, "multipart finished");
    });

    server.run();
    return 0;
}

int cinatra_websocket()
{
    http_server server(std::thread::hardware_concurrency());
    server.listen("0.0.0.0", "8080");

    //web socket
    server.set_http_handler<GET, POST>("/ws", [](request& req, response& res) {
        assert(req.get_content_type() == content_type::websocket);

        req.on(ws_open, [](request& req){
            std::cout << "websocket start" << std::endl;
        });

        req.on(ws_message, [](request& req) {
            auto part_data = req.get_part_data();
            //echo
            std::string str = std::string(part_data.data(), part_data.length());
            req.get_conn<cinatra::NonSSL>()->send_ws_string(std::move(str));
            std::cout << part_data.data() << std::endl;
        });

        req.on(ws_error, [](request& req) {
            std::cout << "websocket pack error or network error" << std::endl;
        });
    });
    server.enable_timeout(false);
    server.run();
    return 0;
}

int ttt()
{
    int max_thread_num = std::thread::hardware_concurrency();
    http_server server(max_thread_num);
    server.listen("0.0.0.0", "8080");
    server.set_http_handler<GET, POST>("/", [&](request& req, response& res) 
    {
        res.set_delay(true);
        std::string ip("192.168.10.52_");
        std::string uuid;
        auto con = req.get_conn<http_server::type>();
        RWSeparate2<http_server::type>::inst().Add2Map(con, uuid);
        SimpleWrapKafka::inst().Add2Kafka("mytest", ip + uuid + "_testfromcinatra");

        // std::string filename("/home/uncle_orange/BorayProduct/tools/www/1616579920094476511.jpg");
        // auto file = std::make_shared<std::ifstream>(filename, std::ios::binary);
        // std::string content;
        // const size_t size = 5 * 1024 * 1024;
        // content.resize(size);
        // file->read(&content[0], size);
        // int64_t read_len = (int64_t)file->gcount();

        // if (read_len < size)
        //     content.resize(read_len);
        // res.set_status_and_content(status_type::ok, std::move(content),req_content_type::multipart);

        res.set_status_and_content(status_type::ok, "multipart finished");
    });
    server.enable_timeout(false);
    server.run();
    return 0;
}

int cinatra_uploadfile()
{
    // http_server server(std::thread::hardware_concurrency());
    http_server server(1);
    server.listen("0.0.0.0", "8080");

    //http upload(multipart)
    server.set_http_handler<GET, POST>("/upload_multipart", [](request& req, response& res) 
    {
        assert(req.get_content_type() == content_type::multipart);
        auto& files = req.get_upload_files();
        for (auto& file : files) {
            std::cout << file.get_file_path() << " " << file.get_file_size() << std::endl;
        }

        res.set_delay(true);
        // res.set_status_and_content(status_type::ok, "multipart finished");
        

        std::string ip("192.168.10.52_");
        std::string uuid;
        auto con = req.get_conn<http_server::type>();
        RWSeparate2<http_server::type>::inst().Add2Map(con, uuid);
        SimpleWrapKafka::inst().Add2Kafka("mytest", ip + uuid + "_testfromcinatra");
    });

    server.run();
    return 0;
}


void readStrJson()
{
	//字符串  
	const char* str =
		"{\"name\":\"shuiyixin\",\"age\":\"21\",\"sex\":\"man\"}";
 
 
	Json::Reader reader;
	Json::Value root;
 
	//从字符串中读取数据  
	if (reader.parse(str, root))
	{
		// std::string name = root["name"].asString();
		// int age = root["nomen"].asInt();
		// string sex = root["sex"].asString();
		// cout << name + "," << age << "," << sex <<  endl;
	}
 
}

void ThreadSafePriorityQueueTest()
{
    struct TTT
    {
        int a;
        std::string b;
        bool operator<(const TTT &t) const
        {
            return a > t.a;
        }
    };

    ThreadSafePriorityQueue<TTT> a;
    a.AddObj(TTT{1,"aa"});
    a.AddObj(TTT{5,"aa"});
    a.AddObj(TTT{4,"aa"});
    a.AddObj(TTT{8,"aa"});
    a.AddObj(TTT{5,"aa"});

    a.DeleteObj([](const auto& p){ return p.a == 5; });

    TTT b;
    auto c = a.GetObj(b);
    std::cout << b.a << std::endl;
    c = a.GetObj(b,[](const TTT &obj) -> bool
    {
        return 0<obj.a ? true:false;
    });
    std::cout << b.a << std::endl;
    c = a.GetObj(b,[](const TTT &obj) -> bool
    {
        return 2<obj.a ? true:false;
    });
    std::cout << b.a << std::endl;
}

void testKafkaCinatra()
{
    auto f = []()->std::tuple<int,double,std::string>
    {
        return std::make_tuple(1,2.3,"456");
    };
    std::string brokers("192.168.10.52:9092");
    std::string topic("mytest");
    std::string groupid("g1");

    SimpleWrapKafka::inst().CreateProducer(brokers, topic);
    SimpleWrapKafka::inst().CreateConsumer(brokers,topic,groupid,[](std::string message)
    {
        RWSeparate2<http_server::type>::inst().ResponseClient(message);
    });
    // SimpleWrapKafka::inst().Add2Kafka(topic,"wrap2");
    // RWSeparate::inst().printt();
    cinatra_uploadfile();
    // ttt();
    // net();
    std::cout << "haha" << std::endl;
    auto [x,y,z] = f();
    std::cout << x << y << z << std::endl;
}

void printnow()
{
    auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm* ptm = localtime(&tt);
    char date[60] = {0};
    sprintf(date, "%d-%02d-%02d      %02d:%02d:%02d",
        (int)ptm->tm_year + 1900,(int)ptm->tm_mon + 1,(int)ptm->tm_mday,
        (int)ptm->tm_hour,(int)ptm->tm_min,(int)ptm->tm_sec);
    std::cout << date << std::endl;
}

void testtimermanager()
{
    struct TTT
    {
        std::string name;
        int id;
    };

    auto fun1 = [](TTT t)
    {
        std::cout << t.name.c_str() << '\t' << t.id << std::endl;
    };
    TimerManager<TTT> *a = TimerManager<TTT>::GetInstance();
    sleep(10);

    a->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(20), TTT{"run fun1",20}, std::bind(fun1,TTT{"run fun1",10}));
    a->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(5), TTT{"run fun1",3}, std::bind(fun1,TTT{"run fun1",3}));

    for(int i=0;i<5;i++)
        a->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(10), TTT{"run fun1",i}, std::bind(fun1,TTT{"run fun1",i}));

    a->DeleteAlarm(std::bind([](const TTT& t, int id){ return t.id == id; },std::placeholders::_1, 3));


    auto fun2 = [](int a, int b)
    {
        std::cout << a << '\t' << b << std::endl;
    };
    TimerManager<TTT> *b = TimerManager<TTT>::GetInstance();
    for(int i=5;i<10;i++)
        b->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(2), TTT{"123",i}, std::bind(fun2,i,i+1));
    b->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(2), TTT{"123", 5}, std::bind(fun2,5,6));
    b->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(2), TTT{"123", 6}, std::bind(fun2,6,7));
    b->DeleteAlarm(std::bind([](const TTT& t, int id){ return t.id == id; },std::placeholders::_1, 5));
}

#include "tools/memorypool.hpp"
void testmemorypool()
{
    struct TTT 
    {
        int_fast64_t foo = 0;
        char data[65535];
        int_fast32_t bar = 0;
        int_fast16_t baz = 0;
        bool boo = false;
        TTT(int_fast64_t foo):foo(foo){};
    };
    MemoryPool<TTT, 1000> pool;
    auto element2 = pool.new_element(1);
    pool.delete_element(element2);
}

#include "tools/globalfun.hpp"
void testpublicfun()
{
    std::cout << CephHashFun(123) << std::endl;
    uint8_t d[3]={1,2,3};
    std::cout << JenkinsHashFun(d,3) << std::endl;
    std::cout << public_align32pow2(5) << std::endl;
}

int main()
{
    // testtimermanager();
    // testDealCommandCenter();
    // ThreadSafePriorityQueueTest();
    // testfreelock();
    // testKafkaCinatra();
    // testmemorypool();
    testpublicfun();
}