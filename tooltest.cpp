#include <iostream>
#include "queue/rte_ring.h"
#include "queue/threadsafecontainer.hpp"
#include "queue/ringqueue.hpp"
#include "tools/timermanager.hpp"
#include "tools/commandcenter.hpp"
#include "tools/threadpool.hpp"
#include <tuple>
#include <vector>
#include <chrono>
#include <thread>
#include <cinatra.hpp>
#include <simplewrapkafka.h>
#include "RWSeparate.hpp"
#include <json/json.h>
#include <type_traits>

using namespace cinatra;

struct point
{
    int *a = nullptr;
    std::string s = "aaaa";
    point()
    {
        std::cout << "普通构造函数" << std::endl;
    }

    point(const point&other):a(other.a),s(other.s)
    {
        std::cout << "拷贝构造函数" << std::endl;
    }
    point(point &&other):a(other.a),s(std::move(other.s))
    {
        std::cout << "移动构造函数" << std::endl;
        other.a = nullptr;
    }
    point& operator = (const point& other)
    {
        std::cout << "赋值构造函数" << std::endl;
        a = other.a;
        s = other.s;
        return *this;
    }
    point& operator = (const point&& other)
    {
        std::cout << "移动赋值构造函数" << std::endl;
        a = other.a;
        s = std::move(other.s);
        return *this;
    }
};

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
    a->AddAlarmInterval(std::chrono::system_clock::now()+std::chrono::seconds(5), TTT{"run fun1interval",3}, std::bind(fun1,TTT{"run fun1interval",3}),std::chrono::seconds(1));

    for(int i=0;i<5;i++)
        a->AddAlarm(std::chrono::system_clock::now()+std::chrono::seconds(10), TTT{"run fun1",i}, std::bind(fun1,TTT{"run fun1",i}));

    // a->DeleteAlarm(std::bind([](const TTT& t, int id){ return t.id == id; },std::placeholders::_1, 3));


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

#include "tools/register.hpp"
void testregister()
{
    class TTT
    {
    public:
        void test(int a)
        {
            std::cout << a << std::endl;
        }
        int htest(int a,int b)
        {
            return a+b;
        }
        int a =11111;
    };
    FunctionsManager::GetInstance()->register_handler("test", [](TTT a){std::cout << a.a << std::endl;return std::string("aa");});
    FunctionsManager::GetInstance()->register_handler("test", [](TTT a){std::cout << a.a << std::endl;return std::string("aa");});
    FunctionsManager::GetInstance()->register_handler("test2", [](std::string a){std::cout << a << std::endl;});
    // auto r = FunctionsManager::GetInstance()->call<std::string>("test", TTT());
    // std::cout << r << std::endl;
    // FunctionsManager::GetInstance()->call<std::string>("test2", std::string("ss"));

    TTT t;
    FunctionsManager::GetInstance()->register_handler("test3", &TTT::htest, &t);
    FunctionsManager::GetInstance()->register_handler("test4", &TTT::test, &t);
    // auto r2 = FunctionsManager::GetInstance()->call<int>("test3", 10, 5);
    // std::cout << r2 << std::endl;
    FunctionsManager::GetInstance()->call<void>("test4", 10);
    FunctionsManager::GetInstance()->call<std::string>("test5", 10);

    std::cout << "end" << std::endl;
}

void testRingFreeLockQueue()
{
    //少分支预测，for循环展开，cacheline对齐
    FreeLockRingQueue<int,100> q;
    int n = 100000;
    int vc1[100000] = {0};
    int n1 = 0;
    int vc2[100000] = {0};
    int n2 = 0;
    int vc3[100000] = {0};
    bool run = true;
    std::cout<<"numeric_limits<unsigned int>::max()= "<< std::numeric_limits<unsigned int>::max()<< std::endl;
    
    auto p = [&q,n]()
    {
        for(int i=0;i<n;i++)
            while(!q.AddObj(std::move(i))) {}
    };
    auto c = [&run, &q](int *vc, int *n)
    {
        while(run)
        {
            auto e = q.GetObj();
            if(e)
            {
                // std::cout << std::this_thread::get_id() << " cnum:"<< e << std::endl;
                vc[*e]++;
                (*n)++;
            }
        }
        std::cout << "stop thread" << std::endl;
    };
    std::thread p1(p);
    p1.detach();
    std::thread p2(p);
    p2.detach();
    std::thread c1(c,vc1,&n1);
    c1.detach();
    std::thread c2(c,vc2,&n2);
    c2.detach();
    std::this_thread::sleep_for(std::chrono::seconds(15));

    std::thread m([&q,&vc1,&vc2,&vc3,n]()
    {
        for(int i=0;i<n;i++)
        {
            vc3[i] = vc1[i] + vc2[i];
            if(vc3[i] != 2)
                std::cout << "num:" << i << " vc1:" << vc1[i] << " vc2:" << vc2[i] << std::endl;
        }
    });
    m.join();
    run = false;
    std::cout << "n1:" << n1 << " n2:" << n2 << std::endl;
    std::cout << "Hello, world!" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

void testLockQueue()
{
    LockQueue<point,100> q;
    auto p1 = point();
    p1.s = "aaaaa";
    auto p2 = point();
    p2.s = "bbbbb";
    q.AddObj(std::move(p1));
    q.AddObj(std::move(p2));
    auto p = q.GetObj();

    p1.s = "aaaaa";
    p2.s = "bbbbb";
    std::vector<point> v;
    v.push_back(p1);
    v.push_back(p2);
    q.AddObjBulk(std::move(v));
    auto bulk = q.GetObjBulk();
    q.AddObj(std::move(p1));
    std::cout << "end" << std::endl;
}

void testFThreadPool()
{
    using QueueType = std::conditional_t<true, LockQueue<std::function<void()>>,  FreeLockRingQueue<std::function<void()>>>;
    ThreadPool<QueueType> pool(2);
    std::cout << "atart" << std::endl;
    std::vector< std::future<int> > results;
    for(int i = 0; i < 3; ++i) 
    {
        results.emplace_back(
            pool.EnqueueFun([i] ()
            {
                std::cout << "hello " << i << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                std::cout << "world " << i << std::endl;
                return i*i;
            })
        );
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    for(auto && result: results)
        std::cout << result.get() << ' ';
    std::cout << std::endl;
}


struct Worker2Params
{
    Worker2Params() = default;
    Worker2Params(int a):worker2paramsa(a){};
    Worker2Params(Worker2Params &&other):worker2paramsa(other.worker2paramsa){};
    Worker2Params(const Worker2Params &other):worker2paramsa(other.worker2paramsa) {};
    Worker2Params& operator = (const Worker2Params&& other)
    {
        worker2paramsa = other.worker2paramsa;
        return *this;
    }
    Worker2Params& operator = (const Worker2Params& other)
    {
        worker2paramsa = other.worker2paramsa;
        return *this;
    }
    int worker2paramsa;
};

template<class T>
class Worker2:public Worker<T>
{
public:
    Worker2(std::shared_ptr<T> queue):Worker<T>(queue){};

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

    virtual typename std::enable_if<std::is_same<typename T::Type, Worker2Params>::value>::type
    // typename std::enable_if<std::is_same<typename GetContainerType<T>::Type, Worker2Params>::value>::type
    DealElement(Worker2Params &&worker2params)
    {
        std::cout << worker2params.worker2paramsa << std::endl;
    }
};

void testPThreadPool()
{
    using QueueType = std::conditional_t<false, LockQueue<Worker2Params>,  FreeLockRingQueue<Worker2Params>>;
    auto queuetask = std::shared_ptr<QueueType>(new QueueType);
    std::shared_ptr<Worker<QueueType>> worker = std::make_shared<Worker2<QueueType>>(queuetask);

    ThreadPool pool(queuetask,worker,2);
    for(int i=0;i<10;i++)
        pool.EnqueueStr(Worker2Params(i));
}

#include "dbng.hpp"
#include "mysql.hpp"

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

void testormpp()
{
	ormpp::dbng<ormpp::mysql> mysql;
	mysql.connect("127.0.0.1", "db", "123", "ai");

    auto res = mysql.query<aicall_tts_file_cache>("id = 5622");
	for(auto& file : res){
		std::cout<<file.id<<" "<<file.TTS_text<<" "<<file.TTS_version_code<<std::endl;
	}
}

int main()
{
    testormpp();
    // testPThreadPool();
    // testFThreadPool();
    // testLockQueue();
    // testRingFreeLockQueue();
    // testregister();
    // testtimermanager();
    // testDealCommandCenter();
    // ThreadSafePriorityQueueTest();
    // testfreelock();
    // testKafkaCinatra();
    // testmemorypool();
    // testpublicfun();
}