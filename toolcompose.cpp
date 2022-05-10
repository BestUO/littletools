#include "cinatra.hpp"
#include "queue/ringqueue.hpp"
#include "tools/threadpool.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include "work/newstructure/GetCallRecord.h"
#include "settingParser/settingParser.h"
#include <vector>
#include <string>
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

void initspdlog()
{
    spdlog::flush_every(std::chrono::seconds(5));
    auto file_logger = spdlog::rotating_logger_mt<spdlog::async_factory>(SPDLOGGERNAME, SPDLOG_FILENAME, 1024 * 1024 * 200, 5);
    LOGGER->set_level(spdlog::level::info); // Set global log level to info
    LOGGER->set_pattern("[%H:%M:%S:%e %z %^%L%$ %t] %v");
}

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

template <class T>
class WorkerForHttp : public Worker<T>
{
public:
    WorkerForHttp(std::shared_ptr<T> queue) : Worker<T>(queue){};

protected:
    virtual void WorkerRun(bool original)
    {
        ormpp::dbng<ormpp::mysql> mysqlclient;
        settingParser mysql_example;
        std::string mysql_db = mysql_example.GetMysqlDb();
        std::string mysql_host = mysql_example.GetMysqlHost();
        std::string mysql_password = mysql_example.GetMysqlPassord();
        std::string mysql_user = mysql_example.GetMysqlUser();
        // std::cout<<mysql_db<<"  "<<mysql_host<<"  "<<mysql_password<<" "<<mysql_user<<std::endl;
         mysqlclient.connect(mysql_host.c_str(), mysql_user.c_str(), mysql_password.c_str(), mysql_db.c_str());
        DealElement(mysqlclient," std::move(e->front())");
        // while (!Worker<T>::_stop)
        // {
        //     auto e = Worker<T>::_queue->GetObjBulk();
        //     if (e)
        //     {

        //         while (!e->empty())
        //         {
        //             DealElement(mysqlclient, std::move(e->front()));
        //             e->pop();
        //         }
        //     }
        //     else
        //     {
        //         if (!original)
        //             break;
        //         std::this_thread::sleep_for(std::chrono::seconds(1));
        //     }
        // }
    }

    std::string MysqlUpdateCommand(std::string db_name, std::vector<std::string> values, std::vector<std::string> columns, std::vector<std::string> condition, std::vector<std::string> condition_name, std::vector<std::string> condition_symbols)
    {
        std::string command = "update " + db_name + " set ";
        for (int i = 0; i < values.size(); i++)
        {
            command += " " + columns[i] + " = \"" + values[i]+"\"";
            if(i!=values.size()-1)
            {
                command+=",";
            }
        }
        command += " where ";
        for (int i = 0; i < condition.size(); i++)
        {
            command += " " + condition_name[i] + " " + condition_symbols[i] + " " + condition[i] + " ";
        }
        return command;
    }

    virtual typename std::enable_if<std::is_same<typename T::Type, std::string>::value>::type
    // typename std::enable_if<std::is_same<typename GetContainerType<T>::Type, Worker2Params>::value>::type
    DealElement(ormpp::dbng<ormpp::mysql> &mysql, std::string &&s)
    {
        LOGGER->info("message #{}", s);

        s = "{\"status\":0,\"info\":\"请求成功\",\"data\":{\"enterprise_type\":1,\"record_url\":\"https:\\/\\/ks3-cn-beijing.ksyun.com\\/yimi-record-1-year\\/1000_00020a4f_918705532687_10001_10000000001_2_20220407_145607_145615_8_1344299.mp3?Expires=1649321794&response-content-type=audio%2Fmp3&KSSAccessKeyId=AKLT0ChC2dqZRK6viMM8BwGZLQ&Signature=7qezhD2bmtkGFWWxueSflOI2S%2BE%3D\",\"records\":[{\"call_type\":\"1\",\"call_state\":\"15\",\"cc_number\":\"1649314541655804conf_1649314555346\",\"dialing\":\"1000_00020a4f\",\"incoming\":\"918705532687\",\"confirm_timestamp\":\"1649314567\",\"start_time\":\"1649314555\",\"end_time\":\"1649314575\",\"duration_time\":\"8\",\"record_filename\":\"1000_00020a4f_918705532687_10001_10000000001_2_20220407_145607_145615_8.mp3\",\"record_status\":\"3\",\"res_token\":null,\"enqueue_timestamp\":\"0\",\"send_query_msg_timestamp\":\"0\",\"recv_answer_msg_timestamp\":\"0\",\"send_invite_timestamp\":\"1649314558\",\"static_time_index\":\"1649314800\",\"switch_number\":\"02566203172\",\"asr_state\":\"\",\"final_ring_time\":9,\"call_result\":1}]}}";

        CallRecord record;
        CallInfo callog = record.GetCallRecord(s, 2);
        cout << callog.call_state << endl;

        std::vector<std::string> columns = {"transfer_number", "transfer_duration"};
        std::vector<std::string> values = {"callog.transfer_number", std::to_string(callog.transfer_duration)};
        std::vector<std::string> condition(1);
        condition[0] = "301992";
        std::vector<std::string> condition_name(1);
        condition_name[0] = "id";
        std::vector<std::string> condition_symbols(1);
        condition_symbols[0] = " = ";

        std::string sql_command = MysqlUpdateCommand(" calllog ", values, columns, condition, condition_name, condition_symbols);
        LOGGER->info(" sql_command is ", sql_command);
        std::cout<<sql_command;
        if (mysql.execute(sql_command))
        {
            LOGGER->info(" update success ");
            std::cout<<"success "<<std::endl;
        }
        else
          {  LOGGER->info(" update failed ");
          std::cout<<"faile"<<std::endl;}

        // auto res = mysql.query<aicall_tts_file_cache>("id = 5622");
        // for(auto& file : res)
        //     std::cout<<file.id<<" "<<file.TTS_text<<" "<<file.TTS_version_code<<std::endl;

        // auto res = mysql.query<aicall_tts_file_cache>("id = 5622");
        // for(auto& file : res)
        //     std::cout<<file.id<<" "<<file.TTS_text<<" "<<file.TTS_version_code<<std::endl;
    }

    virtual typename std::enable_if<std::is_same<typename T::Type, std::string>::value>::type
    DealElement(std::string &&s)
    {
        return;
    }
};

template <class T>
void SetApiCallBackHandler(cinatra::http_server &server, T threadpool)
{
    server.set_http_handler<cinatra::GET, cinatra::POST>("/", [threadpool = threadpool](cinatra::request &req, cinatra::response &res)
                                                         {
        std::cout << req.body() << std::endl;
        threadpool->EnqueueStr(std::string(req.body()));
		res.set_status_and_content(cinatra::status_type::ok, "hello world"); });
}

int main()
{
    initspdlog();
    int max_thread_num = 1;
    cinatra::http_server server(max_thread_num);
    server.listen("0.0.0.0", "8080");

    using QueueType = std::conditional_t<true, LockQueue<std::string>, FreeLockRingQueue<std::string>>;
    auto queuetask = std::shared_ptr<QueueType>(new QueueType);
    std::shared_ptr<Worker<QueueType>> worker = std::make_shared<WorkerForHttp<QueueType>>(queuetask);
    std::shared_ptr<ThreadPool<QueueType>> threadpool(new ThreadPool(queuetask, worker, 2));

    SetApiCallBackHandler(server, threadpool);

    server.run();
    return 0;
}