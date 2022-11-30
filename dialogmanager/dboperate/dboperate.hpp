#include "global.h"
#include "tools/jsonwrap.hpp"
// struct AIACourse
// {
//     unsigned int id;
//     unsigned int eid;
//     std::string title;
//     std::string tts_sounds;
//     unsigned int tts_speed;
//     std::string content;
//     std::string intro;
//     unsigned int cover;
//     unsigned int category_id;
//     unsigned int user_id;
//     unsigned int advisor_user_id;
//     std::string score_scale;
//     unsigned int status;
//     unsigned int online_offline_time;
//     unsigned int create_time;
//     unsigned int update_time;
// };
// REFLECTION(AIACourse,id,eid,title,tts_sounds,tts_speed,content,intro,cover,category_id,user_id,advisor_user_id,score_scale,status,online_offline_time,create_time,update_time)

// struct aicall_tts_file_cache
// {
// 	int id;
// 	std::string TTS_text;
// 	int TTS_version_code;
// 	std::string tts_src;
// 	int tts_duration;
// 	int create_time;
// 	int access_time;
// 	int extension;
// };
// REFLECTION(aicall_tts_file_cache, id, TTS_text, TTS_version_code, tts_src, tts_duration, create_time, access_time, extension)

struct aia_course_practise
{
    unsigned int id;
    unsigned int course_id;
    unsigned int practise_id;
    unsigned int node_id;
    unsigned int question_id;
    unsigned int question_statement_id;
    unsigned int question_time;
    unsigned int answer_time;
    unsigned int status;//todo
    std::string answer_text;
    std::string answer_record_file;
    unsigned int answer_duration;
    std::string answer_match;
    std::string answer_analyse;
    unsigned int create_time;
    unsigned int update_time;
};
REFLECTION(aia_course_practise, id, course_id, practise_id, node_id)
//REFLECTION(aia_course_practise, id, course_id, practise_id, node_id, question_id, question_statement_id, question_time, answer_time)


class DBOperate
{
public:
    static DBOperate *GetInstance()
    {
        static DBOperate instance;
        return &instance;
    }

    std::string GetCourseInfo(unsigned int course_id)
    {
        auto fun = [&course_id](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            auto result = std::move(conn->query<std::tuple<std::string>>("select content from aia_course where id=?",course_id)); 
            return result;
        };
        auto content = std::move(ExecuteCommand(std::function(fun)));

        if(content.empty())
        {
            LOGGER->info("course_id {} is empty",course_id);
            return "";
        }
        else
            return std::get<0>(content.front());
    }

    auto GetQuestionDetail(unsigned int questiondetail_id)
    {
        using type = std::tuple<int, int, std::string, std::string, std::string, std::string, std::string,int,int>;
        auto fun = [&questiondetail_id](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            auto result = std::move(conn->query<type>
                            ("select id,standard,similars,answer,keywords,prompt_txt,prompt_steps,perfect_tolerance,max_tolerance from aia_question where id=?",questiondetail_id)); 
            return result;
        };
        auto content = std::move(ExecuteCommand(std::function(fun)));

        if(content.empty())
        {
            LOGGER->info("questiondetail_id {} is empty",questiondetail_id);
            return type();
        }
        else
            return content.front();
    }

    auto GetTTSStatement(std::string ttsstatement_ids)
    {
        using type = std::tuple<int, std::string, std::string>;
        auto fun = [&ttsstatement_ids](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = "select statement.id, statement.statement, soundcache.path from aia_tts_statement as statement,aia_tts_sound_cache as soundcache where statement.id in ("+
                ttsstatement_ids + ") and statement.file_id = soundcache.id";
            auto result = std::move(conn->query<type>(sql)); 
            return result;
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GenerateCoursePractiseDetail(const aia_course_practise &detail) {
        using type = std::tuple<int, std::string, std::string>;
        auto fun = [&detail](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn) {
            return conn->insert(detail);
        };
        return ExecuteCommand(std::function(fun));
    }

private:
    DBOperate()
    {
        auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json");
        InitConnectionPool(config.value());
    }
    ~DBOperate()=default;

    template<class ...Args>
    std::vector<std::tuple<Args...>> ExecuteCommand(std::function<std::vector<std::tuple<Args...>>(std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)> f)
    {
        auto &pool = ormpp::connection_pool<ormpp::dbng<ormpp::mysql>>::instance();
        auto conn = pool.get();
        while(!conn)
        {
            LOGGER->info("connection_pool is not enough");
            conn = pool.get();
        }
        auto result = std::move(f(conn));
        pool.return_back(conn);
        return result;
    }

    int ExecuteCommand(const std::function<int (std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)>& f)
    {
        auto &pool = ormpp::connection_pool<ormpp::dbng<ormpp::mysql>>::instance();
        auto conn = pool.get();
        while(!conn)
        {
            LOGGER->info("connection_pool is not enough");
            conn = pool.get();
        }
        auto result = f(conn);
        pool.return_back(conn);
        return result;
    }

    bool InitConnectionPool(rapidjson::Document &config)
    {
        auto &pool = ormpp::connection_pool<ormpp::dbng<ormpp::mysql>>::instance();
        try
        {
            pool.init(config["mysql_setting"]["mysql_connect_pool_size"].GetInt(),
                        config["mysql_setting"]["mysql_host"].GetString(),
                        config["mysql_setting"]["mysql_user"].GetString(),
                        config["mysql_setting"]["mysql_password"].GetString(),
                        config["mysql_setting"]["mysql_db"].GetString(),
                        config["mysql_setting"]["mysql_timeout"].GetInt(),
                        config["mysql_setting"]["mysql_port"].GetInt());
        } 
        catch (const std::exception &e) 
        {
            LOGGER->info("InitConnectionPool err so return {}",e.what());
            return false;
        }
        return true;
    }
};