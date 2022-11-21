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
        using type = std::tuple<int, int, std::string, std::string, std::string, std::string, std::string>;
        auto fun = [&questiondetail_id](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            auto result = std::move(conn->query<type>
                            ("select id,standard,similars,answer,keywords,prompt_txt,prompt_steps from aia_question where id=?",questiondetail_id)); 
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