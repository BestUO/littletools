#pragma once
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

struct aia_course_practise_detail
{
    unsigned int id;
    unsigned int course_id;
    unsigned int practise_id;
    std::string node_code;
    unsigned int question_id;
    unsigned int question_statement_id;
    uint64_t question_time;
    uint64_t answer_time;
    unsigned int status;//todo
    std::string answer_text;
    std::string answer_record_file;
    unsigned int answer_duration;
    std::string answer_match;
    std::string answer_analyse;
    unsigned int create_time;
    unsigned int update_time;
};
REFLECTION(aia_course_practise_detail, id, course_id, practise_id, node_code)
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

    auto GetTTSSoundCache(std::string ttsstatement_ids)
    {
        using type = std::tuple<int, std::string, std::string,int,int>;
        auto fun = [&ttsstatement_ids](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = "select statement.id, statement.statement, soundcache.path, soundcache.sound_id, soundcache.tts_speed from aia_tts_statement as statement,aia_tts_sound_cache as soundcache where statement.id in ("+
                ttsstatement_ids + ") and statement.id = soundcache.statement_id";
            auto result = std::move(conn->query<type>(sql)); 
            return result;
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GetUploadSoundfile(std::string ttsstatement_ids)
    {
        using type = std::tuple<int, std::string, std::string>;
        auto fun = [&ttsstatement_ids](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = "select statement.id, statement.statement, file.path from aia_tts_statement as statement,aia_files as file where statement.id in ("+
                ttsstatement_ids + ") and statement.file_id = file.id";
            auto result = std::move(conn->query<type>(sql)); 
            return result;
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GenerateCoursePractiseDetail(const aia_course_practise_detail &detail) {
        auto fun = [&detail](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn) {
            return conn->insert(detail);
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GetSoundInfo(int session_id)
    {
        using type = std::tuple<std::string,int>;
        auto fun = [&session_id](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = "select sound,start_time from aia_course_practise where id = ?";
            auto result = std::move(conn->query<type>(sql,session_id)); 
            return result;
        };
        return ExecuteCommand(std::function(fun));
    }

    auto CoursePratiseUpdate(int duration, int end_time, int session_id)
    {
        auto fun = [&session_id,duration=duration,end_time=end_time](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = "update aia_course_practise set duration = " + std::to_string(duration) + ",end_time=" + std::to_string(end_time) + " where id = " + std::to_string(session_id);
            auto result = std::move(conn->execute(sql));
            return result;
            // std::string sql = "update aia_course_practise set duration=?, end_time=? where id = ?";
            // auto result = std::move(conn->execute(sql,duration,end_time,session_id));
            // return result;
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GetNodeTypeAnswerPromptTxt(unsigned int course_id, std::string node_code)
    {
        using type = std::tuple<int, std::string, std::string>;
        auto fun = [&course_id, &node_code](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = "select node_type,node_txt,prompt_txt from aia_course_node where course_id = ? and node_code = ?";
            auto result = std::move(conn->query<type>(sql,course_id,node_code)); 
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

    // template<class ...Args>
    // std::vector<std::tuple<Args...>> ExecuteCommand(std::function<std::vector<std::tuple<Args...>>(std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)> f)
    // {
    //     auto &pool = ormpp::connection_pool<ormpp::dbng<ormpp::mysql>>::instance();
    //     auto conn = pool.get();
    //     while(!conn)
    //     {
    //         LOGGER->info("connection_pool is not enough");
    //         conn = pool.get();
    //     }
    //     auto result = std::move(f(conn));
    //     pool.return_back(conn);
    //     return result;
    // }

    template<class R>
    R ExecuteCommand(std::function<R(std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)> f)
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