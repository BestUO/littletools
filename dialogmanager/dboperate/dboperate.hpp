#pragma once
#include "global.h"
#include "tools/jsonwrap.hpp"
#include <array>
#include <string>
#include <tuple>

template<size_t N>
std::array<std::string, N> split(const std::string& s, const char delimiter)
{
    size_t start = 0;
    size_t end = s.find_first_of(delimiter);
    std::array<std::string, N> output;

    size_t i = 0;
    while (end <= std::string::npos)
    {
        output[i++] = std::move(s.substr(start, end - start));
        if (end == std::string::npos)
            break;

        start = end + 1;
        end = s.find_first_of(delimiter, start);
    }

    return output;
}

template<size_t N, typename T>
static inline auto make(const std::array<std::string, N>&ar, size_t index, const T& args)
{
    return args;
}

template<size_t N, typename T, typename T1, typename... Args>
static inline auto make(const std::array<std::string, N>&ar, size_t index, T const & t, T1& first, Args&... args)
{
    return make(ar, index + 1, std::tuple_cat(t, std::make_tuple(std::pair<std::string, T1&>(ar[index], first))), args...);
}

#define VA_ARGS_NUM(...) std::tuple_size<decltype(std::make_tuple(__VA_ARGS__))>::value

#define META(...) auto meta(){\
    auto ar = split<VA_ARGS_NUM(__VA_ARGS__)>(#__VA_ARGS__, ',');\
    return make(ar, 0, std::tuple<>(), __VA_ARGS__);\
}

struct aia_course_practise_detail
{
    unsigned int course_id;
    unsigned int practise_id;
    std::string node_code;
    unsigned int question_id;
    unsigned int question_statement_id;
    unsigned int question_time;
    unsigned int answer_time;
    unsigned int status;
    std::string answer_txt;
    std::string answer_record_file;
    unsigned int answer_duration;
    std::string answer_match;
    std::string answer_analyse;
    META(course_id, practise_id, node_code, question_id, question_statement_id, question_time,
        answer_time, status, answer_txt, answer_record_file, answer_duration, answer_match, answer_analyse)
};


class DBOperate
{
public:
    static DBOperate *GetInstance()
    {
        static DBOperate instance;
        return &instance;
    }

    auto GetCourseInfo(unsigned int course_id)
    {
        using type = std::tuple<std::string, unsigned int>;
        auto fun = [&course_id](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = get_sql("select content, eid from aia_course where id=?",course_id);
            LOGGER->info(sql);
            return conn->query<type>(sql);
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GetQuestionDetail(unsigned int questiondetail_id)
    {
        using type = std::tuple<int, int, std::string, std::string, std::string, std::string, std::string,int,int>;
        auto fun = [&questiondetail_id](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql =  get_sql("select id,standard,similars,answer,keywords,prompt_txt,prompt_steps,perfect_tolerance,max_tolerance from aia_question where id=?",questiondetail_id);
            LOGGER->info(sql);
            return conn->query<type>(sql);
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GetTTSSoundCache(std::string ttsstatement_ids)
    {
        using type = std::tuple<int, std::string, std::string,int,int>;
        auto fun = [&ttsstatement_ids](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            // std::string sql = "select statement.id, statement.statement, soundcache.path, soundcache.sound_id, soundcache.tts_speed from aia_tts_statement as statement,aia_tts_sound_cache as soundcache where statement.id in ("+
            //     ttsstatement_ids + ") and statement.id = soundcache.statement_id";
            std::string sql = get_sql("select statement.id, statement.statement, soundcache.path, soundcache.sound_id, soundcache.tts_speed from aia_tts_statement as statement,aia_tts_sound_cache as soundcache where statement.id in (?) and statement.id = soundcache.statement_id",ttsstatement_ids);
            LOGGER->info(sql);
            return conn->query<type>(sql);
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GetUploadSoundfile(std::string ttsstatement_ids)
    {
        using type = std::tuple<int, std::string, std::string>;
        auto fun = [&ttsstatement_ids](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            // std::string sql = "select statement.id, statement.statement, file.path from aia_tts_statement as statement,aia_files as file where statement.id in ("+
            //     ttsstatement_ids + ") and statement.file_id = file.id";
            std::string sql = get_sql("select statement.id, statement.statement, file.path from aia_tts_statement as statement,aia_files as file where statement.id in (?) and statement.file_id = file.id",ttsstatement_ids);
            LOGGER->info(sql);
            return conn->query<type>(sql); 
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GetSoundInfo(int session_id)
    {
        using type = std::tuple<std::string,int>;
        auto fun = [&session_id](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = get_sql("select sound,start_time from aia_course_practise where id = ?",session_id);
            LOGGER->info(sql);
            return conn->query<type>(sql); 
        };
        return ExecuteCommand(std::function(fun));
    }

    auto CoursePratiseUpdate(int duration, int end_time, int session_id, int status)
    {
        auto fun = [&session_id,duration=duration,end_time=end_time,status=status](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = get_sql("update aia_course_practise set duration=?,end_time=?,status=? where id=?",duration,end_time,status,session_id);
            LOGGER->info(sql);
            return conn->execute(sql);
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GetNodeTypeAnswerPromptTxt(unsigned int course_id, std::string node_code)
    {
        using type = std::tuple<int, std::string, std::string>;
        auto fun = [&course_id, &node_code](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = get_sql("select node_type,node_txt,prompt_txt from aia_course_node where course_id = ? and node_code = ?",course_id,node_code);
            LOGGER->info(sql);
            return conn->query<type>(sql);
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GetEidOfCourse(unsigned int course_id) {
        using type = std::tuple<unsigned int>;
        auto fun = [&course_id](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = get_sql("select eid from aia_course where id = ?",course_id);
            LOGGER->info(sql);
            return conn->query<type>(sql);
        };
        return ExecuteCommand(std::function(fun));
    }
    auto GetParentIdOfWordData(unsigned int eid) {
        using type = std::tuple<unsigned int>;
        auto fun = [&eid](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = get_sql("select id from aia_word_data where eid = ? and parent_id = 0",eid);
            LOGGER->info(sql);
            return conn->query<type>(sql);
        };
        return ExecuteCommand(std::function(fun));
    }

    auto GetEnterpriseDirtyWords(unsigned int eid, unsigned int parent_id) {
        using type = std::tuple<std::string>;
        auto fun = [&eid, &parent_id](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            std::string sql = get_sql("select content from aia_word_data where eid = ? and parent_id = ?",eid,parent_id);
            LOGGER->info(sql);
            return conn->query<type>(sql);
        };
        return ExecuteCommand(std::function(fun));
    }

    template<typename... Args> 
    auto DBInsert(std::string tablename, std::tuple<Args...> kvs)
    {
        std::string sql = std::apply([&tablename](auto &&... args) 
        {
            std::string key = ((args.first+",")+...);
            if(!key.empty())
                key.pop_back();

            std::string value = ((to_str(args.second)+",")+...);
            if(!value.empty())
                value.pop_back();

            return "insert into " + tablename + "(" + key + ")" + " values(" + value + ")";
        },kvs);

        auto fun = [&sql=sql](std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)
        {
            LOGGER->info(sql);
            return conn->execute(sql);
        };
        return ExecuteCommand(std::function(fun));
    }
private:
    struct DBInfo
    {
        int pool_size;
        std::string mysql_host;
        std::string mysql_user;
        std::string mysql_password;
        std::string mysql_db;
        int mysql_timeout;
        int mysql_port;
    };
    DBInfo __dbinfo;
    DBOperate()
    {
        auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json");
        InitConnectionPool(config.value());
    }
    ~DBOperate()=default;

    template<class R>
    R ExecuteCommand(std::function<R(std::shared_ptr<ormpp::dbng<ormpp::mysql>> conn)> f)
    {
        auto &pool = ormpp::connection_pool<ormpp::dbng<ormpp::mysql>>::instance();
        auto conn = pool.get();
        while(!conn)
        {
            LOGGER->info("connection_pool is not enough");
            pool.return_back(conn);
            conn = pool.get();
        }

        auto result = std::move(f(conn));
        pool.return_back(conn);

        return result;
    }

    void CompleteDBInfo(rapidjson::Document &config)
    {
        __dbinfo.pool_size = config["mysql_setting"]["mysql_connect_pool_size"].GetInt();
        __dbinfo.mysql_host = config["mysql_setting"]["mysql_host"].GetString();
        __dbinfo.mysql_user = config["mysql_setting"]["mysql_user"].GetString();
        __dbinfo.mysql_password = config["mysql_setting"]["mysql_password"].GetString();
        __dbinfo.mysql_db = config["mysql_setting"]["mysql_db"].GetString();
        __dbinfo.mysql_timeout = config["mysql_setting"]["mysql_timeout"].GetInt();
        __dbinfo.mysql_port = config["mysql_setting"]["mysql_port"].GetInt();
    }

    bool InitConnectionPool(rapidjson::Document &config)
    {
        auto &pool = ormpp::connection_pool<ormpp::dbng<ormpp::mysql>>::instance();
        try
        {
            CompleteDBInfo(config);
            pool.init(__dbinfo.pool_size, __dbinfo.mysql_host.c_str(), __dbinfo.mysql_user.c_str(),
                        __dbinfo.mysql_password.c_str(), __dbinfo.mysql_db.c_str(), __dbinfo.mysql_timeout, __dbinfo.mysql_port);
        } 
        catch (const std::exception &e) 
        {
            LOGGER->info("InitConnectionPool err so return {}",e.what());
            return false;
        }
        return true;
    }
};