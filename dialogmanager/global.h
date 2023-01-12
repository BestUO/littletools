#pragma once

#include "log.h"
#include "ormpp/mysql.hpp"
#include "ormpp/connection_pool.hpp"
#include "ormpp/dbng.hpp"
#include <filesystem>

struct TTSStatement
{
    unsigned int tts_statement_id=0;
    std::string question="";
    std::string audio_path="";
    int ttssound = 0;
    int ttsspeed = 0;
};

struct QuestionDetail
{
    unsigned int question_id;
    std::vector<TTSStatement> ttsstatement;
    std::string answer;
    std::string keywords;
    std::string prompt_txt;
    std::string prompt_steps;
    int perfect_tolerance;
    int max_tolerance;
};

struct Question
{
    std::vector<std::weak_ptr<QuestionDetail>> array;
    unsigned int order;
};

struct Node   
{
    std::string node_code;
    Question question;
    std::vector<unsigned int> label;
    std::vector<std::weak_ptr<Node>> childs;
    std::vector<std::weak_ptr<Node>> parents;
};

struct CourseInfo
{
    std::map<std::string,std::shared_ptr<Node>> node_map;
    std::map<unsigned int,std::shared_ptr<QuestionDetail>> question_detail_map;
    std::weak_ptr<Node> root;
    unsigned int course_id;
    unsigned int eid;
};

struct QAInfo
{
    unsigned int course_id;
    unsigned int session_id;
    unsigned int question_time;
    unsigned int answer_time;
    unsigned int answer_duration;
    unsigned int is_expired;
    std::string answer_txt;
    std::filesystem::path relative_path;
    std::filesystem::path answer_audio_path;
    std::string answer_analyse;

    std::weak_ptr<Node> current_node;
    std::weak_ptr<QuestionDetail> question_detail;
    TTSStatement tts_statement;
    std::string answer_stander;
    std::string last_asr_result;
    int last_score;
};

struct Session
{
    unsigned int session_id=0;
    int ttssound = 0;
    int ttsspeed = 0;
    int start_time = 0;
    std::chrono::system_clock::time_point create_time = std::chrono::system_clock::now();
    std::shared_ptr<CourseInfo> course_info = nullptr;
    std::weak_ptr<Node> current_node = std::weak_ptr<Node>();
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<std::weak_ptr<QuestionDetail>> left_questions;                              //当前节点待提问的问题
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    std::shared_ptr<QAInfo> current_qa = nullptr;
    std::vector<std::string> dirty_words;
    std::string cburl="";
    int status=1;//1正常结束流程;2异常流程终止比如超时

    Session() = default;
    Session(unsigned int session_id,std::shared_ptr<CourseInfo> course_info):session_id(session_id),course_info(course_info){};
    ~Session();
};

template <typename T> inline constexpr auto to_str(T &&t) {
  if constexpr (std::is_arithmetic_v<std::decay_t<T>>) {
    return std::to_string(std::forward<T>(t));
  } else {
    return std::string("'") + t + std::string("'");
  }
}

inline void get_str(std::string &sql, const std::string &s) {
  auto pos = sql.find_first_of('?');
  sql.replace(pos, 1, " ");
  sql.insert(pos, s);
}

template <typename... Args>
inline std::string get_sql(const std::string &o, Args &&...args) {
  auto sql = o;
  std::string s = "";
  (get_str(sql, to_str(args)), ...);

  return sql;
}