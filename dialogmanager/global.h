#pragma once

#include "spdlog/spdlog.h"
#include "ormpp/mysql.hpp"
#include "ormpp/connection_pool.hpp"
#include "ormpp/dbng.hpp"
#include <filesystem>
#define SPDLOG_FILENAME "log/DialogerManagerLogger.log"
#define SPDLOGGERNAME "DialogerManagerLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

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
    unsigned int node_id;
    std::string node_text;
    Question question;
    std::vector<unsigned int> label;
    std::vector<std::weak_ptr<Node>> childs;
    std::vector<std::weak_ptr<Node>> parents;
};

struct CourseInfo
{
    std::map<unsigned int,std::shared_ptr<Node>> node_map;
    std::map<unsigned int,std::shared_ptr<QuestionDetail>> question_detail_map;
    std::weak_ptr<Node> root;
    unsigned int course_id;
};

struct QAInfo
{
    unsigned int course_id;
    unsigned int session_id;
    std::chrono::system_clock::time_point question_time;
    std::chrono::system_clock::time_point answer_time;
    unsigned int is_expired;
    std::string answer_txt;
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
    std::string node_text;                                                                  //当前节点独白节点答案
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    std::shared_ptr<QAInfo> current_qa = nullptr;
    Session() = default;
    Session(unsigned int session_id,std::shared_ptr<CourseInfo> course_info):session_id(session_id),course_info(course_info){};
    ~Session();
};
