#pragma once

#include "spdlog/spdlog.h"
#include "ormpp/mysql.hpp"
#include "ormpp/connection_pool.hpp"
#include "ormpp/dbng.hpp"

#define SPDLOG_FILENAME "log/DialogerManagerLogger.log"
#define SPDLOGGERNAME "DialogerManagerLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

struct TTSStatement
{
    unsigned int tts_statement_id=0;
    std::string question="";
    std::string audio_path="";
};

struct QuestionDetail
{
    unsigned int question_id;
    std::vector<TTSStatement> ttsstatement;
    std::string answer;
    std::string keywords;
    std::string prompt_txt;
    std::string prompt_steps;
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
    ~Node()
    {
        std::cout << "byd node" << std::endl;
    }
};

struct CourseInfo
{
    std::map<unsigned int,std::shared_ptr<Node>> node_map;
    std::map<unsigned int,std::shared_ptr<QuestionDetail>> question_detail_map;
    std::weak_ptr<Node> root;
    unsigned int course_id;
    ~CourseInfo()
    {
        std::cout << "byd CourseInfo" << std::endl;
    }
};

struct QAInfo
{
    unsigned int course_id;
    unsigned int session_id;
    std::chrono::system_clock::time_point question_time;
    std::chrono::system_clock::time_point answer_time;
    std::string answer_txt;
    std::string answer_audio_path;
    std::string answer_analyse;

    std::weak_ptr<Node> current_node;
    std::weak_ptr<QuestionDetail> question_detail;
    TTSStatement tts_statement;
    std::string answer_stander;

    ~QAInfo()
    {std::cout << "byd QAInfo" << std::endl;}
};

struct Session
{
    unsigned int session_id=0;
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
    ~Session()
    {std::cout << "byd Session" << std::endl;}
};

/*error code
std::vector<std::string_view> splitSV(std::string_view strv, std::string_view delims = " ")
{
    std::vector<std::string_view> output;
    size_t first = 0;

    while (first < strv.size())
    {
        const auto second = strv.find_first_of(delims, first);

        if (first != second)
            output.emplace_back(strv.substr(first, second-first));

        if (second == std::string_view::npos)
            break;

        first = second + 1;
    }

    return output;
}
*/

std::vector<std::string>
split(const std::string& str, const std::string& delims = " ")
{
    std::vector<std::string> output;
    auto first = std::cbegin(str);

    while (first != std::cend(str))
    {
        const auto second = std::find_first_of(first, std::cend(str), 
                  std::cbegin(delims), std::cend(delims));

        if (first != second)
            output.emplace_back(first, second);

        if (second == std::cend(str))
            break;

        first = std::next(second);
    }

    return output;
}

std::vector<std::string> splitString(std::string str, char sep = ',') {
    std::vector<std::string> vecString;
    std::string item;

    std::stringstream stringStream(str);

    while (std::getline(stringStream, item, sep))
    {
        vecString.push_back(item);
    }

    return vecString;
}