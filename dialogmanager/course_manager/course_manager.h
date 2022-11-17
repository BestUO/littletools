#pragma once
#include "global.h"
#include "rapidjson/document.h"
#include "tools/lrucache.hpp"
#include <map>
#include <shared_mutex>
#include <optional>

class CourseManager
{
public:
    static CourseManager *GetInstance()
    {
        static CourseManager instance;
        return &instance;
    }
    std::shared_ptr<CourseInfo> GetCourse(unsigned int course_id);
    void DeleteCourse(unsigned int course_id);
private:
    LRUCache<unsigned int, std::shared_ptr<CourseInfo>> __lrucache;
    CourseManager();
    ~CourseManager()=default;

    std::optional<std::shared_ptr<CourseInfo>> GetFromCourseMap(unsigned int course_id);
    std::shared_ptr<CourseInfo> CreateCourseInsertToMap(unsigned int course_id);
    std::shared_ptr<CourseInfo> CreateCourse(unsigned int course_id);

    Question NodeParseQuestion(const rapidjson::Value & value, 
                std::map<unsigned int,std::shared_ptr<QuestionDetail>> &question_detail_map);
    std::vector<unsigned int> NodeParseLabel(const rapidjson::Value & value);
    std::vector<std::weak_ptr<Node>> NodeParseChildNode(const rapidjson::Value &value, std::map<unsigned int,std::shared_ptr<Node>> &node_map, std::weak_ptr<Node> parent);
    std::vector<std::weak_ptr<Node>> NodeParseJumpNode(const rapidjson::Value &value, std::map<unsigned int,std::shared_ptr<Node>> &node_map, std::weak_ptr<Node> parent);
    std::shared_ptr<QuestionDetail> CreateQuestionDetail(unsigned int questiondetail_id);
    std::vector<TTSStatement> GetTTSStatementInfo(unsigned int standard,std::string similars);
    std::weak_ptr<Node> GetCourseRoot(std::shared_ptr<Node> node);
    std::shared_ptr<CourseInfo> CreateCourse(unsigned int course_id, std::map<unsigned int,std::shared_ptr<Node>> &node_map, 
                std::map<unsigned int,std::shared_ptr<QuestionDetail>> &question_detail_map);
};