#include "course_manager.h"
#include "dboperate/dboperate.hpp"

CourseManager::CourseManager()
{
    auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json").value();
    __lrucache.SetMaxSizeAndRemainDay(config["lru_cache_course"]["size"].GetInt(),config["lru_cache_course"]["remain_days"].GetInt());
};

std::shared_ptr<CourseInfo> CourseManager::GetCourse(unsigned int course_id)
{
    std::shared_ptr<CourseInfo> CourseInfo= nullptr;
    auto opt = GetFromCourseMap(course_id);
    if(opt != std::nullopt)
        CourseInfo = opt.value();
    else
        CourseInfo = CreateCourseInsertToMap(course_id);
    return CourseInfo;
}

std::optional<std::shared_ptr<CourseInfo>> CourseManager::GetFromCourseMap(unsigned int course_id)
{
    return __lrucache.GetValue(course_id);
}

std::shared_ptr<CourseInfo> CourseManager::CreateCourseInsertToMap(unsigned int course_id)
{
    LOGGER->info("cant find course {} so create", course_id);
    auto courseinfo = CreateCourse(course_id);
    if(courseinfo != nullptr)
        __lrucache.StoreKeyValue(course_id,courseinfo);
    return courseinfo;
}

std::shared_ptr<CourseInfo> CourseManager::CreateCourse(unsigned int course_id)
{
    /*
    {
        "node_array":
        [
            {
                "id":1,
                "node_text":"独白节点",
                "question":
                {
                    "question_detail_array":[1],
                    "order":2
                },
                "label":[1],
                "child_nodes":[2,3],
                "jump_nodes":[]
            },
            {
                "id":2,
                "node_text":"",
                "question":
                {
                    "question_detail_array":[1,2],
                    "order":1
                },
                "label":[1,2,3],
                "child_nodes":[],
                "jump_nodes":[]
            },
            {
                "id":3,
                "node_text":"",
                "question":
                {
                    "question_detail_array":[1,2],
                    "order":1
                },
                "label":[1,2],
                "child_nodes":[],
                "jump_nodes":[]
            }
        ]
    }
    */
    std::string content = std::move(DBOperate::GetInstance()->GetCourseInfo(course_id));
    rapidjson::Document parser;
    if (parser.Parse(content.data()).HasParseError())
    {
        LOGGER->info("parse course {} content err",course_id);
        return nullptr;
    }
    else
    {
        try
        {
            std::map<unsigned int,std::shared_ptr<Node>> node_map;
            std::map<unsigned int,std::shared_ptr<QuestionDetail>> question_detail_map;
            const rapidjson::Value& node_array = parser["node_array"];
            for(auto &node:node_array.GetArray())
            {
                std::shared_ptr<Node> nodeptr = nullptr;
                if(node_map.find(node["id"].GetUint()) != node_map.end())
                    nodeptr = node_map[node["id"].GetUint()];
                else
                {
                    nodeptr = std::make_shared<Node>();
                    node_map[node["id"].GetUint()] = nodeptr;
                }

                nodeptr->node_id = node["id"].GetUint();
                nodeptr->node_text = node["node_text"].GetString();
                nodeptr->question = std::move(NodeParseQuestion(node["question"],question_detail_map));
                nodeptr->label = std::move(NodeParseLabel(node["label"]));
                nodeptr->childs = std::move(NodeParseChildNode(node["child_nodes"], node_map, nodeptr));
                auto jump_nodes = std::move(NodeParseJumpNode(node["jump_nodes"], node_map, nodeptr));
                nodeptr->childs.insert(nodeptr->childs.end(), jump_nodes.begin(), jump_nodes.end());
            }
            return CreateCourse(course_id, node_map, question_detail_map);
        }
        catch(const std::exception& e)
        {
            LOGGER->info("parse course {} content err",course_id);
            return nullptr;
        }
    }
}

std::shared_ptr<CourseInfo> CourseManager::CreateCourse(unsigned int course_id, std::map<unsigned int,std::shared_ptr<Node>> &node_map, 
                    std::map<unsigned int,std::shared_ptr<QuestionDetail>> &question_detail_map)
{
    auto courseptr = std::make_shared<CourseInfo>();
    courseptr->course_id = course_id;
    courseptr->node_map = std::move(node_map);
    courseptr->question_detail_map = std::move(question_detail_map);
    courseptr->root = GetCourseRoot(courseptr->node_map.begin()->second);
    return courseptr;
}

std::weak_ptr<Node> CourseManager::GetCourseRoot(std::shared_ptr<Node> node)
{
    std::shared_ptr<Node> root = node;
    while(!root->parents.empty())
        root = root->parents[0].lock();
    return root;
}

Question CourseManager::NodeParseQuestion(const rapidjson::Value & value, 
                            std::map<unsigned int,std::shared_ptr<QuestionDetail>> &question_detail_map)
{
    /*
    "question":
    {
        "question_detail_array":[1,2],
        "order":2
    },
    */
    Question tmp;
    tmp.order = value["order"].GetUint();
    for(auto &item:value["question_detail_array"].GetArray())
    {
        std::shared_ptr<QuestionDetail> questiondetail = nullptr;
        if(question_detail_map.find(item.GetUint()) == question_detail_map.end())
            question_detail_map[item.GetUint()] = CreateQuestionDetail(item.GetUint());

        tmp.array.emplace_back(question_detail_map[item.GetUint()]);
    }
    return tmp;
}

std::shared_ptr<QuestionDetail> CourseManager::CreateQuestionDetail(unsigned int questiondetail_id)
{
    auto tmp = std::make_shared<QuestionDetail>();
    auto [id, standard, similars, answer, keywords,prompt_txt,prompt_steps] = DBOperate::GetInstance()->GetQuestionDetail(questiondetail_id);
    tmp->question_id = id;
    tmp->answer = std::move(answer);
    tmp->keywords = std::move(keywords);
    tmp->prompt_txt = std::move(prompt_txt);
    tmp->prompt_steps = std::move(prompt_steps);
    tmp->ttsstatement = std::move(GetTTSStatementInfo(standard,std::move(similars)));
    return tmp;
}

std::vector<TTSStatement> CourseManager::GetTTSStatementInfo(unsigned int standard,std::string similars)
{ 
    std::vector<TTSStatement> tmp;
    similars += (similars.empty()?"":",") + std::to_string(standard);
    for(auto& [id, statement, path]:DBOperate::GetInstance()->GetTTSStatement(similars))
        tmp.emplace_back(TTSStatement{(unsigned int)id, statement, path});
    return tmp;
}

std::vector<unsigned int> CourseManager::NodeParseLabel(const rapidjson::Value & value)
{
    /*"label":[1,2,3],*/
    std::vector<unsigned int> tmp;
    for(auto &item:value.GetArray())
        tmp.emplace_back(item.GetUint());
    return tmp;
}

std::vector<std::weak_ptr<Node>> CourseManager::NodeParseChildNode(const rapidjson::Value &value,std::map<unsigned int,std::shared_ptr<Node>> &node_map, std::weak_ptr<Node> parent)
{
    /*"child_nodes":[2,3]*/
    std::vector<std::weak_ptr<Node>> tmp;
    for(auto &item:value.GetArray())
    {
        std::shared_ptr<Node> nodeptr = nullptr;
        if(node_map.find(item.GetUint()) == node_map.end())
        {
            nodeptr = std::make_shared<Node>();
            nodeptr->parents.emplace_back(parent);
            node_map[item.GetUint()] = nodeptr;
        }
        tmp.emplace_back(node_map[item.GetUint()]);
    }
    return tmp;
}

std::vector<std::weak_ptr<Node>> CourseManager::NodeParseJumpNode(const rapidjson::Value &value,std::map<unsigned int,std::shared_ptr<Node>> &node_map, std::weak_ptr<Node> parent)
{
    return NodeParseChildNode(value, node_map, parent);
}

void CourseManager::DeleteCourse(unsigned int course_id)
{
    __lrucache.DeleteKey(course_id);
}

