#include "session_manager.h"
#include "course_manager/course_manager.h"
#include "dmthreadpool.hpp"
#include "callback_function/qainfo_callback.hpp"
#include "tools/timermanager.hpp"
#include <random>

std::shared_ptr<Session> SessionManager::GetSession(unsigned int session_id, unsigned int course_id)
{
    std::shared_ptr<Session> session= nullptr;
    auto opt = GetFromSessionMap(session_id);
    if(opt != std::nullopt)
        session = opt.value();
    else
    {
        session = CreateSessionInsertToMap(session_id, course_id);
        InsertToTimerManager(session);
    }
    return session;
}

std::optional<std::shared_ptr<Session>> SessionManager::GetFromSessionMap(unsigned int session_id)
{
    std::shared_lock<std::shared_mutex> lock(__rwlock);
    if(__session_map.find(session_id) != __session_map.end())
        return __session_map[session_id];
    else
        return std::nullopt;
}

std::shared_ptr<Session> SessionManager::CreateSessionInsertToMap(unsigned int session_id, unsigned int course_id)
{
    LOGGER->info("cant find session {} so create", session_id);
    auto courseinfo = CourseManager::GetInstance()->GetCourse(course_id);
    if(courseinfo == nullptr)
        return nullptr;
    else
    {
        auto session = std::make_shared<Session>(session_id,courseinfo);
        session->current_node = courseinfo->root;
        std::tie(session->left_questions, session->node_text) = GetTotalQuestionDetail(session->current_node);
        session->current_qa = nullptr;

        std::unique_lock<std::shared_mutex> lock(__rwlock);
        __session_map[session_id] = session;
        return session;
    }
}

std::tuple<std::vector<std::weak_ptr<QuestionDetail>>, std::string> SessionManager::GetTotalQuestionDetail(std::weak_ptr<Node> node)
{
    auto nodeptr = node.lock();
    std::vector<std::weak_ptr<QuestionDetail>> tmp = nodeptr->question.array;
    if(nodeptr->question.order == 1) //随机提问
    {
        std::shuffle(tmp.begin(), tmp.end(), std::default_random_engine(std::random_device()()));
        unsigned int popnum = std::random_device()()%tmp.size();
        for(unsigned int i = 0;i < popnum;i++)
            tmp.pop_back();
    }
    return {tmp,nodeptr->node_text};
}

bool SessionManager::ProcessSession(std::shared_ptr<Session> session, std::string_view content, std::chrono::system_clock::time_point question_time, 
                                            std::chrono::system_clock::time_point answer_time)
{
    if(session->current_qa)
    {
        session->current_qa->course_id = session->course_info->course_id;
        session->current_qa->session_id = session->session_id;
        session->current_qa->question_time = question_time;
        session->current_qa->answer_time = answer_time;
        if(content.find("http") == 0)
            session->current_qa->answer_audio_path = content;
        else
            session->current_qa->answer_txt = content;
        DMThreadPool::GetInstance()->GetThreadPool()->EnqueueFun(QAInfoCallBackFunction::StoreInDB,session->current_qa);
    }
    session->current_qa = std::make_shared<QAInfo>();
    return CompleteQAInfo(session);
}

bool SessionManager::CompleteQAInfo(std::shared_ptr<Session> session)
{
    if(!NodeHaveQuestionsLeft(session))
    {
        //nextnode
        auto current_node = GetNextNode(session->current_node);
        if(current_node == std::nullopt)
            return false;
        else
        {
            session->current_node = current_node.value();
            std::tie(session->left_questions, session->node_text) = GetTotalQuestionDetail(session->current_node);
            NodeHaveQuestionsLeft(session);
        }
    }
    return true;
}

bool SessionManager::NodeHaveQuestionsLeft(std::shared_ptr<Session> session)
{
    bool flag = false;
    if(!session->node_text.empty() || !session->left_questions.empty())
    {
        if(!session->node_text.empty())
        {
            session->current_qa->answer_stander = session->node_text;
            session->node_text.clear();
        }
        else
        {
            auto question_detail = session->left_questions.back().lock();
            session->left_questions.pop_back();

            session->current_qa->question_detail = question_detail;
            session->current_qa->tts_statement = GetTTSStatement(question_detail);
            session->current_qa->answer_stander = question_detail->answer;
        }
        flag = true;
    }
    else
        flag = false;

    session->current_qa->current_node = session->current_node;
    return flag;
}

std::optional<std::weak_ptr<Node>> SessionManager::GetNextNode(std::weak_ptr<Node> node)
{
    auto nodeptr = node.lock();
    if(nodeptr->childs.empty())
        return std::nullopt;
    else
        return nodeptr->childs[std::random_device()()%nodeptr->childs.size()];
}

TTSStatement SessionManager::GetTTSStatement(std::shared_ptr<QuestionDetail> question_detail)
{
    return question_detail->ttsstatement[std::random_device()()%question_detail->ttsstatement.size()];
}

void SessionManager::InsertToTimerManager(std::shared_ptr<Session> session)
{
    auto timermanager = TimerManager<unsigned int>::GetInstance();
    timermanager->AddAlarm(std::chrono::system_clock::now() + std::chrono::minutes(30), session->session_id, [this,key=session->session_id]()
    {
        DeleteSessionMap(key);
    });
}

void SessionManager::DeleteSession(unsigned int session_id)
{
    auto timermanager = TimerManager<unsigned int>::GetInstance();
    timermanager->DeleteAlarm(session_id);
    DeleteSessionMap(session_id);
}

void SessionManager::DeleteSessionMap(unsigned int session_id)
{
    std::unique_lock<std::shared_mutex> lock(__rwlock);
    __session_map.erase(session_id);
}

void SessionManager::StopSessionManager()
{
    TimerManager<unsigned int>::GetInstance()->StopTimerManager();
    DMThreadPool::GetInstance()->StopThreadPool();
}