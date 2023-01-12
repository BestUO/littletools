#include "session_manager.h"
#include "course_manager/course_manager.h"
#include "tools/timermanager.hpp"
#include <random>

SessionManager::SessionManager()
{
    auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json").value();
    __fileprefix = config["fileprefix"].GetString();
    __cburl = config["cburl"].GetString();
}

std::shared_ptr<Session> SessionManager::GetSession(unsigned int session_id, unsigned int course_id)
{
    std::shared_ptr<Session> session= nullptr;
    auto opt = GetFromSessionMap(session_id);
    if(opt != std::nullopt)
        session = opt.value();
    else
        session = CreateSessionInsertToMap(session_id, course_id);
    if(session)
        InsertToTimerManager(session);
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

auto SessionManager::GetSoundAndSpeedAndStartTime(unsigned int session_id)
{
    auto [sound,start_time] = DBOperate::GetInstance()->GetSoundInfo(session_id).front();
    //sound:{"soundId":1,"ttsSpeed":50}
    auto tmp = JsonSimpleWrap::GetPaser(sound);
    return std::tuple(tmp.value()["soundId"].GetInt (),tmp.value()["ttsSpeed"].GetInt(),start_time);
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
        std::tie(session->ttssound,session->ttsspeed,session->start_time) = GetSoundAndSpeedAndStartTime(session_id);
        session->left_questions = GetTotalQuestionDetail(session->current_node);
        session->current_qa = nullptr;
        session->dirty_words = GetDirtyWords(courseinfo->eid);
        session->cburl = __cburl;
        
        std::unique_lock<std::shared_mutex> lock(__rwlock);
        __session_map[session_id] = session;
        return session;
    }
}

std::vector<std::weak_ptr<QuestionDetail>> SessionManager::GetTotalQuestionDetail(std::weak_ptr<Node> node)
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
    return tmp;
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
            session->left_questions = GetTotalQuestionDetail(session->current_node);
            NodeHaveQuestionsLeft(session);
        }
    }
    return true;
}

bool SessionManager::NodeHaveQuestionsLeft(std::shared_ptr<Session> session)
{
    bool flag = false;
    if(!session->left_questions.empty())
    {
        auto question_detail = session->left_questions.back().lock();
        session->left_questions.pop_back();

        session->current_qa->question_detail = question_detail;
        session->current_qa->tts_statement = GetTTSStatement(question_detail,session->ttssound, session->ttsspeed);
        session->current_qa->answer_stander = question_detail->answer;

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

std::vector<TTSStatement> SessionManager::FilterTTSStatement(std::vector<TTSStatement> ttsstatement,int ttssound, int ttsspeed)
{
    for (std::vector<TTSStatement>::iterator it = ttsstatement.begin(); it != ttsstatement.end();)
    {
        if (((*it).ttssound == 0 && (*it).ttsspeed == 0) || ((*it).ttssound == ttssound && (*it).ttsspeed == ttsspeed))
            ++it;
        else
            it = ttsstatement.erase(it);
    }
    return ttsstatement;
}

TTSStatement SessionManager::GetTTSStatement(std::shared_ptr<QuestionDetail> question_detail,int ttssound, int ttsspeed)
{
    auto tmp = std::move(FilterTTSStatement(question_detail->ttsstatement,ttssound, ttsspeed));
    if(tmp.empty())
        return TTSStatement();
    else
        return tmp[std::random_device()()%tmp.size()];
}

void SessionManager::InsertToTimerManager(std::shared_ptr<Session> session)
{
    auto timermanager = TimerManager<unsigned int>::GetInstance();
    timermanager->DeleteAlarm(session->session_id);
    timermanager->AddAlarm(std::chrono::system_clock::now() + std::chrono::minutes(5), session->session_id, [this,key=session->session_id]()
    {
        DeleteSessionMap(key,2);
    });
}

void SessionManager::DeleteSession(unsigned int session_id)
{
    auto timermanager = TimerManager<unsigned int>::GetInstance();
    timermanager->DeleteAlarm(session_id);
    DeleteSessionMap(session_id,1);
}

void SessionManager::DeleteSessionMap(unsigned int session_id, int status)
{
    std::unique_lock<std::shared_mutex> lock(__rwlock);
    __session_map[session_id]->status = status;
    __session_map.erase(session_id);
}

void SessionManager::StopSessionManager()
{
    TimerManager<unsigned int>::GetInstance()->StopTimerManager();
    DMThreadPool::GetInstance()->StopThreadPool();
}

std::vector<std::string> SessionManager::GetDirtyWords(unsigned int eid) {
    auto ret = std::vector<std::string>();
    auto parent_ids = DBOperate::GetInstance()->GetParentIdOfWordData(eid);
    if (parent_ids.size() != 1) {
        LOGGER->error("Error parent_id of eid:{}", eid);
        return ret;
    }
    auto parent_id = std::get<0>(parent_ids[0]);
    auto dirty_words = DBOperate::GetInstance()->GetEnterpriseDirtyWords(eid, parent_id);
    for (auto&& [w] : dirty_words) {
        ret.push_back(w);
    }
    return ret;
}
