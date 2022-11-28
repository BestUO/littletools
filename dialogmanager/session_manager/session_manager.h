#pragma once

#include "global.h"
#include "dmthreadpool.hpp"
#include "callback_function/qainfo_callback.hpp"
#include <map>
#include <shared_mutex>
#include <optional>

class SessionManager
{
public:
    static SessionManager *GetInstance()
    {
        static SessionManager instance;
        return &instance;
    }
    std::shared_ptr<Session> GetSession(unsigned int session_id, unsigned int course_id);
    template<class ...Args>
    bool ProcessSession(std::shared_ptr<Session> session, std::tuple<Args...> params)
    {
        auto [session_id,course_id,question_time,answer_time,isexpired,content] = params;
        if(session->current_qa)
        {
            session->current_qa->course_id = session->course_info->course_id;
            session->current_qa->session_id = session->session_id;
            session->current_qa->question_time = std::chrono::system_clock::from_time_t(question_time);
            session->current_qa->answer_time = std::chrono::system_clock::from_time_t(answer_time);
            session->current_qa->isexpired = isexpired;
            if(content.find("http") == 0)
                session->current_qa->answer_audio_path = content;
            else
                session->current_qa->answer_txt = content;
            auto result = DMThreadPool::GetInstance()->GetThreadPool()->EnqueueFun(QAInfoCallBackFunction::StoreInDB,session->current_qa);
            LOGGER->info(result.get());
        }
        session->current_qa = std::make_shared<QAInfo>();
        return CompleteQAInfo(session);
    }
    void DeleteSession(unsigned int session_id);
    void StopSessionManager();

private:
    std::map<unsigned int,std::shared_ptr<Session>> __session_map;
    std::shared_mutex __rwlock;

    SessionManager()=default;
    ~SessionManager()=default;

    std::optional<std::shared_ptr<Session>> GetFromSessionMap(unsigned int session_id);
    std::shared_ptr<Session> CreateSessionInsertToMap(unsigned int session_id, unsigned int course_id);
    std::tuple<std::vector<std::weak_ptr<QuestionDetail>>, std::string> GetTotalQuestionDetail(std::weak_ptr<Node> node);
    TTSStatement GetTTSStatement(std::shared_ptr<QuestionDetail> question_detail);
    bool CompleteQAInfo(std::shared_ptr<Session> session);
    std::optional<std::weak_ptr<Node>> GetNextNode(std::weak_ptr<Node> node);
    bool NodeHaveQuestionsLeft(std::shared_ptr<Session> session);
    void InsertToTimerManager(std::shared_ptr<Session> session);
    void DeleteSessionMap(unsigned int session_id);
};