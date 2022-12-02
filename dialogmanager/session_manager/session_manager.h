#pragma once

#include "global.h"
#include "dmthreadpool.hpp"
#include "callback_function/qainfo_callback.hpp"
#include <map>
#include <shared_mutex>
#include <optional>
#include <filesystem>

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
        auto [session_id,course_id,question_time,answer_time,is_expired,content] = params;
        std::string asr_result;
        int score = 0;
        if(session->current_qa)
        {
            session->current_qa->course_id = session->course_info->course_id;
            session->current_qa->session_id = session->session_id;
            session->current_qa->question_time = std::chrono::system_clock::from_time_t(question_time);
            session->current_qa->answer_time = std::chrono::system_clock::from_time_t(answer_time);
            session->current_qa->is_expired = is_expired;
            auto tmppath = __fileprefix;
            tmppath.append(content);
            if(std::filesystem::exists(tmppath))
                session->current_qa->answer_audio_path = tmppath;
            else
                session->current_qa->answer_txt = content;
            auto result = DMThreadPool::GetInstance()->GetThreadPool()->EnqueueFun(QAInfoCallBackFunction::StoreInDB,session->current_qa);
            std::tie(asr_result,score) = result.get();
        }
        session->current_qa = std::make_shared<QAInfo>();
        session->current_qa->last_asr_result = std::move(asr_result);
        session->current_qa->last_score = std::move(score);
        return CompleteQAInfo(session);
    }
    void DeleteSession(unsigned int session_id);
    void StopSessionManager();

private:
    std::map<unsigned int,std::shared_ptr<Session>> __session_map;
    std::shared_mutex __rwlock;
    std::filesystem::path __fileprefix;

    SessionManager();
    ~SessionManager()=default;

    std::optional<std::shared_ptr<Session>> GetFromSessionMap(unsigned int session_id);
    std::shared_ptr<Session> CreateSessionInsertToMap(unsigned int session_id, unsigned int course_id);
    std::tuple<std::vector<std::weak_ptr<QuestionDetail>>, std::string> GetTotalQuestionDetail(std::weak_ptr<Node> node);
    TTSStatement GetTTSStatement(std::shared_ptr<QuestionDetail> question_detail,int ttssound, int ttsspeed);
    bool CompleteQAInfo(std::shared_ptr<Session> session);
    std::optional<std::weak_ptr<Node>> GetNextNode(std::weak_ptr<Node> node);
    bool NodeHaveQuestionsLeft(std::shared_ptr<Session> session);
    void InsertToTimerManager(std::shared_ptr<Session> session);
    void DeleteSessionMap(unsigned int session_id);
    auto GetSoundAndSpeed(unsigned int session_id);
    std::vector<TTSStatement> FilterTTSStatement(std::vector<TTSStatement> ttsstatement,int ttssound, int ttsspeed);
};