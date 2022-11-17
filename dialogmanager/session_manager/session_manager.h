#pragma once

#include "global.h"
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
    bool ProcessSession(std::shared_ptr<Session> session, std::string_view content, std::chrono::system_clock::time_point question_time, 
                std::chrono::system_clock::time_point answer_time);
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