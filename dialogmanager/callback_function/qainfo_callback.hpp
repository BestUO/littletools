#pragma once
#include "global.h"
#include "score_manager/score_manager.h"

class QAInfoCallBackFunction
{
public:
    static void StoreInDB(std::shared_ptr<QAInfo> qainfo)
    {
        auto scoremanager = ScoreManager::GetInstance();
        auto question_detail = qainfo->question_detail.lock();
        auto current_node = qainfo->current_node.lock();
        unsigned int course_id=qainfo->course_id;
        unsigned int practise_id=qainfo->session_id;
        unsigned int node_code=current_node->node_id;
        unsigned int question_id=question_detail?question_detail->question_id:0;
        unsigned int question_statement_id=qainfo->tts_statement.tts_statement_id;
        unsigned int question_time=std::chrono::system_clock::to_time_t(qainfo->question_time);
        unsigned int answer_time=std::chrono::system_clock::to_time_t(qainfo->answer_time);
        unsigned int status=qainfo->isexpired;
        std::string answer_txt = qainfo->answer_txt.empty()?scoremanager->GetASRResult(qainfo->answer_audio_path):qainfo->answer_txt;
        std::string answer_record_file = qainfo->answer_audio_path.empty()?qainfo->answer_audio_path:"";
        unsigned int answer_duration=answer_time - question_time;
        std::string answer_match = "";
        std::string answer_analyse = scoremanager->GetAnswerAnalyse();
        unsigned int create_time = time(nullptr);
        unsigned int update_time = create_time;
        
        //打分、数据落盘
        LOGGER->info("QAInfo落盘 course_id:{} session_id:{} node_id:{}", 
                    qainfo->course_id, qainfo->session_id, current_node->node_id);
    }
};