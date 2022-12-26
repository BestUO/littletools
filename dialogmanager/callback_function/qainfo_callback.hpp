#pragma once

#include "dboperate/dboperate.hpp"
#include "speech_service/SpeecService.h"
#include "global.h"
#include "score_manager/score_manager.h"


class QAInfoCallBackFunction
{
public:
    static std::tuple<std::string,int> StoreInDB(std::shared_ptr<QAInfo> qainfo) {
        auto scoremanager = ScoreManager::GetInstance();
        auto question_detail = qainfo->question_detail.lock();
        auto current_node = qainfo->current_node.lock();
        unsigned int course_id = qainfo->course_id;
        unsigned int practise_id = qainfo->session_id;
        std::string node_code = current_node->node_code;
        unsigned int question_id = question_detail ? question_detail->question_id : 0;
        unsigned int question_statement_id = qainfo->tts_statement.tts_statement_id;
        unsigned int question_time = std::chrono::system_clock::to_time_t(qainfo->question_time);
        unsigned int answer_time = std::chrono::system_clock::to_time_t(qainfo->answer_time);
        unsigned int status = qainfo->is_expired;
        std::string answer_txt = qainfo->answer_txt;
        if (qainfo->answer_txt.empty()) {
            auto ret = SpeecService::GetInstance()->SpeechTranscribeFile(practise_id, qainfo->answer_audio_path, "pcm",
                                                                         8000);
            answer_txt = std::get<1>(ret);
        }
        std::string answer_record_file = qainfo->answer_audio_path;
        unsigned int answer_duration = answer_time - question_time;

        auto similar_data = scoremanager->GetAnswerSimilarData(answer_txt, {{
                node_code, qainfo->answer_stander}});

        auto [answer_match, answer_analyse] = scoremanager->GetAnswerAnalyse(similar_data);
        unsigned int create_time = time(nullptr);
        unsigned int update_time = create_time;

        //打分、数据落盘
        LOGGER->info("QAInfo落盘 course_id:{} session_id:{} node_code:{}",
                     qainfo->course_id, qainfo->session_id, current_node->node_code);

        aia_course_practise_detail detail = {0, course_id, practise_id, node_code, question_id, question_statement_id,
                                             question_time, answer_time, status, answer_txt, answer_record_file,
                                             answer_duration, answer_match, answer_analyse, create_time, update_time};
        DBOperate::GetInstance()->GenerateCoursePractiseDetail(detail);
        return {answer_txt, 6};
    }

};