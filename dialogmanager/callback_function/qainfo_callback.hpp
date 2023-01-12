#pragma once

#include "dboperate/dboperate.hpp"
#include "speech_service/SpeecService.h"
#include "global.h"
#include "score_manager/score_manager.h"


class QAInfoCallBackFunction {
public:
    static std::tuple<std::string, int> StoreInDB(std::shared_ptr<QAInfo> qainfo, const std::vector<std::string> &dirty_words) {
        auto scoremanager = ScoreManager::GetInstance();
        auto question_detail = qainfo->question_detail.lock();
        auto current_node = qainfo->current_node.lock();
        unsigned int course_id = qainfo->course_id;
        unsigned int practise_id = qainfo->session_id;
        std::string node_code = current_node->node_code;

        unsigned int question_id = question_detail ? question_detail->question_id : 0;
        auto keywords = question_detail ? question_detail->keywords : "";
        auto prompt_txt = question_detail ? question_detail->prompt_txt : "";
        auto prompt_steps = question_detail ? GetDocument(question_detail->prompt_steps) : rapidjson::Document();

        unsigned int question_statement_id = qainfo->tts_statement.tts_statement_id;
        unsigned int question_time = qainfo->question_time;
        unsigned int answer_time = qainfo->answer_time;
        unsigned int answer_duration = qainfo->answer_duration;
        unsigned int status = qainfo->is_expired;
        std::string answer_txt = qainfo->answer_txt;
        if (qainfo->answer_txt.empty()) {
            // auto ret = SpeecService::GetInstance()->SpeechTranscribeFile(practise_id, qainfo->answer_audio_path, "wav",
            //                                                              8000);
            auto ret = SpeecService::GetInstance()->RestfulAsr(practise_id, qainfo->answer_audio_path, "wav",
                                                                        8000);
            answer_txt = std::get<1>(ret);
        }
        std::string answer_record_file = qainfo->relative_path;

        auto similar_data = scoremanager->GetAnswerSimilarData(answer_txt, {{node_code, qainfo->answer_stander}});

        auto keywords_data = scoremanager->GetKeywordsData(answer_txt, keywords);

        auto dirty_words_data = scoremanager->GetDirtyWordsData(answer_txt, dirty_words);

        auto response_data = scoremanager->GetResponseData(question_time, answer_time);

        auto answer_analyse = scoremanager->GetAnswerAnalyse(similar_data, keywords_data,
                                                                            dirty_words_data, response_data);
        auto answer_match = scoremanager->GetAnswerMatch(similar_data, qainfo->answer_stander, prompt_txt, prompt_steps);

        //打分、数据落盘
        LOGGER->info("QAInfo落盘 course_id:{} session_id:{} node_code:{}",
                     qainfo->course_id, qainfo->session_id, current_node->node_code);

        aia_course_practise_detail detail = {course_id, practise_id, node_code, question_id, question_statement_id,
                                             question_time, answer_time, status, answer_txt, answer_record_file,
                                             answer_duration, answer_match, answer_analyse};
        DBOperate::GetInstance()->DBInsert("aia_course_practise_detail",detail.meta());
        return {answer_txt, std::get<0>(similar_data)};
    }

    static rapidjson::Document GetDocument(const std::string &json) {
        rapidjson::Document doc;
        doc.Parse(json.c_str());
        return doc;
    }
};