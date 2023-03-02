#pragma once
#include <string>
#include "TextCompare.h"
#include "TextSimilarity.h"

class ScoreManager {
public:
    static ScoreManager *GetInstance() {
        static ScoreManager instance;
        return &instance;
    }

    std::tuple<int, float, std::string> GetAnswerSimilarData(const std::string &answer_txt,
                                                             const std::vector<std::pair<std::string, std::string>> &candidate_answers);
    std::tuple<int, float, std::string> GetAnswerSimilarDataLocal(const std::string &answer_txt,
                                                             const std::vector<std::pair<std::string, std::string>> &candidate_answers);
    std::string
    GetAnswerAnalyse(const std::tuple<int, float, std::string> &similar_data,
                     const std::tuple<float, std::vector<std::string>, std::vector<std::string>> &keywords_data,
                     const std::tuple<int, std::vector<std::string>> &dirty_words_data,
                     const std::tuple<int, int> &response_data);

    std::string
    GetAnswerMatch(const std::tuple<int, float, std::string> &similar_data, const std::string &answer_stander,
                   const std::string &prompt_txt,
                   const rapidjson::Document &prompt_steps);

    std::tuple<float, std::vector<std::string>, std::vector<std::string>>
    GetKeywordsData(const std::string &answer_txt, const std::string &keywords);

    std::tuple<int, std::vector<std::string>> GetDirtyWordsData(const std::string &answer_txt, unsigned int course_id);

    std::tuple<int, std::vector<std::string>>
    GetDirtyWordsData(const std::string &answer_txt, const std::vector<std::string> &dirty_words);

    std::tuple<int, int> GetResponseData(int perfect_tolerance, unsigned int question_time, unsigned int answer_time);

private:
    ScoreManager();
    ~ScoreManager() = default;
    std::string service_url_;
    std::shared_ptr<TextSimilarity> similarity_;
    MatchStrategy strategy_{MatchStrategy::WHOLE};

    static int normalized_score(float);

    template<int N>
    std::string FloatToFixedPointString(float);
};