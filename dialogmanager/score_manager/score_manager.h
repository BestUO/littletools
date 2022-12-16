#pragma once
#include <string>

class ScoreManager {
public:
    static ScoreManager *GetInstance() {
        static ScoreManager instance;
        return &instance;
    }

    std::tuple<int, float, std::string> GetAnswerSimilarData(const std::string &answer_txt,
                                                             const std::vector<std::pair<std::string, std::string>> &candidate_answers);

    std::tuple<std::string, std::string> GetAnswerAnalyse(const std::tuple<int, float, std::string> &similar_data);

    std::string GetASRResult(std::string_view audiopath);

private:
    ScoreManager();

    ~ScoreManager() = default;
};