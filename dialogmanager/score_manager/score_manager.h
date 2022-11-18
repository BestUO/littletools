#pragma once
#include <string>

class ScoreManager
{
public:
    static ScoreManager *GetInstance()
    {
        static ScoreManager instance;
        return &instance;
    }
    
    std::string GetAnswerAnalyse();
    std::string GetASRResult(std::string_view audiopath);
private:
    ScoreManager();
    ~ScoreManager()=default;
};