#include "score_manager.h"
#include "tools/jsonwrap.hpp"

ScoreManager::ScoreManager()
{
    auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json").value();
}

std::string ScoreManager::GetAnswerAnalyse()
{
    return "json";
}

std::string ScoreManager::GetASRResult(std::string_view audiopath)
{
    return "ASRResult";
}