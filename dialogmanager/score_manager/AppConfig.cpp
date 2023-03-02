#include "AppConfig.h"

#include <tools/jsonwrap.hpp>
#include "log.h"

using namespace std;


AppConfig* AppConfig::GetInstance() {
    static AppConfig config;
    return &config;
}

AppConfig::AppConfig() {
    auto value = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json").value();
    if (value.HasMember("TextSimilarity")) {
        config_ = value["TextSimilarity"];
    } else{
        config_.SetObject();
    }
}

AppConfig::~AppConfig() {
    LOGGER->info("AppConfig RELEASED");
}

float AppConfig::GetPerfectScore()
{
    float score = 1.0;
    if (config_.HasMember("perfect_score")) {
        score = config_["perfect_score"].GetFloat();
    }
    return score;
}

float AppConfig::GetUpperBoundScore()
{
    float score = 0.9;
    if (config_.HasMember("upper_bound_score")) {
        score = config_["upper_bound_score"].GetFloat();
    }
    return score;
}

float AppConfig::GetLowerBoundScore()
{
    float score = 0.6;
    if (config_.HasMember("lower_bound_score")) {
        score = config_["lower_bound_score"].GetFloat();
    }
    return score;
}

float AppConfig::GetLowerNlpScore()
{
    float score = 0.6;
    if (config_.HasMember("lower_nlp_score")) {
        score = config_["lower_nlp_score"].GetFloat();
    }
    return score;
}

float AppConfig::GetAdvanceScoreWhenHit()
{
    float score = 0.0;
    if (config_.HasMember("advance_when_hit")) {
        score = config_["advance_when_hit"].GetFloat();
    }
    return score;
}

std::string AppConfig::GetStopWords()
{
    std::string stop_words;
    if (config_.HasMember("stop_words")) {
        stop_words = config_["stop_words"].GetString();
    }
    return stop_words;
}

std::string AppConfig::GetSymbols()
{
    std::string symbols;
    if (config_.HasMember("symbols")) {
        symbols = config_["symbols"].GetString();
    }
    return symbols;
}

bool AppConfig::GetUseKeywordMatch()
{
    bool use_keyword_match = false;
    if (config_.HasMember("use_keyword_match")) {
        use_keyword_match = config_["use_keyword_match"].GetBool();
    }
    return use_keyword_match;
}

int AppConfig::GetSliceSize() {
    int slice_size = 0;
    if (config_.HasMember("slice_size")) {
        slice_size = config_["slice_size"].GetInt();
    }
    return slice_size;
}
