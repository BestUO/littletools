#ifndef EMIC_TASK_APPCONFIG_H
#define EMIC_TASK_APPCONFIG_H


#include <string>
#include <vector>
#include <thread>
#include <rapidjson/document.h>

class AppConfig {
public:
    static AppConfig* GetInstance();

    AppConfig(const AppConfig&) = delete;
    const AppConfig&operator=(const AppConfig&) = delete;

    float GetPerfectScore();

    float GetUpperBoundScore();

    float GetLowerBoundScore();
    float GetLowerNlpScore();
	float GetAdvanceScoreWhenHit();

	float GetKeyWordBonus();

	int GetSliceSize();

	std::string GetStopWords();
	std::string GetSymbols();

	bool GetUseKeywordMatch();
    
private:
    rapidjson::Value config_;
    AppConfig();
    virtual ~AppConfig();
};


#endif //EMIC_TASK_APPCONFIG_H
