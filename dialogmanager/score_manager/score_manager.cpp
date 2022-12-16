#include "cinatra/cinatra.hpp"
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include "score_manager.h"
#include "tools/jsonwrap.hpp"
#include "log.h"

ScoreManager::ScoreManager()
{
    auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json").value();
}

std::tuple<int, float, std::string> ScoreManager::GetAnswerSimilarData(const std::string &answer_txt, const std::vector<std::pair<std::string, std::string>> &candidate_answers) {
    auto client = cinatra::client_factory::instance().new_client();
    auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json").value();
    auto url = config["answer_similar_service_uri"].GetString();

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("user_answer");
    writer.String(answer_txt.c_str());
    writer.EndObject();

    writer.StartArray();
    for(auto&& p: candidate_answers) {
        writer.StartObject();
        writer.Key("id");
        writer.String(p.first.c_str());
        writer.Key("text");
        writer.String(p.second.c_str());
        writer.EndObject();
    }
    writer.EndArray();

    auto jsonstr = buffer.GetString();
    LOGGER->info("post:{}, content:{}", url, jsonstr);
    auto rsp = client->post(url, jsonstr);
    if (!rsp.resp_body.empty()) {
        if (auto body = JsonSimpleWrap::GetPaser(rsp.resp_body)) {
            auto score = body.value()["score"].GetFloat();
            auto normallized_score = int(score / 10);
            auto similar_id = body.value()["similar_id"].GetString();
            return std::make_tuple(normallized_score, score, similar_id);
        }
    }
    return {0,0.0f, ""};
}

std::tuple<std::string, std::string> ScoreManager::GetAnswerAnalyse(const std::tuple<int, float, std::string> &similar_data)
{
    return std::make_tuple("", "");
}

std::string ScoreManager::GetASRResult(std::string_view audiopath)
{
    return "ASRResult";
}
