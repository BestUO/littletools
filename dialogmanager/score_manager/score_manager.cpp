#include "cinatra/cinatra.hpp"
#include <rapidjson/allocators.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <dialogmanager/dboperate/dboperate.hpp>
#include "score_manager.h"
#include "tools/jsonwrap.hpp"
#include "log.h"

ScoreManager::ScoreManager():similarity_(std::make_shared<TextSimilarity>(false))
{
    auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json").value();
    service_url_ = config["answer_similar_service_uri"].GetString();
}

std::tuple<int, float, std::string> ScoreManager::GetAnswerSimilarData(const std::string &answer_txt, const std::vector<std::pair<std::string, std::string>> &candidate_answers) 
{
    if(!answer_txt.empty())
    {
        auto client = cinatra::client_factory::instance().new_client();

        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        writer.StartObject();
        writer.Key("user_answer");
        writer.String(answer_txt.c_str());
        writer.Key("candidate_answers");
        writer.StartArray();
        for (auto &&p: candidate_answers) {
            writer.StartObject();
            writer.Key("id");
            writer.String(p.first.c_str());
            writer.Key("text");
            writer.String(p.second.c_str());
            writer.EndObject();
        }
        writer.EndArray();
        writer.EndObject();

        auto jsonstr = buffer.GetString();
        LOGGER->info("post:{}, content:{}", service_url_, jsonstr);
        auto rsp = client->post(service_url_, jsonstr);
        LOGGER->info("rsp.resp_body:{}", rsp.resp_body);
        if (!rsp.resp_body.empty()) {
            if (auto body = JsonSimpleWrap::GetPaser(rsp.resp_body)) {
                auto score = body.value()["score"].GetFloat();
                auto normalized = normalized_score(score);
                auto similar_id = body.value()["similar_id"].IsInt() ? std::to_string(body.value()["similar_id"].GetInt())
                                                                    : body.value()["similar_id"].GetString();
                return std::make_tuple(normalized, score, similar_id);
            }
        }
    }
    return {0, 0.0f, ""};
}

std::tuple<int, float, std::string> ScoreManager::GetAnswerSimilarDataLocal(const std::string &answer_txt, const std::vector<std::pair<std::string, std::string>> &candidate_answers) {
    if (!answer_txt.empty()) {
        std::set<std::string> b;
        std::transform(candidate_answers.cbegin(), candidate_answers.cend(), std::inserter(b, b.begin()),
                       [](const auto &pair) { return pair.second; });
        auto tmp = similarity_->GetScore(answer_txt, b, strategy_);
        tmp.second = (tmp.second > 1.0f) ? 1.0f : tmp.second;
        auto n = tmp.second;
        if (tmp.second <= similarity_->LowerBoundScore()) {
            n = 0;
        }
        LOGGER->info("tmp.second:{}, n:{}", tmp.second, n);
        int score = 10;
        if (n < 0.9f) {
            if (n >= 0.8f) {
                score -= 2;
            } else {
                if (n >= 0.7f) {
                    score -= 4;
                } else {
                    if (n >= 0.6f) {
                        score -= 6;
                    } else {
                        if (n >= 0.5f) {
                            score -= 8;
                        } else {
                            score -= 10;
                        }
                    }
                }
            }
        }
        return std::make_tuple(score, n, "");
    }
    return {0, 0.0f, ""};
}

std::string ScoreManager::GetAnswerAnalyse(
        const std::tuple<int, float, std::string> &similar_data,
        const std::tuple<float, std::vector<std::string>, std::vector<std::string>>& keywords_data,
        const std::tuple<int, std::vector<std::string>>&dirty_words_data,
        const std::tuple<int, int> &response_data) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();

    writer.Key("similar");
    writer.StartObject();
    writer.Key("score");
    writer.String(std::to_string(std::get<0>(similar_data)).c_str());
    writer.Key("detail");
    writer.String(FloatToFixedPointString<2>(std::get<1>(similar_data)).c_str());
    writer.EndObject();

    writer.Key("keywords");
    writer.StartObject();
    writer.Key("score");
    writer.String(FloatToFixedPointString<2>(std::get<0>(keywords_data)).c_str());

    writer.Key("detail");
    writer.StartObject();

    writer.Key("right");
    writer.StartArray();
    for (auto &&w : std::get<1>(keywords_data)) {
        writer.String(w.c_str());
    }
    writer.EndArray();

    writer.Key("wrong");
    writer.StartArray();
    for (auto &&w : std::get<2>(keywords_data)) {
        writer.String(w.c_str());
    }
    writer.EndArray();

    writer.EndObject();

    writer.EndObject();

    writer.Key("dirtyWords");
    writer.StartObject();
    writer.Key("score");
    writer.String(std::to_string(std::get<0>(dirty_words_data)).c_str());
    writer.Key("detail");
    writer.StartArray();
    for (auto &&w : std::get<1>(dirty_words_data)) {
        writer.String(w.c_str());
    }
    writer.EndArray();
    writer.EndObject();

    writer.Key("response");
    writer.StartObject();
    writer.Key("score");
    writer.String(std::to_string(std::get<0>(response_data)).c_str());
    writer.Key("detail");
    writer.String(std::to_string(std::get<1>(response_data)).c_str());
    writer.EndObject();

    writer.EndObject();

    return buffer.GetString();
}

std::string ScoreManager::GetAnswerMatch(const std::tuple<int, float, std::string> &similar_data, const std::string &answer_stander, const std::string &prompt_txt,
                                         const rapidjson::Document &prompt_steps) {
    rapidjson::Document d;
    d.SetObject();
    rapidjson::Value similar_value(rapidjson::kObjectType);
    similar_value.SetObject();
    similar_value.AddMember("score", rapidjson::Value().SetString(
            rapidjson::StringRef(std::to_string(std::get<0>(similar_data)).c_str())), d.GetAllocator());
    d.AddMember("similar", similar_value, d.GetAllocator());

    d.AddMember("standard", rapidjson::Value().SetString(rapidjson::StringRef(answer_stander.c_str())),
                            d.GetAllocator());
    d.AddMember("prompt_txt", rapidjson::Value().SetString(rapidjson::StringRef(prompt_txt.c_str())),
                            d.GetAllocator());

    rapidjson::Value prompt_steps_value(rapidjson::kArrayType);
    prompt_steps_value.SetArray();
    if (prompt_steps.IsArray()) {
        prompt_steps_value.CopyFrom(prompt_steps, d.GetAllocator());
    }
    d.AddMember("prompt_steps", prompt_steps_value, d.GetAllocator());

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    d.Accept(writer);
    return buffer.GetString();
}

std::tuple<float, std::vector<std::string>, std::vector<std::string>>
ScoreManager::GetKeywordsData(const std::string &answer_txt, const std::string &keywords) {
    auto ret = std::make_tuple(0.0f, std::vector<std::string>(), std::vector<std::string>());
    if (auto parser = JsonSimpleWrap::GetPaser(keywords)) {
        if (parser.value().Size() == 0) {
            std::get<0>(ret) = 10;
        } else {
            for (rapidjson::SizeType i = 0; i < parser.value().Size(); ++i) {
                auto &&w = parser.value()[i].GetString();
                if (answer_txt.find(w) != std::string::npos) {
                    std::get<1>(ret).push_back(w);
                } else {
                    std::get<2>(ret).push_back(w);
                }
            }
            std::get<0>(ret) = std::get<1>(ret).size() * 10.0f / parser.value().Size();
        }
    } else {
        std::get<0>(ret) = 10;
    }
    return ret;
}

std::tuple<int, std::vector<std::string>> ScoreManager::GetDirtyWordsData(const std::string &answer_txt, unsigned int course_id) {
    auto ret = std::make_tuple(10, std::vector<std::string>());
    auto eids = DBOperate::GetInstance()->GetEidOfCourse(course_id);
    if (eids.size() != 1) {
        LOGGER->error("Error eid of course_id:{}", course_id);
        return ret;
    }
    auto eid = std::get<0>(eids[0]);
    auto parent_ids = DBOperate::GetInstance()->GetParentIdOfWordData(eid);
    if (parent_ids.size() != 1) {
        LOGGER->error("Error parent_id of eid:{}", eid);
        return ret;
    }
    auto parent_id = std::get<0>(parent_ids[0]);
    auto dirty_words = DBOperate::GetInstance()->GetEnterpriseDirtyWords(eid, parent_id);
    for (auto&& [w] : dirty_words) {
        if (answer_txt.find(w) != std::string::npos) {
            std::get<1>(ret).push_back(w);
        }
    }
    if (!std::get<1>(ret).empty()) {
        std::get<0>(ret) = 0;
    }
    return ret;
}

std::tuple<int, std::vector<std::string>> ScoreManager::GetDirtyWordsData(const std::string &answer_txt, const std::vector<std::string> &dirty_words) {
    auto ret = std::make_tuple(10, std::vector<std::string>());
    for (auto&& w : dirty_words) {
        if (answer_txt.find(w) != std::string::npos) {
            std::get<1>(ret).push_back(w);
        }
    }
    if (!std::get<1>(ret).empty()) {
        std::get<0>(ret) = 0;
    }
    return ret;
}

std::tuple<int, int> ScoreManager::GetResponseData(int perfect_tolerance, unsigned int question_time, unsigned int answer_time) {
    auto response = (answer_time - question_time);
    int score = 10;
    LOGGER->info("perfect_tolerance:{}", perfect_tolerance);
    if (perfect_tolerance == 0) {
        perfect_tolerance = 3;
    }
    if (response > perfect_tolerance && response <= perfect_tolerance + 1) {
        score -= 2;
    } else if (response > perfect_tolerance + 1 && response <= perfect_tolerance + 2) {
        score -= 4;
    } else if (response > perfect_tolerance + 2 && response <= perfect_tolerance + 3) {
        score -= 6;
    } else if (response > perfect_tolerance + 3 && response <= perfect_tolerance + 4) {
        score -= 8;
    } else if (response > perfect_tolerance + 4) {
        score -= 10;
    }
    auto ret = std::make_tuple(score, response);
    return ret;
}

int ScoreManager::normalized_score(float score) {
    if (score <= 80) {
        return 0;
    }
    return int((score - 80 + 1) / 2);
}

template<int N>
std::string ScoreManager::FloatToFixedPointString(float value) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(N) << value;
    return ss.str();
}
