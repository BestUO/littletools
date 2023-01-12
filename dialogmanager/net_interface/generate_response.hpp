#pragma once
#include <string>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

class GenerateResponse
{
public:
    GenerateResponse(/* args */) = default;
    ~GenerateResponse() = default;

    template<class... Args>
    static std::string GetResponse(std::tuple<Args...> &&params)
    {
        rapidjson::StringBuffer s;
        rapidjson::Writer<rapidjson::StringBuffer> writer(s);
        writer.StartObject();

        std::apply([&writer](auto&&... args)
        { 
            ((GenerateResponse::WritekeyValue(writer, args)),...);
        },
        std::move(params));

        writer.EndObject();
        return s.GetString();
    }

    static auto ErrorOccur()
    {
        return std::make_tuple(std::pair<std::string,int>("status",STATUS::ERROR),std::pair<std::string,std::string>("info","异常停止"));
    }

    static auto WrongJson()
    {
        return std::make_tuple(std::pair<std::string,int>("status",STATUS::ERROR),std::pair<std::string,std::string>("info","json解析错误"));
    }

    static auto LostParams()
    {
        return std::make_tuple(std::pair<std::string,int>("status",STATUS::ERROR),std::pair<std::string,std::string>("info","参数缺失"));
    }

    static auto GetSessionFail()
    {
        return std::make_tuple(std::pair<std::string,int>("status",STATUS::ERROR),std::pair<std::string,std::string>("info","session获取失败"));
    }

    static auto NoMoreQuestions(std::shared_ptr<QAInfo> current_qa)
    {
        return std::make_tuple(std::pair<std::string,int>("status",STATUS::FINISH),
                std::pair<std::string,std::string>("last_asr_result",current_qa->last_asr_result),
                std::pair<std::string,int>("last_score",current_qa->last_score),
                std::pair<std::string,std::string>("info","练习结束"));
    }

    static auto ExecuteSuccess()
    {
        return std::make_tuple(std::pair<std::string,int>("status",STATUS::SUCCESS),std::pair<std::string,std::string>("info","执行成功"));
    }

    static auto NextQuestion(std::shared_ptr<QAInfo> current_qa)
    {
        auto tmp = current_qa->question_detail.lock();
        return std::make_tuple(std::pair<std::string,int>("status",STATUS::SUCCESS),
            std::pair<std::string,std::string>("node_code",current_qa->current_node.lock()->node_code),
            std::pair<std::string,std::string>("last_asr_result",current_qa->last_asr_result),
            std::pair<std::string,int>("last_score",current_qa->last_score),
            std::pair<std::string,int>("statement_id",current_qa->tts_statement.tts_statement_id),
            std::pair<std::string,std::string>("question",current_qa->tts_statement.question),
            std::pair<std::string,std::string>("audio_path",current_qa->tts_statement.audio_path),
            std::pair<std::string,int>("perfect_tolerance",tmp?tmp->perfect_tolerance:0),
            std::pair<std::string,int>("question_id",tmp?tmp->question_id:0),
            std::pair<std::string,std::string>("prompt_steps",tmp?tmp->prompt_steps:""),
            std::pair<std::string,std::string>("prompt_txt",tmp?tmp->prompt_txt:""));
    }

private:
    enum STATUS {SUCCESS,ERROR,FINISH};
    //urgly code
    static void WritekeyValue(rapidjson::Writer<rapidjson::StringBuffer> &writer, std::pair<std::string,std::string> arg)
    {
        writer.Key(arg.first.c_str());
        writer.String(arg.second.c_str());
    };
    static void WritekeyValue(rapidjson::Writer<rapidjson::StringBuffer> &writer, std::pair<std::string,int> arg)
    {
        writer.Key(arg.first.c_str());
        writer.Int(arg.second);
    };
    static void WritekeyValue(rapidjson::Writer<rapidjson::StringBuffer> &writer, std::pair<std::string,double> arg)
    {
        writer.Key(arg.first.c_str());
        writer.Double(arg.second);
    };
};
