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

    static auto WrongJson()
    {
        return std::make_tuple(std::pair<std::string,int>("status",1),std::pair<std::string,std::string>("info","json解析错误"));
    }

    static auto LostParams()
    {
        return std::make_tuple(std::pair<std::string,int>("status",1),std::pair<std::string,std::string>("info","参数缺失"));
    }

private:
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
