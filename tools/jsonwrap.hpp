#pragma once
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <unistd.h>
#include <fstream>
#include <optional>

class JsonSimpleWrap
{
public:
    static std::optional<rapidjson::Document> GetPaser(std::string_view str)
    {
        if(access(str.data(),F_OK) != -1)
        {
            std::ifstream file(str.data(), std::ios::binary);
            std::string buf((std::istreambuf_iterator<char>(file)),  
                            std::istreambuf_iterator<char>()); 

            return JsonPaser(buf.c_str());
        }
        else
            return JsonPaser(str.data());
    }

private:
    static std::optional<rapidjson::Document> JsonPaser(const char *buf)
    {
        rapidjson::Document parser;
        if (parser.Parse(buf).HasParseError())
            return std::nullopt;
        else 
            return parser;
    }
};