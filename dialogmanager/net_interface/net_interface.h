#pragma once
#include "cinatra/cinatra.hpp"
#include "tools/jsonwrap.hpp"

class NetInterFace
{
public:
    NetInterFace(rapidjson::Document &config);
    ~NetInterFace() = default;
    
    void NetInterFaceStart();
private:
    cinatra::http_server __server;

    void NextContext(cinatra::request& req, cinatra::response& res);
    void StopDialogManager(cinatra::request& req, cinatra::response& res);
    std::optional<rapidjson::Document> ParseJson(cinatra::request& req, cinatra::response& res);
    void DeleteCourse(cinatra::request& req, cinatra::response& res);
    void DeleteSession(cinatra::request& req, cinatra::response& res);
    std::optional<std::tuple<unsigned int,unsigned int,unsigned int,unsigned int,std::string_view>>
    ParseNextContext(rapidjson::Document& body,cinatra::request& req, cinatra::response& res);
    std::optional<std::tuple<unsigned int>> ParseDeleteCourse(rapidjson::Document& body,cinatra::request& req, cinatra::response& res);
    std::optional<std::tuple<unsigned int>> ParseDeleteSession(rapidjson::Document& body,cinatra::request& req, cinatra::response& res);
    template<class ...T> bool AllMemberExist(rapidjson::Document& body, cinatra::response& res, T ...args);
};