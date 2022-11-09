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
};