#pragma once
#include "cinatra.hpp"
#include "tools/jsonwrap.hpp"

class NetInterFace
{
public:
    NetInterFace(rapidjson::Document &config);
    ~NetInterFace() = default;
    
    void NetInterFaceStart();
private:
    cinatra::http_server __server;
};