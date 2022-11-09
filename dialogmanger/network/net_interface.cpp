#include "net_interface.h"
#include "../global.h"
#include "tools/jsonwrap.hpp"
#include "generate_response.hpp"

NetInterFace::NetInterFace(rapidjson::Document &config):__server(1)
{
    __server.listen(config["httpserver_setting"]["host"].GetString(), config["httpserver_setting"]["port"].GetString());

    __server.set_http_handler<cinatra::GET, cinatra::POST>("/dialogmanger/nextcontext", [this](cinatra::request& req, cinatra::response& res) 
    {
        NextContext(req, res);
	});
}

void NetInterFace::NetInterFaceStart()
{
    __server.run();
}

void NetInterFace::NextContext(cinatra::request& req, cinatra::response& res)
{
    auto body = JsonSimpleWrap::GetPaser(req.body());
    if(body == std::nullopt)
    {
        LOGGER->info("nextcontext parser err {}", req.body());
        res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::WrongJson())));
    }
    else
    {
        if(!body.value().HasMember("session_id") || !body.value().HasMember("course_id") || !body.value().HasMember("user_id") || !body.value().HasMember("audio_path"))
        {
            LOGGER->info("nextcontext lost params {}", req.body());
            res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::LostParams())));
        }
        unsigned int session_id=body.value()["session_id"].GetInt();
        unsigned int user_id=body.value()["user_id"].GetInt();
        unsigned int course_id=body.value()["course_id"].GetInt();
        std::string_view audio_path=body.value()["audio_path"].GetString();

        res.set_status_and_content(cinatra::status_type::ok, "hello world");
    }
}