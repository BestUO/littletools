#include "net_interface.h"
#include "session_manager/session_manager.h"
#include "tools/jsonwrap.hpp"
#include "generate_response.hpp"
#include "global.h"
#include <random>

NetInterFace::NetInterFace(rapidjson::Document &config):__server(1)
{
    __server.listen(config["httpserver_setting"]["host"].GetString(), config["httpserver_setting"]["port"].GetString());
    __server.set_http_handler<cinatra::GET, cinatra::POST>("/dialogmanger/nextcontext", [this](cinatra::request& req, cinatra::response& res) 
    {
        NextContext(req, res);
	});

    __server.set_http_handler<cinatra::GET, cinatra::POST>("/dialogmanger/stop", [this](cinatra::request& req, cinatra::response& res) 
    {
        StopDialogManager(req, res);
	});
}

void NetInterFace::NetInterFaceStart()
{
    __server.run();
}

void NetInterFace::StopDialogManager(cinatra::request& req, cinatra::response& res)
{
    SessionManager::GetInstance()->StopSessionManager();
    __server.stop();
    res.set_status_and_content(cinatra::status_type::ok, "bye");
}

void NetInterFace::NextContext(cinatra::request& req, cinatra::response& res)
{
    LOGGER->info("receive message:{}", req.body());
    auto body = JsonSimpleWrap::GetPaser(req.body());
    if(body == std::nullopt)
    {
        LOGGER->warn("nextcontext parser err");
        res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::WrongJson())));
        return;
    }
    else
    {
        if(!body.value().HasMember("session_id") || !body.value().HasMember("course_id") || !body.value().HasMember("user_id") || 
            !body.value().HasMember("content") || !body.value().HasMember("question_time") || !body.value().HasMember("answer_time"))
        {
            LOGGER->warn("nextcontext lost params");
            res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::LostParams())));
            return;
        }
        unsigned int session_id=body.value()["session_id"].GetInt();
        unsigned int user_id=body.value()["user_id"].GetInt();
        unsigned int course_id=body.value()["course_id"].GetInt();
        std::string_view content=body.value()["content"].GetString();

        auto sminstance = SessionManager::GetInstance();
        auto session = sminstance->GetSession(session_id,course_id);
        if(!session)
        {
            LOGGER->error("get session {} err",session_id);
            res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::GetSessionFail())));
            return;
        }

        bool next = sminstance->ProcessSession(session, content, std::chrono::system_clock::from_time_t(body.value()["question_time"].GetInt()),
                    std::chrono::system_clock::from_time_t(body.value()["answer_time"].GetInt()));
        std::string s;
        if(next)
            s = std::move(GenerateResponse::GetResponse(std::move(GenerateResponse::NextQuestion(session->current_qa))));
        else
            s = std::move(GenerateResponse::GetResponse(std::move(GenerateResponse::NoMoreQuestions())));

        res.set_status_and_content(cinatra::status_type::ok, s.c_str());
        LOGGER->info("response session {} message {}", session->session_id, s);
    }
}

