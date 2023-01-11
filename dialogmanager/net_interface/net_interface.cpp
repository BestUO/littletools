#include "net_interface.h"
#include "session_manager/session_manager.h"
#include "course_manager/course_manager.h"
#include "tools/jsonwrap.hpp"
#include "generate_response.hpp"
#include "global.h"
#include <random>

NetInterFace::NetInterFace(rapidjson::Document &config):__server(1)
{
    __server.listen(config["httpserver_setting"]["host"].GetString(), config["httpserver_setting"]["port"].GetString());
    __server.set_http_handler<cinatra::GET, cinatra::POST>("/dialogmanger/nextcontext", [this](cinatra::request& req, cinatra::response& res) 
    {
        FunException(std::bind(&NetInterFace::NextContext,this,std::placeholders::_1, std::placeholders::_2), req, res);
	});

    __server.set_http_handler<cinatra::GET, cinatra::POST>("/dialogmanger/deletecourse", [this](cinatra::request& req, cinatra::response& res) 
    {
        FunException(std::bind(&NetInterFace::DeleteCourse,this,std::placeholders::_1, std::placeholders::_2), req, res);
	});

    __server.set_http_handler<cinatra::GET, cinatra::POST>("/dialogmanger/deletesession", [this](cinatra::request& req, cinatra::response& res) 
    {
        FunException(std::bind(&NetInterFace::DeleteSession,this,std::placeholders::_1, std::placeholders::_2), req, res);
	});

    __server.set_http_handler<cinatra::GET, cinatra::POST>("/dialogmanger/stop", [this](cinatra::request& req, cinatra::response& res) 
    {
        FunException(std::bind(&NetInterFace::StopDialogManager,this,std::placeholders::_1, std::placeholders::_2), req, res);
	});
}

void NetInterFace::NetInterFaceStart()
{
    __server.run();
}

void NetInterFace::FunException(std::function<void(cinatra::request& req, cinatra::response& res)> fun,cinatra::request& req, cinatra::response& res)
{
    try
    {
        fun(req,res);
    }
    catch(const std::exception& e)
    {
        LOGGER->error("FunException {}", e.what());
        res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::ErrorOccur())));
    }
}

void NetInterFace::DeleteCourse(cinatra::request& req, cinatra::response& res)
{
    auto body = ParseJson(req,res);
    if(body != std::nullopt)
    {
        auto params = ParseDeleteCourse(body.value(), req, res);
        if(params != std::nullopt)
        {    
            auto [course_id] = params.value();
            CourseManager::GetInstance()->DeleteCourse(course_id);
            res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::ExecuteSuccess())));
            LOGGER->info("DeleteCourse course {}", course_id);
        }
    }
}

void NetInterFace::DeleteSession(cinatra::request& req, cinatra::response& res)
{
    auto body = ParseJson(req,res);
    if(body != std::nullopt)
    {
        auto params = ParseDeleteSession(body.value(), req, res);
        if(params != std::nullopt)
        {   
            auto [session_id] = params.value();
            SessionManager::GetInstance()->DeleteSession(session_id);
            res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::ExecuteSuccess())));
        }
    }
}

void NetInterFace::StopDialogManager(cinatra::request& req, cinatra::response& res)
{
    SessionManager::GetInstance()->StopSessionManager();
    __server.stop();
    res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::ExecuteSuccess())));
    LOGGER->info("StopDialogManager");
}

void NetInterFace::NextContext(cinatra::request& req, cinatra::response& res)
{
    auto body = ParseJson(req,res);
    if(body != std::nullopt)
    {
        auto params = ParseNextContext(body.value(), req, res);
        if(params != std::nullopt)
        {
            auto [session_id,course_id,question_time,answer_time,answer_duration,is_expired,content] = params.value();
            auto sminstance = SessionManager::GetInstance();
            auto session = sminstance->GetSession(session_id,course_id);
            if(!session)
            {
                LOGGER->error("get session {} err",session_id);
                res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::GetSessionFail())));
                return;
            }

            std::string s;
            bool next = sminstance->ProcessSession(session, params.value());
            if(next)
                s = std::move(GenerateResponse::GetResponse(std::move(GenerateResponse::NextQuestion(session->current_qa))));
            else
                s = std::move(GenerateResponse::GetResponse(std::move(GenerateResponse::NoMoreQuestions(session->current_qa))));

            res.set_status_and_content(cinatra::status_type::ok, s.c_str());
            LOGGER->info("response session {} message {}", session->session_id, s);
        }
    }
}

std::optional<rapidjson::Document> NetInterFace::ParseJson(cinatra::request& req, cinatra::response& res)
{
    LOGGER->info("{} receive message:{}", req.get_full_url(), req.body());
    auto body = JsonSimpleWrap::GetPaser(req.body());
    if(body == std::nullopt)
    {
        LOGGER->warn("nextcontext parser err");
        res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::WrongJson())));
    }
    return body;
}

template<class ...T>
bool NetInterFace::AllMemberExist(rapidjson::Document& body, cinatra::response& res, T ...args)
{
    bool allmemberexist = (body.HasMember(args) && ...);
    if(!allmemberexist)
    {
        LOGGER->warn("nextcontext lost params");
        res.set_status_and_content(cinatra::status_type::ok, GenerateResponse::GetResponse(std::move(GenerateResponse::LostParams())));
    }
    return allmemberexist;
}

std::optional<std::tuple<unsigned int,unsigned int,unsigned int,unsigned int,unsigned int,unsigned int,std::string_view>>
NetInterFace::ParseNextContext(rapidjson::Document& body,cinatra::request& req, cinatra::response& res)
{
    if(!AllMemberExist(body,res,"session_id","course_id","content","question_time","answer_time","answer_duration","is_expired"))
        return std::nullopt;
    else
    {
        unsigned int session_id=body["session_id"].GetUint();
        unsigned int course_id=body["course_id"].GetUint();
        unsigned int question_time=body["question_time"].GetUint();
        unsigned int answer_time=body["answer_time"].GetUint();
        unsigned int answer_duration=body["answer_duration"].GetUint();
        unsigned int is_expired=body["is_expired"].GetUint();
        std::string_view content=body["content"].GetString();
        return std::make_tuple(session_id,course_id,question_time,answer_time,answer_duration,is_expired,content);
    }
}

std::optional<std::tuple<unsigned int>>
NetInterFace::ParseDeleteCourse(rapidjson::Document& body,cinatra::request& req, cinatra::response& res)
{
    if(!AllMemberExist(body,res,"course_id"))
        return std::nullopt;
    else
        return std::make_tuple(body["course_id"].GetInt());
}

std::optional<std::tuple<unsigned int>>
NetInterFace::ParseDeleteSession(rapidjson::Document& body,cinatra::request& req, cinatra::response& res)
{
    if(!AllMemberExist(body,res,"session_id"))
        return std::nullopt;
    else
        return std::make_tuple(body["session_id"].GetInt());
}