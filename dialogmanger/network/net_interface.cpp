#include "net_interface.h"
#include "../global.h"
#include "tools/jsonwrap.hpp"

NetInterFace::NetInterFace(rapidjson::Document &config):__server(1)
{
    __server.listen(config["httpserver_setting"]["host"].GetString(), config["httpserver_setting"]["port"].GetString());

    __server.set_http_handler<cinatra::GET, cinatra::POST>("/dialogmanger/nextcontext", [](cinatra::request& req, cinatra::response& res) 
    {
		res.set_status_and_content(cinatra::status_type::ok, "hello world");
	});
}

void NetInterFace::NetInterFaceStart()
{
    __server.run();
}