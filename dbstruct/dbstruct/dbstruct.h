#ifndef _DBSTRUCT_H
#define _DBSTRUCT_H
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include <iostream>

#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

// struct aicall_tts_file_cache
// {
// 	int id;
// 	std::string TTS_text;
// 	int TTS_version_code;
// 	std::string tts_src;
// 	int tts_duration;
// 	int create_time;
// 	int access_time;
// 	int extension;
// };

// REFLECTION(aicall_tts_file_cache, id, TTS_text, TTS_version_code, tts_src, tts_duration, create_time, access_time, extension)

class MySql{
public:
    ormpp::dbng<ormpp::mysql> mysqlclient;
	static MySql * getInstance();
	void connect();
    void ReSetStatus();
    static MySql *instance;
private:
    // void RedisConnect();
    MySql(){};
    MySql(const MySql &){};
    MySql &operator=(const MySql &){};
    ~MySql();
};

#endif