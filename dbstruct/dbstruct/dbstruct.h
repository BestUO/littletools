#pragma once

#include "ormpp/dbng.hpp"

struct aicall_tts_file_cache
{
	int id;
	std::string TTS_text;
	int TTS_version_code;
	std::string tts_src;
	int tts_duration;
	int create_time;
	int access_time;
	int extension;
};

REFLECTION(aicall_tts_file_cache, id, TTS_text, TTS_version_code, tts_src, tts_duration, create_time, access_time, extension)