#ifndef COMMON_DEFINE_H_INCLUDED
#define COMMON_DEFINE_H_INCLUDED

#include "stdlib.h"
#include <string>
#include <vector>


typedef unsigned int uuid_t;

extern int64_t get_timestamp_micro(struct timeval &tv);

extern int64_t get_timestamp_micro();

extern int64_t get_timestamp(struct timeval &tv);

extern int64_t get_timestamp();

extern int64_t get_timestamp_sec(struct timeval &tv);

extern int64_t get_timestamp_sec();

extern int64_t get_timer_timestamp(struct timeval &tv);

extern int64_t get_timer_timestamp();

extern void time_str(const struct timeval& tval, std::string &out);

extern void time_str(std::string &out);

extern int stoi_s(const std::string &str, int defval=0);

extern unsigned long stoul_s(const std::string &str, unsigned long defval=0);

extern void SplitString(const std::string& s, std::vector<std::string>& tokens, const std::string& delimiters = " ");

extern std::string generate_callrecord_key(int eid, std::string phone, bool answered=false);

extern std::string generate_ecfg_key(int eid);

extern std::string generate_ephones_key(int eid);

extern std::string script_generate_callrecord_key(int eid,int script_id, std::string phone, bool answered=false);

extern std::string script_generate_ecfg_key(int eid,int script_id);

extern std::string script_generate_ephones_key(int eid,int script_id);

extern std::string GenerateCheatKey(std::string title, std::string script_id, std::string phone);

extern std::string GenerateCheatValue(std::string cc_number, std::string nodeid, std::string state, bool promptfromquestion);
#endif // COMMON_DEFINE_H_INCLUDED
