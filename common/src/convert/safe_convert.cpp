//
// Created by zhangjin on 9/29/18.
//

#include <string>
// #include "../common_define.h"
int stoi_s(const std::string &str, int defval) {
    int i = defval;
    try {
        i = std::stoi(str);
    } catch (...) {
        
    }
    return i;
}

unsigned long stoul_s(const std::string &str, unsigned long defval) {
    unsigned long i = defval;
    try {
        i = std::stoul(str);
    } catch (...) {

    }
    return i;
}

// void SplitString(const std::string& s, std::vector<std::string>& tokens, const std::string& delimiters = " ")
// {
//     std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
//     std::string::size_type pos = s.find_first_of(delimiters, lastPos);
//     while (std::string::npos != pos || std::string::npos != lastPos) {
//         tokens.push_back(s.substr(lastPos, pos - lastPos));
//         lastPos = s.find_first_not_of(delimiters, pos);
//         pos = s.find_first_of(delimiters, lastPos);
//     }
// }
