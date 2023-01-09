#ifndef UO_TOOLS_SPEECHRESTFULASR_H
#define UO_TOOLS_SPEECHRESTFULASR_H

#include <string>

typedef unsigned int uuid_t;

struct ParamRestfulAsrCallBack {
    uuid_t userId;
    std::string recognizedContent;
};

extern int restfulAsrProcess(const std::string &appKey, const std::string &token, const std::string &file_name,
        const std::string &format, int sample_rate, ParamRestfulAsrCallBack* arg);


#endif //UO_TOOLS_SPEECHRESTFULASR_H
