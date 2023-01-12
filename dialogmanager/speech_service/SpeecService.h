#ifndef VOICEDIALOGUE_SPEECSERVICE_H
#define VOICEDIALOGUE_SPEECSERVICE_H

#include <string>
#include "speechTranscriber.h"


class SpeecService {
public:
    SpeecService(SpeecService const &) = delete;

    SpeecService(SpeecService &&) = delete;

    SpeecService &operator=(const SpeecService &) = delete;

    static SpeecService *GetInstance();

    std::tuple<int, std::string>
    SpeechTranscribeFile(uuid_t id, const std::string &file_name, const std::string &format, int sample_rate);

    std::tuple<int, std::string>
    RestfulAsr(uuid_t id, const std::string &file_name, const std::string &format, int sample_rate);

    int SpeechSynthesizeText(uuid_t id, const std::string &text);

private:
    SpeecService();

    virtual ~SpeecService();

    bool Start();
    
    void onTimer();

    int OnFileTranscribed(uuid_t id, const std::string &content);

    int OnTextSynthesized(uuid_t id, const std::string &content);

    std::string appkey_;

    pthread_rwlock_t token_rwlock_;
};


#endif //VOICEDIALOGUE_SPEECSERVICE_H
