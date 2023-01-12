#ifndef VOICEDIALOGUE_SPEECHTRANSCRIBER_H
#define VOICEDIALOGUE_SPEECHTRANSCRIBER_H

#include <string>

typedef unsigned int uuid_t;

// 自定义事件回调参数。
struct ParamTranscribeCallBack {
public:
    ParamTranscribeCallBack() {
        pthread_mutex_init(&mtxWord, NULL);
        pthread_cond_init(&cvWord, NULL);
    };
    ~ParamTranscribeCallBack() {
        pthread_mutex_destroy(&mtxWord);
        pthread_cond_destroy(&cvWord);
    };

    pthread_mutex_t mtxWord;
    pthread_cond_t cvWord;

    uuid_t userId;
    std::string recognizedContent;
};
// 自定义线程参数。
struct ParamTranscribe {
    std::string token;
    std::string appkey;
    std::string fileName;
    std::string fileFormat;
    int sampleRate;

    ParamTranscribeCallBack cbData;
};

extern void* pthreadTranscriber(void* arg);

#endif //VOICEDIALOGUE_SPEECHTRANSCRIBER_H
