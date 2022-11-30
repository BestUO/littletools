#ifndef VOICEDIALOGUE_SPEECHSYNTHESIZER_H
#define VOICEDIALOGUE_SPEECHSYNTHESIZER_H
#include <string>
#include <fstream>

// 自定义事件回调参数。
struct ParamSynthesizeCallBack {
    std::string binAudioFile;
    std::ofstream audioFile;
    uint64_t startMs;
};
// 自定义线程参数。
struct ParamSynthesize {
    std::string token;
    std::string appkey;
    std::string text;
    std::string audioFile;
};

extern uint64_t getNow();
extern void* pthreadSynthesizer(void* arg);

#endif //VOICEDIALOGUE_SPEECHSYNTHESIZER_H
