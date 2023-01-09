#include <pthread.h>
#include <unistd.h>
#include <ctime>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <fstream>
#include <sys/time.h>
#include "nlsClient.h"
#include "nlsEvent.h"
#include "speechTranscriberRequest.h"
#include "nlsToken.h"
#include "speechTranscriber.h"

#define FRAME_SIZE 3200

using namespace AlibabaNlsCommon;
using AlibabaNls::NlsClient; 
using AlibabaNls::NlsEvent; 
using AlibabaNls::LogDebug; 
using AlibabaNls::LogInfo; 
using AlibabaNls::SpeechTranscriberRequest; 


//@brief 获取sendAudio发送延时时间
//@param dataSize 待发送数据大小
//@param sampleRate 采样率：16k/8K
//@param compressRate 数据压缩率，例如压缩比为10:1的16k OPUS编码，此时为10，非压缩数据则为1。
//@return 返回sendAudio之后需要sleep的时间。
//@note 对于8k pcm 编码数据, 16位采样，建议每发送1600字节 sleep 100 ms.
//             对于16k pcm 编码数据, 16位采样，建议每发送3200字节 sleep 100 ms.
//             对于其它编码格式（OPUS）的数据, 由于传递给SDK的仍然是PCM编码数据,
//             按照SDK OPUS/OPU 数据长度限制, 需要每次发送640字节 sleep 20ms.
unsigned int getSendAudioSleepTime(int dataSize, int sampleRate, int compressRate) { 
    // 仅支持16位采样
    const int sampleBytes = 16;     // 仅支持单通道
    const int soundChannel = 1;     // 当前采样率，采样位数下每秒采样数据的大小。
    int bytes = (sampleRate * sampleBytes * soundChannel) / 8;     // 当前采样率，采样位数下每毫秒采样数据的大小。
    int bytesMs = bytes / 1000;     // 待发送数据大小除以每毫秒采样数据大小，以获取sleep时间。
    int sleepMs = (dataSize * compressRate) / bytesMs; 
    return sleepMs; 
}

//@brief 调用start()，成功与云端建立连接，SDK内部线程上报started事件。
//@param cbEvent 回调事件结构，详见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL。可以根据需求自定义参数。
void onTranscriptionStarted(NlsEvent* cbEvent, void* cbParam) { 
    ParamTranscribeCallBack* tmpParam = (ParamTranscribeCallBack*)cbParam;
    // 演示如何打印/使用用户自定义参数示例。
    printf("onTranscriptionStarted: %d\n", tmpParam->userId); 
    // 当前任务的task id，方便定位问题。
    printf("onTranscriptionStarted: status code=%d, task id=%s\n", 
            cbEvent->getStatusCode(), cbEvent->getTaskId()); 

    // 获取服务端返回的全部信息
    //printf("onTranscriptionStarted: all response=%s\n", cbEvent->getAllResponse());
    // 通知发送线程start()成功, 可以继续发送数据
    pthread_mutex_lock(&(tmpParam->mtxWord));
    pthread_cond_signal(&(tmpParam->cvWord));
    pthread_mutex_unlock(&(tmpParam->mtxWord));
} 

//@brief 服务端检测到了一句话的开始，SDK内部线程上报SentenceBegin事件。
//@param cbEvent 回调事件结构，详情参见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void onSentenceBegin(NlsEvent* cbEvent, void* cbParam) { 
    ParamTranscribeCallBack* tmpParam = (ParamTranscribeCallBack*)cbParam;
    // 演示如何打印/使用用户自定义参数示例。
    printf("onSentenceBegin: %d\n", tmpParam->userId);
    printf("onSentenceBegin: status code=%d, task id=%s, index=%d, time=%d\n", 
            cbEvent->getStatusCode(), cbEvent->getTaskId(),  
            cbEvent->getSentenceIndex(), //句子编号，从1开始递增。
            cbEvent->getSentenceTime() //当前已处理的音频时长，单位：毫秒。   
    );

    // 获取服务端返回的全部信息
    //printf("onTranscriptionStarted: all response=%s\n", cbEvent->getAllResponse()); 
} 

//@brief 服务端检测到了一句话结束，SDK内部线程上报SentenceEnd事件。
//@param cbEvent 回调事件结构，详情参见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void onSentenceEnd(NlsEvent* cbEvent, void* cbParam) {  
    ParamTranscribeCallBack* tmpParam = (ParamTranscribeCallBack*)cbParam;
    // 演示如何打印
    //使用用户自定义参数示例。
    printf("onSentenceEnd: %d\n", tmpParam->userId);     
    printf("onSentenceEnd: status code=%d, task id=%s, index=%d, time=%d, begin_time=%d, result=%s\n", 
            cbEvent->getStatusCode(), 
            cbEvent->getTaskId(), 
            cbEvent->getSentenceIndex(), //句子编号，从1开始递增。  
            cbEvent->getSentenceTime(), //当前已处理的音频时长，单位：毫秒。     
            cbEvent->getSentenceBeginTime(), // 对应的SentenceBegin事件的时间。  
            cbEvent->getResult()    // 当前句子的完整识别结果。   
    ); 
    // 获取服务端返回的全部信息
    //printf("onTranscriptionStarted: all response=%s\n", cbEvent->getAllResponse());
    tmpParam->recognizedContent = cbEvent->getResult();
} 

//@brief 识别结果发生了变化，SDK在接收到云端返回的最新结果时，其内部线程上报ResultChanged事件。
//@param cbEvent 回调事件结构，详情参见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void onTranscriptionResultChanged(NlsEvent* cbEvent, void* cbParam) {
    ParamTranscribeCallBack* tmpParam = (ParamTranscribeCallBack*)cbParam;
    // 演示如何打印/使用用户自定义参数示例。
    printf("onTranscriptionResultChanged: %d\n", tmpParam->userId);   
    printf("onTranscriptionResultChanged: status code=%d, task id=%s, index=%d, time=%d, result=%s\n", 
            cbEvent->getStatusCode(), 
            cbEvent->getTaskId(),                 
            cbEvent->getSentenceIndex(), //句子编号，从1开始递增。  
            cbEvent->getSentenceTime(), //当前已处理的音频时长，单位：毫秒。  
            cbEvent->getResult()    // 当前句子的完整识别结果  
    );     
    // 获取服务端返回的全部信息
    //printf("onTranscriptionStarted: all response=%s\n", cbEvent->getAllResponse()); 
}

//@brief 服务端停止实时音频流识别时，SDK内部线程上报Completed事件。
//@note 上报Completed事件之后，SDK内部会关闭识别连接通道。此时调用sendAudio会返回-1，请停止发送。
//@param cbEvent 回调事件结构，详情参见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void onTranscriptionCompleted(NlsEvent* cbEvent, void* cbParam) {
    ParamTranscribeCallBack* tmpParam = (ParamTranscribeCallBack*)cbParam;
    // 演示如何打印/使用用户自定义参数示例。
    printf("onTranscriptionCompleted: %d\n", tmpParam->userId);     
    printf("onTranscriptionCompleted: status code=%d, task id=%s\n", 
            cbEvent->getStatusCode(), 
            cbEvent->getTaskId());
    printf("onTranscriptionCompleted: all response=%s\n", cbEvent->getAllResponse());
} 

//@brief 识别过程（包含start()、send()、stop()）发生异常时，SDK内部线程上报TaskFailed事件。
//@note 上报TaskFailed事件之后，SDK内部会关闭识别连接通道。此时调用sendAudio会返回-1，请停止发送。
//@param cbEvent 回调事件结构，详情参见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void onTaskFailed(NlsEvent* cbEvent, void* cbParam) { 
    ParamTranscribeCallBack* tmpParam = (ParamTranscribeCallBack*)cbParam;
    // 演示如何打印/使用用户自定义参数示例。
    printf("onTaskFailed: %d\n", tmpParam->userId); 
    printf("onTaskFailed: status code=%d, task id=%s, error message=%s\n", 
            cbEvent->getStatusCode(), 
            cbEvent->getTaskId(), 
            cbEvent->getErrorMessage()
    );     
    // 获取服务端返回的全部信息
    //printf("onTaskFailed: all response=%s\n", cbEvent->getAllResponse()); 
} 

//@brief SDK内部线程上报语音表单结果事件
//@param cbEvent 回调事件结构，详情参见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void onSentenceSemantics(NlsEvent* cbEvent, void* cbParam) {     
    ParamTranscribeCallBack* tmpParam = (ParamTranscribeCallBack*)cbParam;
    // 演示如何打印/使用用户自定义参数示例。
    printf("onSentenceSemantics: %d\n", tmpParam->userId);     
    // 获取服务端返回的全部信息。
    printf("onSentenceSemantics: all response=%s\n", cbEvent->getAllResponse()); 
} 

//@brief 识别结束或发生异常时，会关闭连接通道，SDK内部线程上报ChannelCloseed事件。
//@param cbEvent 回调事件结构，详情参见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void onChannelClosed(NlsEvent* cbEvent, void* cbParam) {
    auto* tmpParam = (ParamTranscribeCallBack*)cbParam;
    printf("onChannelClosed: all response=%s\n", cbEvent->getAllResponse());

    //通知发送线程, 最终识别结果已经返回, 可以调用stop()
    pthread_mutex_lock(&(tmpParam->mtxWord));
    pthread_cond_signal(&(tmpParam->cvWord));
    pthread_mutex_unlock(&(tmpParam->mtxWord));
} 

// 工作线程
void* pthreadTranscriber(void* arg) {
    int sleepMs = 0;
    int ret = 0;
    // 0: 从自定义线程参数中获取token，配置文件等参数。     
    auto* tst = (ParamTranscribe*)arg;
    if (tst == NULL) {
        printf("arg is not valid\n");         
        return NULL;     
    }
    ParamTranscribeCallBack *cbParam = &tst->cbData;
    /* 打开音频文件，获取数据 */     
    std::ifstream fs;
    fs.open(tst->fileName.c_str(), std::ios::binary | std::ios::in);     
    if (!fs) {         
        printf("%s isn't exist..\n", tst->fileName.c_str());         
        return NULL;     
    }     
    //2: 创建实时音频流识别SpeechTranscriberRequest对象。     
    SpeechTranscriberRequest* request = NlsClient::getInstance()->createTranscriberRequest();    
    if (request == NULL) {         
        printf("createTranscriberRequest failed.\n");         
        return NULL;     
    }     
    request->setOnTranscriptionStarted(onTranscriptionStarted, cbParam);                // 设置识别启动回调函数     
    request->setOnTranscriptionResultChanged(onTranscriptionResultChanged, cbParam);    // 设置识别结果变化回调函数     
    request->setOnTranscriptionCompleted(onTranscriptionCompleted, cbParam);            // 设置语音转写结束回调函数     
    request->setOnSentenceBegin(onSentenceBegin, cbParam);                              // 设置一句话开始回调函数     
    request->setOnSentenceEnd(onSentenceEnd, cbParam);                                  // 设置一句话结束回调函数     
    request->setOnTaskFailed(onTaskFailed, cbParam);                                    // 设置异常识别回调函数     
    request->setOnChannelClosed(onChannelClosed, cbParam);                              // 设置识别通道关闭回调函数     
    request->setAppKey(tst->appkey.c_str());            // 设置appkey，必选参数。     
    request->setFormat(tst->fileFormat.c_str());                          // 设置音频数据编码格式，默认值PCM。
    request->setSampleRate(tst->sampleRate);                // 设置音频数据采样率，可选参数，目前支持16000/8000，默认值16000。
    request->setIntermediateResult(false);               // 设置是否返回中间识别结果，可选参数，默认false。
    request->setPunctuationPrediction(true);            // 设置是否在后处理中添加标点，可选参数，默认false。     
    request->setInverseTextNormalization(true);         // 设置是否在后处理中执行数字转写，可选参数，默false。
    //语音断句检测阈值，一句话之后静音长度超过该值，即本句结束，合法参数范围200ms～2000ms，默认值800ms。
    //request->setMaxSentenceSilence(800);
    //request->setCustomizationId("TestId_123"); //定制模型id，可选。
    //request->setVocabularyId("TestId_456"); //定制泛热词id，可选。
    // 用于传递某些定制化、高级参数设置，参数格式为JSON格式： {"key": "value"}
    //request->setPayloadParam("{\"vad_model\": \"farfield\"}");
    //设置是否开启词模式。     
    request->setPayloadParam("{\"enable_words\": true}");
    //语义断句，默认false，非必需则不建议设置。
    //request->setPayloadParam("{\"enable_semantic_sentence_detection\": false}");
    //是否开启顺滑，默认不开启，非必需则不建议设置。
    //request->setPayloadParam("{\"disfluency\": true}");
    //设置vad的模型，默认不设置，非必需则不建议设置。
    //request->setPayloadParam("{\"vad_model\": \"farfield\"}");
    //设置是否忽略单句超时
    //request->setPayloadParam("{\"enable_ignore_sentence_timeout\": false}");
    //vad断句开启后处理，默认不设置，非必需则不建议设置。
    //request->setPayloadParam("{\"enable_vad_unify_post\": true}");     
    request->setToken(tst->token.c_str());

    struct timespec outtime;
    struct timeval now;
    //3: start()为异步操作。成功返回started事件，失败返回TaskFailed事件。
    ret = request->start();
    if (ret < 0) {
        printf("start() failed. may be can not connect server. please check network or firewalld\n"); 
        NlsClient::getInstance()->releaseTranscriberRequest(request); // start()失败，释放request对象。
        return NULL;    
    } else {
        // 等待started事件返回, 再发送
        printf("wait started callback.\n");

        // 语音服务器存在来不及处理当前请求, 10s内不返回任何回调的问题,
        // 然后在10s后返回一个TaskFailed回调, 所以需要设置一个超时机制.
        gettimeofday(&now, NULL);
        outtime.tv_sec = now.tv_sec + 5;
        outtime.tv_nsec = now.tv_usec * 1000;
        pthread_mutex_lock(&(cbParam->mtxWord));
        if (ETIMEDOUT == pthread_cond_timedwait(&(cbParam->cvWord), &(cbParam->mtxWord), &outtime)) {
            printf("start timeout.\n");
            pthread_mutex_unlock(&(cbParam->mtxWord));
            request->cancel();
            NlsClient::getInstance()->releaseTranscriberRequest(request);
            return NULL;
        }
        pthread_mutex_unlock(&(cbParam->mtxWord));
    }

    while (!fs.eof()) {         
        uint8_t data[FRAME_SIZE] = {0};         
        fs.read((char *)data, sizeof(uint8_t) * FRAME_SIZE);         
        size_t nlen = fs.gcount();         
        if (nlen <= 0) {             
            continue;         
        }         
        //4: 发送音频数据。sendAudio返回-1表示发送失败，需要停止发送。
        int ret = request->sendAudio(data, nlen);         
        if (ret < 0) {             
            // 发送失败，退出循环数据发送。
            printf("send data fail.\n");             
            break;         
        }         
         //语音数据发送控制：
        //语音数据是实时的，不需要sleep控制速率。
        //语音数据来自文件，发送时需要控制速率，使单位时间内发送的数据大小接近单位时间原始语音数据存储的大小。         
        sleepMs = getSendAudioSleepTime(nlen, tst->sampleRate, 1);  // 根据发送数据大小、采样率、数据压缩比，获取sleep时间。
        //5: 语音数据发送延时控制
        usleep(sleepMs * 30);
    }
    // 关闭音频文件     
    fs.close();     
    //6: 通知云端数据发送结束
    //stop()为异步操作，失败返回TaskFailed事件。     
    ret = request->stop();
    if (ret == 0) {
        printf("wait closed callback.\n");

        // 语音服务器存在来不及处理当前请求, 10s内不返回任何回调的问题,
        // 然后在10s后返回一个TaskFailed回调, 错误信息为:
        // "Gateway:IDLE_TIMEOUT:Websocket session is idle for too long time, the last directive is 'StopRecognition'!"
        // 所以需要设置一个超时机制.
        gettimeofday(&now, NULL);
        outtime.tv_sec = now.tv_sec + 5;
        outtime.tv_nsec = now.tv_usec * 1000;
        // 等待closed事件后再进行释放
        pthread_mutex_lock(&(cbParam->mtxWord));
        if (ETIMEDOUT == pthread_cond_timedwait(&(cbParam->cvWord), &(cbParam->mtxWord), &outtime)) {
            printf("stop timeout\n");
            pthread_mutex_unlock(&(cbParam->mtxWord));
            NlsClient::getInstance()->releaseTranscriberRequest(request);
            return NULL;
        }
        pthread_mutex_unlock(&(cbParam->mtxWord));
    } else {
        printf("stop ret is %d\n", ret);
    }
    //7: 识别结束，释放request对象。     
    NlsClient::getInstance()->releaseTranscriberRequest(request);     
    return NULL;
} 
