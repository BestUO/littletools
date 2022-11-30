#include<string.h>
#include<unistd.h>
#include<pthread.h>
#include<stdlib.h>
#include<ctime>
#include<string>
#include<iostream>
#include<vector>
#include<fstream>
#include<sys/time.h>
#include"nlsClient.h"
#include"nlsEvent.h"
#include"nlsToken.h"
#include "speechSynthesizerRequest.h"
#include "speechSynthesizer.h"

using namespace AlibabaNlsCommon; 
using AlibabaNls::NlsClient; 
using AlibabaNls::NlsEvent; 
using AlibabaNls::LogDebug; 
using AlibabaNls::LogInfo; 
using AlibabaNls::SpeechSynthesizerRequest;


//@brief 在接收到云端返回合成结束消息时，SDK内部线程上报Completed事件。
//@note 上报Completed事件之后，SDK内部会关闭识别连接通道。
//@param cbEvent 回调事件结构，详见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void OnSynthesisCompleted(NlsEvent* cbEvent, void* cbParam) { 
    ParamSynthesizeCallBack* tmpParam = (ParamSynthesizeCallBack*)cbParam;
    // 演示如何打印/使用用户自定义参数示例。
    printf("OnSynthesisCompleted: %s\n", tmpParam->binAudioFile.c_str()); 
    // 获取消息的状态码，成功为0或者20000000，失败时对应失败的错误码。
    // 当前任务的task id，方便定位问题，作为和服务端交互的唯一标识建议输出。
    printf("OnSynthesisCompleted: status code=%d, task id=%s\n", cbEvent->getStatusCode(), cbEvent->getTaskId()); 
    // 获取服务端返回的全部信息。
    //printf("OnSynthesisCompleted: all response=%s\n", cbEvent->getAllResponse()); 
} 

//@brief 合成过程发生异常时，SDK内部线程上报TaskFailed事件。
//@note 上报TaskFailed事件之后，SDK内部会关闭识别连接通道。
//@param cbEvent 回调事件结构，详见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void OnSynthesisTaskFailed(NlsEvent* cbEvent, void* cbParam) { 
    ParamSynthesizeCallBack* tmpParam = (ParamSynthesizeCallBack*)cbParam;
    // 演示如何打印/使用用户自定义参数示例。
    printf("OnSynthesisTaskFailed: %s\n", tmpParam->binAudioFile.c_str()); 
    // 当前任务的task id。
    printf("OnSynthesisTaskFailed: status code=%d, task id=%s, error message=%s\n", 
cbEvent->getStatusCode(), cbEvent->getTaskId(), cbEvent->getErrorMessage()); 
} 

//@brief 识别结束或发生异常时，会关闭连接通道，SDK内部线程上报ChannelCloseed事件。
//@param cbEvent 回调事件结构，详见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void OnSynthesisChannelClosed(NlsEvent* cbEvent, void* cbParam) { 
    ParamSynthesizeCallBack* tmpParam = (ParamSynthesizeCallBack*)cbParam;
    // 演示如何打印/使用用户自定义参数示例。
    printf("OnSynthesisChannelClosed: %s\n", tmpParam->binAudioFile.c_str());   
    printf("OnSynthesisChannelClosed: %s\n", cbEvent->getAllResponse()); 
    tmpParam->audioFile.close();
    delete tmpParam;
}

//@brief 文本上报服务端后，收到服务端返回的二进制音频数据，SDK内部线程通过BinaryDataRecved事件上报给用户。
//@param cbEvent 回调事件结构，详见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void OnBinaryDataRecved(NlsEvent* cbEvent, void* cbParam) { 
    ParamSynthesizeCallBack* tmpParam = (ParamSynthesizeCallBack*)cbParam;
    if(tmpParam->startMs > 0 ) {  
        // 重要提示：一旦获取到语音流，如第一次从服务端返回合成语音流，即可以进行播放或者其他处理。示例为保存到本地文件。
        // 第一次收到语音流数据，计算TTS合成首包延迟。另外此处计算首包延迟时也包括了start操作（即本程序连接公共云服务端的时间），而该时间受不同网络因素影响可能有较大差异。
        uint64_t now = getNow(); 
        printf("first latency = %lld ms, task id = %s\n",
                (now - tmpParam->startMs) / 1000, cbEvent->getTaskId());  
        tmpParam->startMs = 0;
    }
     // 演示如何打印/使用用户自定义参数示例。
    printf("OnBinaryDataRecved: %s\n", tmpParam->binAudioFile.c_str()); 
    const std::vector<unsigned char>& data = cbEvent->getBinaryData(); // getBinaryData() ：获取文本合成的二进制音频数据。
    printf("OnBinaryDataRecved: status code=%d, task id=%s, data size=%d\n", 
cbEvent->getStatusCode(), cbEvent->getTaskId(), data.size()); 
    // 以追加形式将二进制音频数据写入文件。
    if (data.size() > 0) {  
        tmpParam->audioFile.write((char*)&data[0], data.size());  
    } 
} 

//@brief 返回tts文本对应的日志信息，增量返回对应的字幕信息。
//@param cbEvent 回调事件结构，详见nlsEvent.h。
//@param cbParam 回调自定义参数，默认为NULL，可以根据需求自定义参数。
void OnMetaInfo(NlsEvent* cbEvent, void* cbParam) { 
    ParamSynthesizeCallBack* tmpParam = (ParamSynthesizeCallBack*)cbParam;
   // 演示如何打印/使用用户自定义参数示例。
    printf("OnBinaryDataRecved: %s\n", tmpParam->binAudioFile.c_str()); 
    printf("OnMetaInfo: task id=%s, respose=%s\n", cbEvent->getTaskId(), cbEvent->getAllResponse()); 
}

 // 工作线程
void* pthreadSynthesizer(void* arg) {
    // 0：从自定义线程参数中获取token，配置文件等参数。 
    ParamSynthesize* tst = (ParamSynthesize*)arg;
    if (tst == NULL) { 
        printf("arg is not valid\n"); 
        return NULL; 
    } 
    // 1：初始化自定义回调参数  
   ParamSynthesizeCallBack* cbParam = new ParamSynthesizeCallBack;
   cbParam->binAudioFile = tst->audioFile;
   cbParam->audioFile.open(cbParam->binAudioFile.c_str(), std::ios::binary | std::ios::out);
   // 2：创建语音识别SpeechSynthesizerRequest对象 
    SpeechSynthesizerRequest* request = NlsClient::getInstance()->createSynthesizerRequest();
    if (request == NULL) {   
        printf("createSynthesizerRequest failed.\n");   
        cbParam->audioFile.close();   
        return NULL; 
    }
    request->setOnSynthesisCompleted(OnSynthesisCompleted, cbParam); // 设置音频合成结束回调函数 
    request->setOnChannelClosed(OnSynthesisChannelClosed, cbParam); // 设置音频合成通道关闭回调函数 
    request->setOnTaskFailed(OnSynthesisTaskFailed, cbParam); // 设置异常失败回调函数    
    request->setOnBinaryDataReceived(OnBinaryDataRecved, cbParam); // 设置文本音频数据接收回调函数
    request->setOnMetaInfo(OnMetaInfo, cbParam); // 设置字幕信息 
    request->setAppKey(tst->appkey.c_str()); 
    request->setText(tst->text.c_str()); // 设置待合成文本，必填参数。文本内容为UTF-8编码。  
    request->setVoice("siqi");              // 发音人，可选参数，默认是xiaoyun。 
    request->setVolume(50);              // 音量，范围是0~100，可选参数，默认50。 
    request->setFormat("wav");             // 音频编码格式，可选参数，默认是WAV。支持：PCM/WAV/MP3。 
    request->setSampleRate(8000);          // 音频采样率，支持8000/16000。可选参数，默认是16000。  
    request->setSpeechRate(0);              // 语速，范围是-500~500。可选参数，默认是0。  
    request->setPitchRate(0);              // 语调，范围是-500~500。可选参数，默认是0。
    //request->setEnableSubtitle(true);      //是否开启字幕，可选。注意并不是所有发音人都支持字幕功能。 
    request->setToken(tst->token.c_str()); // 设置账号校验token，必填参数。 
    cbParam->startMs = getNow(); 

    // 3：start()为异步操作。成功返回BinaryRecv事件，失败返回TaskFailed事件。
    if (request->start() < 0) {
        printf("start() failed. may be can not connect server. please check network or firewalld\n");   
        NlsClient::getInstance()->releaseSynthesizerRequest(request); // 调用start()失败，释放request对象。  
        cbParam->audioFile.close();
        return NULL;
    } 
    //4: 通知云端数据发送结束。
    //stop()为异步操作，失败返回TaskFailed事件。  
    request->stop();     //5: 识别结束，释放request对象。
    NlsClient::getInstance()->releaseSynthesizerRequest(request); 
    return NULL; 
}
