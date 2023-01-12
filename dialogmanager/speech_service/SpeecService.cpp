#include <string>
#include <ctime>
#include <thread>
#include <sys/time.h>
#include <tools/jsonwrap.hpp>
#include <curl/curl.h>
#include "SpeecService.h"
#include "nlsClient.h"
#include "nlsEvent.h"
#include "nlsToken.h"
#include "speechSynthesizer.h"
#include "tools/timermanager.hpp"
#include "log.h"
#include "speechRestfulAsr.h"

using namespace AlibabaNlsCommon;
using AlibabaNls::NlsClient;
using AlibabaNls::NlsEvent;
using AlibabaNls::LogDebug;
using AlibabaNls::LogInfo;

//全局维护一个服务鉴权token和其对应的有效期时间戳，
//每次调用服务之前，首先判断token是否已经过期。
//如果已经过期，则根据AccessKey ID和AccessKey Secret重新生成一个token，并更新这个全局的token和其有效期时间戳。
//获取Token具体操作，请参见：https://help.aliyun.com/document_detail/450514.html
//说明：只需在token即将过期时进行重新生成。所有的服务并发可共用一个token。
std::string g_akId = "";
std::string g_akSecret = "";
std::string g_token = "";
long g_expireTime = -1;

int generateToken(std::string akId, std::string akSecret, std::string* token, long* expireTime) {
    NlsToken nlsTokenRequest;
    nlsTokenRequest.setAccessKeyId(akId);
    nlsTokenRequest.setKeySecret(akSecret);
    if (-1 == nlsTokenRequest.applyNlsToken()) {
        // 获取失败原因。
        printf("generateToken Failed: %s\n", nlsTokenRequest.getErrorMsg());
        return -1;
    }
    *token = nlsTokenRequest.getToken();
    *expireTime = nlsTokenRequest.getExpireTime();
    return 0;
}

uint64_t getNow() {
    struct timeval now;
    gettimeofday(&now, NULL);
    return now.tv_sec * 1000 * 1000 + now.tv_usec;
}

SpeecService *SpeecService::GetInstance() {
    static SpeecService instance;
    return &instance;
}

SpeecService::SpeecService() {
    auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json").value();
    appkey_ = config["ali_nls"]["appkey"].GetString();
    g_akId = config["ali_nls"]["access_key_id"].GetString();
    g_akSecret = config["ali_nls"]["access_key_secret"].GetString();
// 根据需要设置SDK输出日志，可选。此处表示SDK日志输出至log-Transcriber.txt，LogDebug表示输出所有级别日志。
    int ret = NlsClient::getInstance()->setLogConfig("log-transcriber", LogDebug);
    if (-1 == ret) {
        printf("set log failed\n");
        return;
    }
    // 全局只初始化一次
    curl_global_init(CURL_GLOBAL_ALL);

    Start();
}

SpeecService::~SpeecService() {
    NlsClient::releaseInstance();
    curl_global_cleanup();
}


bool SpeecService::Start() {
    //启动工作线程
    NlsClient::getInstance()->startWorkThread(4);     // 识别单个音频数据

    struct TokenTimer{};
    TokenTimer tt;
    auto tm = TimerManager<TokenTimer>::GetInstance();
    tm->AddAlarm(std::chrono::system_clock::now(), tt, [this]{
        this->onTimer();
    }, std::chrono::seconds(10));

    for(int i=0;i<10;i++)
    {
        if(!g_token.empty())
            break;
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

void SpeecService::onTimer() {
    // 获取当前系统时间戳，判断token是否过期。
    std::time_t curTime = std::time(0);
    std::string token = "";
    long expireTime = -1;

    if (g_expireTime - curTime <= 10) {
        LOGGER->info("the token will be expired, please generate new token by AccessKey-ID and AccessKey-Secret.");
        if (-1 == generateToken(g_akId, g_akSecret, &token, &expireTime)) {
            return;
        }
        pthread_rwlock_wrlock(&token_rwlock_);
        g_token = token;
        pthread_rwlock_unlock(&token_rwlock_);
        g_expireTime = expireTime;
    }
}

std::tuple<int, std::string>
SpeecService::SpeechTranscribeFile(uuid_t id, const std::string &file_name, const std::string &format, int sample_rate) {
    ParamTranscribe pa;
    pthread_rwlock_rdlock(&token_rwlock_);
    pa.token = g_token;
    pthread_rwlock_unlock(&token_rwlock_);
    pa.appkey = appkey_;
    pa.fileName = file_name;
    pa.fileFormat = format;
    pa.sampleRate = sample_rate;
    pa.cbData.userId = id;

    pthreadTranscriber((void *) (&pa));

    return std::make_tuple(1, pa.cbData.recognizedContent);
}

std::tuple<int, std::string>
SpeecService::RestfulAsr(uuid_t id, const std::string &file_name, const std::string &format, int sample_rate) {
    pthread_rwlock_rdlock(&token_rwlock_);
    std::string token = g_token;
    pthread_rwlock_unlock(&token_rwlock_);
    std::string appKey = appkey_;

    ParamRestfulAsrCallBack cb;
    cb.userId = id;

    int ret = restfulAsrProcess(appKey, token, file_name, format, sample_rate, &cb);
    return std::make_tuple(ret, cb.recognizedContent);
}

int SpeecService::OnFileTranscribed(uuid_t id, const std::string &content) {
    return 0;
}

int SpeecService::SpeechSynthesizeText(uuid_t id, const std::string &text) {
    ParamSynthesize pa;
    pthread_rwlock_rdlock(&token_rwlock_);
    pa.token = g_token;
    pthread_rwlock_unlock(&token_rwlock_);
    pa.appkey = appkey_;
    // 说明：Windows平台下，合成文本中如果包含中文，请将本CPP文件设置为带签名的UTF-8编码或者GB2312编码。
    pa.text = text;
    pa.audioFile = std::to_string(id) + "_" + std::to_string(getNow()) + ".wav";

    std::thread async_call([id, pa, this]() {
        pthreadSynthesizer((void*)(&pa));
        this->OnTextSynthesized(id, pa.audioFile);
    });
    async_call.detach();

    return 0;
}

int SpeecService::OnTextSynthesized(uuid_t id, const std::string &content) {
    return 0;
}
