#include <sstream>
#include <curl/curl.h>
#include <iostream>
#include <fstream>
#include <rapidjson/pointer.h>
#include "speechRestfulAsr.h"
#include "log.h"

using namespace std;

/**
 * 一句话识别RESTful API HTTP请求的响应回调函数
 * 识别结果为JSON格式的字符串
 */
size_t responseCallback(void* ptr, size_t size, size_t nmemb, void* userData) {
    string* srResult = (string*)userData;

    size_t len = size * nmemb;
    char *pBuf = (char*)ptr;
    string response = string(pBuf, pBuf + len);

    *srResult += response;
    cout << "current result: " << response << endl;
    cout << "total result: " << *srResult << endl;

    return len;
}

int sendAsrRequest(const char* request, const char* token, const char* fileName, string* srResult) {
    CURL* curl = NULL;
    CURLcode res;

    /**
    * 读取音频文件
    */
    ifstream fs;
    fs.open(fileName, ios::out | ios::binary);
    if (!fs.is_open()) {
        cerr << "The audio file is not exist!" << endl;
        return -1;
    }
    stringstream buffer;
    buffer << fs.rdbuf();
    string audioData(buffer.str());

    curl = curl_easy_init();
    if (curl == NULL) {
        return -1;
    }

    /**
    * 设置HTTP请求行
    */
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");
    curl_easy_setopt(curl, CURLOPT_URL, request);

    /**
    * 设置HTTP请求头部
    */
    struct curl_slist* headers = NULL;
    // token
    string X_NLS_Token = "X-NLS-Token:";
    X_NLS_Token += token;
    headers = curl_slist_append(headers, X_NLS_Token.c_str());
    // Content-Type
    headers = curl_slist_append(headers, "Content-Type:application/octet-stream");
    // Content-Length
    string content_Length = "Content-Length:";
    ostringstream oss;
    oss << content_Length << audioData.length();
    content_Length = oss.str();
    headers = curl_slist_append(headers, content_Length.c_str());

    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    /**
    * 设置HTTP请求数据
    */
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, audioData.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, audioData.length());

    /**
    * 设置HTTP请求的响应回调函数
    */
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, responseCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, srResult);

    /**
    * 发送HTTP请求
    */
    res = curl_easy_perform(curl);

    // 释放资源
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) {
        cerr << "curl_easy_perform failed: " << curl_easy_strerror(res) << endl;
        return -1;
    }

    return 0;
}

int restfulAsrProcess(const std::string &appKey, const std::string &token, const std::string &file_name,
                        const std::string &format, int sample_rate, ParamRestfulAsrCallBack* arg) {
    std::string url = "http://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/asr";
    bool enablePunctuationPrediction = true;
    bool enableInverseTextNormalization = true;
    bool enableVoiceDetection = false;

    /**
    * 设置RESTful请求参数
    */
    std::ostringstream oss;
    oss << url;
    oss << "?appkey=" << appKey;
    oss << "&format=" << format;
    oss << "&sample_rate=" << sample_rate;
    if (enablePunctuationPrediction) {
        oss << "&enable_punctuation_prediction=" << "true";
    }
    if (enableInverseTextNormalization) {
        oss << "&enable_inverse_text_normalization=" << "true";
    }
    if (enableVoiceDetection) {
        oss << "&enable_voice_detection=" << "true";
    }

    std::string request = oss.str();
    LOGGER->info("restfulAsrProcess userId: {}, request: {}", arg->userId, request);

    int ret = sendAsrRequest(request.c_str(), token.c_str(), file_name.c_str(), &arg->recognizedContent);
    rapidjson::Document doc;
    doc.Parse(arg->recognizedContent.c_str());
    if (!doc.HasParseError() && doc.IsObject()) {
        arg->recognizedContent = doc["result"].GetString();
    } else {
        LOGGER->error("asr result parse error:{}", arg->recognizedContent);
    }

    return ret;
}
