#ifndef HTTP_H
#define HTTP_H
#include <curl/curl.h>
#include <iostream>
#include <string.h>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "spdlog/spdlog.h"
#include <openssl/sha.h>
#include <openssl/md5.h>
#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)
class HttpRequester
{
public:
    HttpRequester() = default;
    virtual ~HttpRequester() {}
    static std::string PostUrl(const std::string &url,
                                const std::string &post_param,
                                bool json_type=false,
                                curl_slist *header = nullptr,
                                std::string authorization = "",
                                std::string secret_key = "");
    static std::string PostUrlNlp(const std::string &url,
                               const std::string &post_param,
                               bool json_type=false,
                               curl_slist *header = nullptr,
                               std::string authorization = "");
    static std::string GetUrl(const std::string &url, curl_slist *header);
    static std::string GenStr(uint8_t len);
    static std::string MD5(const std::string &src);
    static std::string Str2SHA256(const std::string &srcStr);
    static int GetResponseData(const std::string &response, rapidjson::Value *&data, rapidjson::Document &doc);
    int ParseReponseStatus(std::string &response);
private:
    // static size_t ProcessData(void *data, size_t size, size_t nmemb,
    //                          std::string &content);
    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);
};
#endif