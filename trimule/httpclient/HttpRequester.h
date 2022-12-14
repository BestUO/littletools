#pragma once

#include <curl/curl.h>
#include <iostream>
#include <string.h>
#include <openssl/sha.h>
#include <openssl/md5.h>

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
    static std::string GetUrl(const std::string &url, curl_slist *header);
    static std::string GenStr(uint8_t len);
    static std::string MD5(const std::string &src);
    static std::string Str2SHA256(const std::string &srcStr);
    static std::string PostUrl2(std::string keyname, std::string signatureKey, const std::string &url, const std::string &post_param, bool usepost);
private:
    // static size_t ProcessData(void *data, size_t size, size_t nmemb,
    //                          std::string &content);
    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);
    template <typename... Args>
    static std::string GetParas(std::string params, Args... args);
};