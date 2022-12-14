#include "HttpRequester.h"
#include "json/json.h"
#include "global.hpp"
#include <algorithm>
#include <openssl/sha.h>
#include <openssl/md5.h>

std::string HttpRequester::MD5(const std::string &src)
{
    MD5_CTX ctx;

    std::string md5_string;
    unsigned char md[16] = { 0 };
    char tmp[33] = { 0 };

    MD5_Init( &ctx );
    MD5_Update( &ctx, src.c_str(), src.size() );
    MD5_Final( md, &ctx );

    for( int i = 0; i < 16; ++i )
    {
        memset( tmp, 0x00, sizeof( tmp ) );
        sprintf( tmp, "%02x", md[i] );
        md5_string += tmp;
    }
    return md5_string;
}

size_t HttpRequester::WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

std::string HttpRequester::Str2SHA256(const std::string &srcStr)  
{
    // 调用sha256哈希    
    unsigned char mdStr[33] = {0};  
    SHA256((const unsigned char *)srcStr.c_str(), srcStr.length(), mdStr);  
  
    // 哈希后的字符串    
    std::string encodedStr = std::string((const char *)mdStr);  
    // 哈希后的十六进制串 32字节    
    char buf[65] = {0};  
    char tmp[3] = {0};  
    for (int i = 0; i < 32; i++)  
    {
        sprintf(tmp, "%02x", mdStr[i]);  
        strcat(buf, tmp);  
    }
    std::string encodedHexStr = std::string(buf);
    std::transform(encodedHexStr.begin(), encodedHexStr.end(), encodedHexStr.begin(), ::toupper);
    return encodedHexStr;
}  

std::string HttpRequester::GenStr(uint8_t len)
{
    char str[256]={0};
    for(int i=0;i<len && len < 256;++i)
    {
        switch((rand()%3))
        {
            case 1:
                str[i]='A'+rand()%26;
                break;
            case 2:
                str[i]='a'+rand()%26;
                break;
            default:
                str[i]='0'+rand()%10;
            break;
        }
    }
    return str;
}

std::string HttpRequester::GetUrl(const std::string &url, curl_slist *header)
{
    CURL *curl;
    CURLcode res;
    curl = curl_easy_init();

    std::string response;

    struct curl_slist* headers = header;
    if (curl)
    {
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        //do not verify ssl certificate
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);

        //timeout
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 30);

        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);

        res = curl_easy_perform(curl);
        LOGGER->info("Get url : {}, rescode : {}",url,res);
       
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    LOGGER->info("GetUrl response is {}",response);
    
    return response;
}

std::string HttpRequester::PostUrl(const std::string &url, const std::string &post_param, bool json_type, curl_slist *header, std::string authorization, std::string secret_key)
{
    CURL *curl;
    CURLcode res;
    curl = curl_easy_init();

    std::string response;
    LOGGER->info("post url is {}, params is {}",url, post_param);
    struct curl_slist* headers = header;
    if (curl)
    {
        if(json_type)
            headers=curl_slist_append(headers, "Content-Type:application/json;charset=UTF-8");
        if(!authorization.empty())
        {
            curl_slist_append(headers, ("Authorization:Bearer " + authorization).c_str());
            curl_slist_append(headers, ("Sign: " + MD5(secret_key)).c_str());
        }

        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_param.c_str());
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        //do not verify ssl certificate
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);

        //timeout
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 30);

        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);

        res = curl_easy_perform(curl);
        LOGGER->info("rescode: {} response: {}",res, response);
    }
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    return response;
}

template <typename... Args>
std::string HttpRequester::GetParas(std::string params, Args... args)
{
    std::string str;
    Json::Value root;
    auto tojson = [&](const std::string &param)
    {
        auto pos = param.find("=");
        std::string key = param.substr(0, pos);
        std::string value = param.substr(pos + 1);
        if (!key.empty() && !value.empty())
            root[key] = value;
    };

    while (true)
    {
        auto pos = params.find("&");
        tojson(params.substr(0, pos));
        if (pos == std::string::npos)
            break;
        params = params.substr(pos + 1);
    }

    std::initializer_list<int>{(tojson(args), 0)...};
    Json::Value::Members mem = root.getMemberNames();
    for (auto iter = mem.begin(); iter != mem.end(); iter++)
        str += *iter + "=" + root[*iter].asString() + "&";
    return str;
}

std::string HttpRequester::PostUrl2(std::string keyname, std::string signatureKey, const std::string &url, const std::string &post_param, bool usepost)
{
    const std::string gtime = std::to_string(time(NULL));
    const std::string gnonce = GenStr(32);
    const std::string source = "source=server.robot";
    const std::string auth_type = "auth-type=signature";
    const std::string timestamp = "timestamp=" + gtime;
    const std::string nonce = "nonce=" + gnonce;

    std::string paras = GetParas(post_param, source, auth_type, timestamp, "keyname=" + keyname, nonce);
    signatureKey = "signatureKey=" + signatureKey;
    paras += signatureKey;
    std::string signature = Str2SHA256(paras);

    struct curl_slist *header = nullptr;
    header = curl_slist_append(header, "auth-type:signature");
    header = curl_slist_append(header, "source:server.robot");
    header = curl_slist_append(header, ("timestamp:" + gtime).c_str());
    header = curl_slist_append(header, ("nonce:" + gnonce).c_str());
    header = curl_slist_append(header, ("keyname:" + keyname).c_str());
    header = curl_slist_append(header, ("signature:" + signature).c_str());
    // std::string url = "http://192.168.1.216/aicall/script/getClustersByPageParams";

    if (usepost)
        return PostUrl(url, post_param, true, header);
    else
        return GetUrl(url, header);
}