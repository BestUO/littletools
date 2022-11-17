#include "HttpRequester.h"
#include <algorithm>

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

// size_t HttpRequester::ProcessData(void *data, size_t size, size_t nmemb, std::string &content)
// {
//     size_t sizes = size * nmemb;
//     std::string temp;
//     temp = std::string((char*)data, sizes);
//     content += temp;
//     return sizes;
// }

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

    LOGGER->info("GetUrl   response is {}",response);
    
    return response;
}

std::string HttpRequester::PostUrl(const std::string &url, const std::string &post_param, bool json_type, curl_slist *header, std::string authorization, std::string secret_key)
{
    CURL *curl;
    CURLcode res;
    curl = curl_easy_init();

    std::string response;
    LOGGER->info("post url is {}",url);
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
        LOGGER->info("post url : {} {}, rescode : {}",url,post_param,res);
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    LOGGER->info("PostUrl response {}" , response);
    
    return response;
}
std::string HttpRequester::PostUrlNlp(const std::string &url, const std::string &post_param, bool json_type, curl_slist *header, std::string authorization)
{
    CURL *curl;
    CURLcode res;
    curl = curl_easy_init();

    std::string response;

    struct curl_slist* headers = header;
    if (curl)
    {
        if(json_type)
            headers=curl_slist_append(headers, "Content-Type:application/json;charset=UTF-8");
        if(!authorization.empty())
            curl_slist_append(headers, ("Authorization:Bearer "+authorization).c_str());

        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_param.c_str());
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        //do not verify ssl certificate
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);

        //timeout
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 0.5);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 0.5);

        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);

        res = curl_easy_perform(curl);
        LOGGER->info("post url : {}  , post_param is  {}  rescode :  {}",url,post_param,res);
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    LOGGER->info("response {}", response);

    return response;
}
int HttpRequester::GetResponseData(const std::string &response,rapidjson::Value *&data, rapidjson::Document &doc)
{
    doc.Parse(response.c_str());
    
    if(!doc.IsObject())
    {
        LOGGER->info("response format in not valid : ");
        return -1;
    }

    if(doc.HasMember("data"))
    {
        auto &data_member = doc["data"];
        if(!data_member.IsNull())
        {
            data = &data_member;
        }
    }
    
    if(doc.HasMember("status"))
    {
        auto &status = doc["status"];
        if(status.IsInt())
        {
            return status.GetInt();
        }
    }
    return -1;
}

int HttpRequester::ParseReponseStatus(std::string &response)
{
	rapidjson::Document doc;
    doc.Parse(response.c_str());

    if(!doc.IsObject() || !doc.HasMember("retcode"))
    {
        // LOG(WARNING)<< "response format in not valid : ";
        response = "<invalid response format>";
        return -1;
    }
    auto &retcode = doc["retcode"];
    if(retcode.IsInt())
    {
        return retcode.GetInt();
    }else if(retcode.IsString())
    {
        return std::stoi(retcode.GetString());
    }
    return -1;
}