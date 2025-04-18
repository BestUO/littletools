#pragma once

#include <curl/curl.h>
#include <string>
#include <iostream>
#include <vector>
#include <tuple>
#include <thread>
#include "global.hpp"

class HttpDownload
{
public:
    HttpDownload()
    {
        curl_global_init(CURL_GLOBAL_ALL);
    }
    ~HttpDownload()
    {
        curl_global_cleanup();
    }

    std::tuple<bool, size_t> GetFileSize(const std::string& url)
    {
        CURL* curl       = curl_easy_init();
        size_t file_size = -1;
        bool result      = false;
        if (curl)
        {
            result = true;
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);
            curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, HeaderCallback);
            curl_easy_setopt(curl, CURLOPT_HEADERDATA, &file_size);
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK)
            {
                std::cerr << "cant't get file size: " << curl_easy_strerror(res)
                          << std::endl;
            }

            if (file_size == -1)
            {
                curl_off_t cl;
                res = curl_easy_getinfo(
                    curl, CURLINFO_CONTENT_LENGTH_DOWNLOAD_T, &cl);
                if (res == CURLE_OK && cl != -1)
                {
                    file_size = cl;
                }
                else
                {
                    std::cerr << "can't get file size" << std::endl;
                    result = false;
                }
            }

            curl_easy_cleanup(curl);
        }

        return {result, file_size};
    }

    bool DownloadChunk(UserData&& user_data)
    {
        CURL* curl  = curl_easy_init();
        bool result = false;
        if (curl)
        {
            std::string range = std::to_string(user_data.start) + "-"
                + std::to_string(user_data.end);
            curl_easy_setopt(curl, CURLOPT_URL, user_data.url.c_str());
            curl_easy_setopt(curl, CURLOPT_RANGE, range.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &user_data);
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, user_data.timeout_seconds);

            while (user_data.max_retries > 0)
            {
                CURLcode res = curl_easy_perform(curl);
                if (res == CURLE_OK)
                {
                    break;
                }
                else
                {
                    std::cerr << "download fail: " << curl_easy_strerror(res)
                              << ", retry " << std::endl;
                    user_data.max_retries--;
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(1000));
                }
            }
            if (user_data.max_retries == 0)
            {
                result = false;
                std::cerr << "download failed after max retries" << std::endl;
            }
            else
            {
                result = true;
            }
            curl_easy_cleanup(curl);
        }
        return result;
    }

private:
    static size_t WriteCallback(void* contents,
        size_t size,
        size_t nmemb,
        void* userp)
    {
        size_t realSize     = size * nmemb;
        UserData* user_data = static_cast<UserData*>(userp);
        off_t offset        = lseek(user_data->fd, user_data->start, SEEK_SET);
        user_data->start += realSize;
        auto r = write(user_data->fd, static_cast<char*>(contents), realSize);
        if (r < 0)
        {
            std::cerr << "write fail: " << std::endl;
        }
        return realSize;
    }

    static size_t HeaderCallback(void* contents,
        size_t size,
        size_t nmemb,
        void* userp)
    {
        size_t real_size = size * nmemb;
        std::string header(static_cast<char*>(contents), real_size);
        size_t* file_size_ptr = static_cast<size_t*>(userp);

        if (header.find("Content-Length:") != std::string::npos)
        {
            try
            {
                size_t pos           = header.find("Content-Length:");
                std::string size_str = header.substr(pos + 16);
                *file_size_ptr       = std::stoll(size_str);
            } catch (...)
            { }
        }
        return real_size;
    }
};