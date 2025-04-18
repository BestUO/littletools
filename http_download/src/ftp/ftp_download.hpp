#pragma once

#include <tuple>
#include "global.hpp"

class FTPDownload
{
public:
    FTPDownload()
    { }
    ~FTPDownload()
    { }

    std::tuple<bool, size_t> GetFileSize(const std::string& url)
    {
        return {false, 0};
    }

    bool DownloadChunk(UserData&& user_data)
    {
        return false;
    }
};