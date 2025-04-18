#pragma once
#include <cstdio>
#include <string>

struct UserData
{
    UserData(const std::string& url, size_t start, size_t end, int fd)
        : url(url)
        , start(start)
        , end(end)
        , fd(fd)
    { }
    UserData(UserData&& other) noexcept
        : url(std::move(other.url))
        , start(other.start)
        , end(other.end)
        , fd(other.fd)
    {
        other.start = 0;
        other.end   = 0;
        other.fd    = -1;
    }
    std::string url        = "";
    size_t start           = 0;
    size_t end             = 0;
    int fd                 = -1;
    size_t timeout_seconds = 10;
    int max_retries        = 3;
};