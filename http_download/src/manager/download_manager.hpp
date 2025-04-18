#pragma once

#include <cstddef>
#include <iostream>
#include <string>
#include <string.h>
#include <thread>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include "workpool/simple_workpool.hpp"
#include "global.hpp"

template <typename T>
concept FUNREQUIRE = requires(T t, const std::string url, UserData&& user_data)
{
    {
        t.GetFileSize(url)
        } -> std::convertible_to<std::tuple<bool, size_t>>;
    {
        t.DownloadChunk(std::move(user_data))
        } -> std::same_as<bool>;
};

template <FUNREQUIRE T, size_t DOWNLOAD_CHUNK_SIZE>
class DownloadManager
{
public:
    bool Download(const std::string& url,
        const std::string& output_file_name,
        uint8_t thread_num = 1)
    {
        auto [result, file_size] = __t.GetFileSize(url);
        if (!result)
        {
            return false;
        }
        std::cout << "file total size: " << file_size << " Byte" << std::endl;
        return Download(url, output_file_name, file_size, thread_num);
    }

private:
    T __t;
    SimpleWorkPool __work_pool;

    bool Download(const std::string& url,
        const std::string& output_file_name,
        size_t file_size,
        uint8_t thread_num)
    {
        size_t total_chunks
            = (file_size + DOWNLOAD_CHUNK_SIZE - 1) / DOWNLOAD_CHUNK_SIZE;
        std::atomic<bool> result = true;
        auto temp_file_name      = output_file_name + ".tmp";
        int fd
            = open(temp_file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0)
        {
            std::cerr << "create " << temp_file_name << " fail " << std::endl;
            return false;
        }
        else if (ftruncate(fd, file_size) != 0)
        {
            std::cerr << "Failed to allocate space for file: "
                      << strerror(errno) << std::endl;
            close(fd);
            return false;
        }

        for (int i = 0; i < total_chunks; ++i)
        {
            size_t start = i * DOWNLOAD_CHUNK_SIZE;
            size_t end   = (i == total_chunks - 1)
                  ? file_size - 1
                  : (i + 1) * DOWNLOAD_CHUNK_SIZE - 1;
            __work_pool.AddTask([this, &result, url, start, end, fd]() {
                result = result
                    && __t.DownloadChunk(UserData{url, start, end, fd});
            });
        }
        __work_pool.StartTask(thread_num);

        close(fd);
        if (!result)
        {
            return false;
        }
        rename(temp_file_name.c_str(), output_file_name.c_str());
        return true;
    }
};