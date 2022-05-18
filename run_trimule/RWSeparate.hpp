#pragma once

#include <iostream>
#include <map>
#include <chrono>
#include <thread>
#include <uuid.h>
#include "singleton.hpp"
#include "simplewrapkafka.h"
// #include "net/include/cinatra.hpp"
// #include "net/include/cinatra/response_cv.hpp"
// #include <cinatra.hpp>
// #include <define.h>
// #include <response_cv.hpp>

// template <typename T> class connection;
// using ttt_type = std::weak_ptr<cinatra::connection<cinatra::NonSSL>>;

template <typename T>
class RWSeparate2:public Singleton<RWSeparate2<T>>
{
public:
    bool Add2Map(std::shared_ptr<cinatra::connection<T>> con, std::string &uuid)
    {
        std::unique_lock<std::mutex> lck(_mutex);
        uuid_t myuuid;
        uuid_generate(myuuid);
        char chuuid[36] = {0};
        uuid_unparse(myuuid,chuuid);
        uuid = std::string(chuuid);
        uuid = "tttt";
        auto res = _mapcon.insert({uuid, con});
        return res.second;
    }

    bool ResponseClient(std::string message)
    {
        std::unique_lock<std::mutex> lck(_mutex);
        std::string uuid("tttt");
        auto iter = _mapcon.find(uuid);
        if(iter != _mapcon.end())
        {
            std::string filename("/home/uncle_orange/BorayProduct/tools/www/1616579920094476511.jpg");
            auto file = std::make_shared<std::ifstream>(filename, std::ios::binary);

            std::string content;
            const size_t size = 5 * 1024 * 1024;
            content.resize(size);
            file->read(&content[0], size);
            int64_t read_len = (int64_t)file->gcount();

            if (read_len < size)
                content.resize(read_len);

            iter->second->get_res().set_status_and_content(cinatra::status_type::ok, std::move(content),cinatra::req_content_type::multipart);

            // iter->second->get_res().set_status_and_content(cinatra::status_type::ok, "OK");

            iter->second->response_now();
            _mapcon.erase(iter);
            return true;
        }
        return false;
    }
private:
    // int _max_thread_num = std::thread::hardware_concurrency();
    std::mutex _mutex;
    // RTE_Ring _ret_ring{"Myring",1024,false};
    std::map<std::string, std::shared_ptr<cinatra::connection<T>>> _mapcon;
};