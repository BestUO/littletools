/*
 * @Description: 公共方法头文件
 * @Author: wangli
 * @Date: 2025-07-30
*/
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <exception>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>

namespace common_utils {
    namespace func {

        /**
         * @brief 执行Linux 系统调用命令
         * @param cmd 命令
         * @param output 输出
         * @return true 成功
         * @return false 失败
        */
        inline bool execCmd(const std::string& cmd, std::string& output)
        {
            char buffer[4096] = {'\0'};
            std::string result;
            
            // 使用lambda表达式作为删除器
            auto pipe_deleter = [](FILE* p) {
                if (p) pclose(p);
            };
            
            // 显式指定删除器类型
            using PipePtr = std::unique_ptr<FILE, decltype(pipe_deleter)>;
            PipePtr pipe(popen(cmd.c_str(), "r"), pipe_deleter);
        
            if (!pipe) {
                return false;
            }
            
            while (fgets(buffer, sizeof(buffer) - 1, pipe.get()) != nullptr) {
                result += buffer;
            }
            
            // 去除末尾的换行符
            if (!result.empty() && result.back() == '\n') {
                result.pop_back();
            }
            
            output = result;
            return true;
        }

        inline bool getEthMacAddr(const std::string& eth_name, std::string& mac_addr)
        {
            std::string cmd = "ifconfig " + eth_name + " | awk '/ether / {print $2}'";
            std::string output;
            if (execCmd(cmd, output)) {
                mac_addr = output;
                return true;
            }
            return false;
        }


        inline bool getEthIpAddr(const std::string& eth_name, std::string& ip_addr)
        {
            std::string cmd = "ifconfig " + eth_name + " | awk '/inet / {print $2}'";
            std::string output;
            if (execCmd(cmd, output)) {
                ip_addr = output;
                return true;
            }
            return false;
        }

        inline bool getUUID(std::string& uuid)
        {
            //使用 cat /proc/sys/kernel/random/uuid 获取内核提供的uuid
            std::string cmd = "cat /proc/sys/kernel/random/uuid";
            std::string output;
            if (execCmd(cmd, output)) {
                uuid = output;
                return true;
            }
            return false;
        }
    }
}

