#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include "tools/jsonwrap.hpp"
#include "global.h"
#include "net_interface/net_interface.h"
#include "tools/threadpool.hpp"

#include <chrono>
#include <ctime>

std::string getTimeStr()
{
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string s(30, '\0');
    std::strftime(&s[0], s.size(), ".%Y%m%d%H%M", std::localtime(&now));
    return s;
}

void Initspdlog()
{
    spdlog::flush_every(std::chrono::seconds(5));
    auto file_logger = spdlog::rotating_logger_mt<spdlog::async_factory>(SPDLOGGERNAME, SPDLOG_FILENAME+getTimeStr(), 1024 * 1024 * 200, 5);
    file_logger->set_level(spdlog::level::info); // Set global log level to info
    file_logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e %^%L%$ %t] %v");
}

int main(int argc,char **argv)
{
    Initspdlog();
    auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json");
    if(config == std::nullopt)
    {
        LOGGER->info("dialog_manager_config.json err so return");
        return 0;
    }

    NetInterFace netinterface(config.value());
    netinterface.NetInterFaceStart();
    return 0;
}