#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"
#include "tools/jsonwrap.hpp"
#include "cinatra.hpp"

#define SPDLOG_FILENAME "log/DialogerManagerLogger.log"
#define SPDLOGGERNAME "DialogerManagerLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

void initspdlog()
{
    spdlog::flush_every(std::chrono::seconds(5));
    auto file_logger = spdlog::rotating_logger_mt<spdlog::async_factory>(SPDLOGGERNAME, SPDLOG_FILENAME, 1024 * 1024 * 200, 5);
    LOGGER->set_level(spdlog::level::info); // Set global log level to info
    LOGGER->set_pattern("[%Y-%m-%d %H:%M:%S.%e %^%L%$ %t] %v");
}

int main(int argc,char **argv)
{
    initspdlog();
    auto config = JsonSimpleWrap::GetPaser("conf/dialog_manager_config.json");
    if(config = std::nullopt)
        LOGGER->info("aa");
    int max_thread_num = 1;
    cinatra::http_server server(max_thread_num);
    server.listen((*config)["httpserver_setting"]["host"].GetString(), (*config)["httpserver_setting"]["port"].GetString());
    return 0;
}