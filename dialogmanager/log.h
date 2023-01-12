#pragma once
#include "spdlog/spdlog.h"

#define SPDLOG_FILENAME "log/DialogerManagerLogger.log." + std::to_string(getpid())
#define SPDLOGGERNAME "DialogerManagerLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)
