#pragma once
#include "spdlog/spdlog.h"

#define SPDLOG_FILENAME "log/DialogerManagerLogger.log"
#define SPDLOGGERNAME "DialogerManagerLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)
