#ifndef UO_TOOLS_LOG_H
#define UO_TOOLS_LOG_H
#include "spdlog/spdlog.h"

#define SPDLOG_FILENAME "log/DialogerManagerLogger.log"
#define SPDLOGGERNAME "DialogerManagerLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

#endif //UO_TOOLS_LOG_H
