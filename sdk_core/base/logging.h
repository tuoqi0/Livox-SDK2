
#ifndef LIVOX_LOGGING_H_
#define LIVOX_LOGGING_H_
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

#ifdef _WIN32
#define __FILENAME__ (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1):__FILE__)
#else
#define __FILENAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1):__FILE__)
#endif

#ifndef suffix
#define suffix(msg)  std::string(msg).append("  [")\
        .append(__FILENAME__).append("] [").append(__func__)\
        .append("] [").append(std::to_string(__LINE__))\
        .append("]").c_str()
#endif

#ifndef SPDLOG_TRACE_ON
#define SPDLOG_TRACE_ON
#endif

#ifndef SPDLOG_DEBUG_ON
#define SPDLOG_DEBUG_ON
#endif

extern std::shared_ptr<spdlog::logger> logger;
extern bool is_save_log_file;
extern bool is_console_log_enable;

void InitLogger();
void UninitLogger();

#define LOG_TRACE(msg, ...) logger->trace(suffix(msg), ##__VA_ARGS__)
#define LOG_DEBUG(msg, ...) logger->debug(suffix(msg), ##__VA_ARGS__)
#define LOG_INFO(msg, ...) logger->info(suffix(msg), ##__VA_ARGS__)
#define LOG_WARN(msg, ...) logger->warn(suffix(msg), ##__VA_ARGS__)
#define LOG_ERROR(msg, ...) logger->error(suffix(msg), ##__VA_ARGS__)
#define LOG_FATAL(msg, ...) logger->critical(suffix(msg), ##__VA_ARGS__)

#endif  // LIVOX_LOGGING_H_
