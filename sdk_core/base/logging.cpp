
#include "logging.h"

std::shared_ptr<spdlog::logger> logger = NULL;
bool is_save_log_file = false;
bool is_console_log_enable = true;

void InitLogger() {

  // SDK logging is owned by this pointer, independently of the application's
  // spdlog registry. Failure cleanup must not remove application loggers.
  if (logger) {
    return;
  }

  std::vector<spdlog::sink_ptr> sinkList;
  if (is_console_log_enable) {
    auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    consoleSink->set_level(spdlog::level::debug);
    sinkList.push_back(consoleSink);
  }

  if (is_save_log_file) {
    auto rotateSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("livox_log.txt", 1024 * 1024 * 5, 2);
    rotateSink->set_level(spdlog::level::debug);
    sinkList.push_back(rotateSink);
  }

  logger = std::make_shared<spdlog::logger>("console", begin(sinkList), end(sinkList));
  logger->set_level(spdlog::level::debug);
  logger->flush_on(spdlog::level::debug);
}

void UninitLogger() {
  logger.reset();
}