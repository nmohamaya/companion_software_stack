// common/util/include/util/log_config.h
// Logging initialisation using spdlog.
#pragma once
#include <cstdlib>
#include <string>

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace LogConfig {

/// Return the log directory: $DRONE_LOG_DIR if set, otherwise `fallback`.
inline std::string resolve_log_dir(const std::string& fallback = "drone_logs") {
    const char* env = std::getenv("DRONE_LOG_DIR");
    return (env && env[0]) ? std::string(env) : fallback;
}

inline void init(const std::string& process_name, const std::string& log_dir,
                 const std::string& level_str = "info") {
    try {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto file_sink    = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_dir + "/" + process_name + ".log",
            5 * 1024 * 1024,  // 5 MB max
            3                 // 3 rotated files
        );

        auto logger = std::make_shared<spdlog::logger>(
            process_name, spdlog::sinks_init_list{console_sink, file_sink});

        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] [t:%t] %v");

        if (level_str == "trace")
            logger->set_level(spdlog::level::trace);
        else if (level_str == "debug")
            logger->set_level(spdlog::level::debug);
        else if (level_str == "info")
            logger->set_level(spdlog::level::info);
        else if (level_str == "warn")
            logger->set_level(spdlog::level::warn);
        else if (level_str == "error")
            logger->set_level(spdlog::level::err);
        else
            logger->set_level(spdlog::level::info);

        spdlog::set_default_logger(logger);
        spdlog::info("Logger '{}' initialised — level={}", process_name, level_str);
    } catch (const spdlog::spdlog_ex& ex) {
        std::fprintf(stderr, "Log init failed: %s\n", ex.what());
        // Fall back to default console logger
        spdlog::set_level(spdlog::level::info);
    }
}

}  // namespace LogConfig
