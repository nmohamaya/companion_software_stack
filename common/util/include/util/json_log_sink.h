// common/util/include/util/json_log_sink.h
// Structured JSON spdlog sink for machine-readable log output.
//
// Each log line is emitted as a single JSON object (one per line / JSON Lines):
//   {"ts":"2026-03-03T12:34:56.789012","level":"info","logger":"perception",
//    "thread":12345,"msg":"Pipeline started","pid":9876}
//
// Usage:
//   auto sink = std::make_shared<JsonLogSink_mt>();
//   auto logger = std::make_shared<spdlog::logger>("mylogger", sink);
//   spdlog::set_default_logger(logger);
//   spdlog::info("Hello");   // → JSON line on stdout
//
// This sink ignores spdlog patterns — it always produces JSON.
// For file output, wrap an std::ofstream-based sink or use JsonFileSink.
#pragma once

#include "util/correlation.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <mutex>
#include <string>
#include <string_view>

#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

namespace drone::util {

namespace detail {

/// Escape a string for JSON output (handles \, ", \n, \r, \t, control chars).
/// Writes directly to the output buffer for speed — no intermediate std::string.
inline void json_escape(std::string& out, std::string_view input) {
    out.reserve(out.size() + input.size() + 16);
    for (char c : input) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    // Control character → \u00XX
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
                    out += buf;
                } else {
                    out += c;
                }
                break;
        }
    }
}

/// Format a time_point as ISO 8601 with microseconds.
/// Example: "2026-03-03T12:34:56.789012"
inline std::string format_timestamp(const std::chrono::system_clock::time_point& tp) {
    auto    tt = std::chrono::system_clock::to_time_t(tp);
    std::tm tm{};
    gmtime_r(&tt, &tm);

    auto us = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch()) %
              std::chrono::seconds(1);

    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%06ld", tm.tm_year + 1900,
                  tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec,
                  static_cast<long>(us.count()));
    return buf;
}

/// Convert spdlog level to a short string.
inline const char* level_to_str(spdlog::level::level_enum level) {
    switch (level) {
        case spdlog::level::trace: return "trace";
        case spdlog::level::debug: return "debug";
        case spdlog::level::info: return "info";
        case spdlog::level::warn: return "warn";
        case spdlog::level::err: return "error";
        case spdlog::level::critical: return "critical";
        default: return "unknown";
    }
}

}  // namespace detail

/// spdlog sink that emits each log message as a single JSON line to a FILE*.
/// Thread safety is provided by the Mutex template parameter.
template<typename Mutex>
class JsonLogSink : public spdlog::sinks::base_sink<Mutex> {
public:
    /// @param output  Destination file handle (default: stdout). Not owned.
    explicit JsonLogSink(std::FILE* output = stdout) : output_(output) {}

    /// Get the last formatted JSON line (useful for testing).
    /// Returns a copy to avoid data races when called concurrently with logging.
    [[nodiscard]] std::string last_json() {
        std::lock_guard<Mutex> lock(this->mutex_);
        return last_json_;
    }

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        std::string json;
        json.reserve(256);

        // Build JSON object
        json += "{\"ts\":\"";
        json += detail::format_timestamp(msg.time);

        json += "\",\"level\":\"";
        json += detail::level_to_str(msg.level);

        json += "\",\"logger\":\"";
        detail::json_escape(json, std::string_view(msg.logger_name.data(), msg.logger_name.size()));

        json += "\",\"thread\":";
        json += std::to_string(msg.thread_id);

        json += ",\"pid\":";
        json += std::to_string(static_cast<int>(getpid()));

        json += ",\"msg\":\"";
        detail::json_escape(json, std::string_view(msg.payload.data(), msg.payload.size()));

        json += "\"";

        // Correlation ID (from thread-local context, omitted when 0)
        auto cid = CorrelationContext::get();
        if (cid != 0) {
            json += ",\"correlation_id\":\"";
            // Format as hex for readability (matches spdlog {:#x} format)
            char cid_buf[32];
            std::snprintf(cid_buf, sizeof(cid_buf), "0x%016lx", static_cast<unsigned long>(cid));
            json += cid_buf;
            json += "\"";
        }

        // Source location (if available)
        if (!msg.source.empty()) {
            json += ",\"src\":{\"file\":\"";
            detail::json_escape(json, msg.source.filename);
            json += "\",\"line\":";
            json += std::to_string(msg.source.line);
            json += ",\"func\":\"";
            detail::json_escape(json, msg.source.funcname);
            json += "\"}";
        }

        json += "}\n";

        last_json_ = json;
        std::fwrite(json.data(), 1, json.size(), output_);
    }

    void flush_() override { std::fflush(output_); }

private:
    std::FILE*  output_ = stdout;
    std::string last_json_;
};

/// Thread-safe (multi-threaded) JSON sink.
using JsonLogSink_mt = JsonLogSink<std::mutex>;

/// Single-threaded JSON sink (no locking).
using JsonLogSink_st = JsonLogSink<spdlog::details::null_mutex>;

}  // namespace drone::util
