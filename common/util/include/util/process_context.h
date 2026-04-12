// common/util/include/util/process_context.h
// ProcessContext — shared boilerplate for all process main() functions.
//
// Every process has the same 20-line startup sequence: parse args, install
// signal handler, init logging, load config, validate, create message bus.
// init_process() captures that in one call, returning a ready-to-use context.
//
// Usage:
//   static std::atomic<bool> g_running{true};
//
//   int main(int argc, char* argv[]) {
//       auto ctx = drone::util::init_process(
//           argc, argv, "payload_manager", g_running,
//           drone::util::payload_manager_schema());
//       if (!ctx.is_ok()) return ctx.error();
//       auto& pc = ctx.value();
//
//       // ... domain-specific wiring using pc.cfg and pc.bus ...
//       while (g_running.load(std::memory_order_acquire)) { ... }
//       drone::systemd::notify_stopping();
//       return 0;
//   }
//
// See: Issue #291 (Epic #284 — Platform Modularity)
#pragma once

#include "ipc/message_bus.h"
#include "ipc/message_bus_factory.h"
#include "ipc/zenoh_liveliness.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/config_validator.h"
#include "util/ilogger.h"
#include "util/log_config.h"
#include "util/result.h"
#include "util/sd_notify.h"
#include "util/signal_handler.h"

#include <atomic>
#include <string>

#include <unistd.h>

namespace drone::util {

/// Shared context produced by init_process().
/// Holds the common infrastructure every process needs after startup.
/// Move-only (MessageBus is non-copyable).
struct ProcessContext {
    std::string          process_name;
    Config               cfg;
    ipc::MessageBus      bus;
    std::atomic<bool>&   running;  // reference to the process's g_running flag
    ipc::LivelinessToken liveliness;

    // Parsed command-line args
    ParsedArgs args;

    // Non-copyable, move-constructible only.
    // Move-assignment is deleted because `running` is a reference member
    // (references cannot be rebound). The defaulted move-assign would be
    // implicitly deleted by the compiler anyway — we make it explicit.
    ProcessContext(ProcessContext&&) noexcept        = default;
    ProcessContext& operator=(ProcessContext&&)      = delete;
    ProcessContext(const ProcessContext&)            = delete;
    ProcessContext& operator=(const ProcessContext&) = delete;

private:
    friend Result<ProcessContext, int> init_process(int argc, char* argv[], const std::string& name,
                                                    std::atomic<bool>&  running,
                                                    const ConfigSchema& schema);

    /// Private constructor — only init_process() can create a ProcessContext.
    ProcessContext(std::string name_arg, Config cfg_arg, ipc::MessageBus bus_arg,
                   std::atomic<bool>& running_ref, ipc::LivelinessToken token,
                   ParsedArgs parsed_args)
        : process_name(std::move(name_arg))
        , cfg(std::move(cfg_arg))
        , bus(std::move(bus_arg))
        , running(running_ref)
        , liveliness(std::move(token))
        , args(std::move(parsed_args)) {}
};

/// Initialize the common boilerplate for any process.
///
/// Performs (in order):
///   1. parse_args()
///   2. SignalHandler::install()
///   3. LogConfig::init()
///   4. Config::load(); if loading succeeds, validate_or_exit(); if config is
///      missing, log a warning and continue with defaults (validation skipped)
///   5. create_message_bus()
///   6. LivelinessToken declaration
///
/// @param argc, argv    Command-line arguments.
/// @param name          Process name (e.g. "payload_manager").
/// @param running       Reference to the process's atomic shutdown flag.
/// @param schema        ConfigSchema for validation.
/// @return Result<ProcessContext, int> — ok with context, or err with exit code.
[[nodiscard]] inline Result<ProcessContext, int> init_process(int argc, char* argv[],
                                                              const std::string&  name,
                                                              std::atomic<bool>&  running,
                                                              const ConfigSchema& schema) {
    auto args = parse_args(argc, argv, name.c_str());
    if (args.help) {
        return Result<ProcessContext, int>::err(0);
    }

    SignalHandler::install(running);
    LogConfig::init(name, LogConfig::resolve_log_dir(), args.log_level, args.json_logs);

    Config cfg;
    if (!cfg.load(args.config_path)) {
        DRONE_LOG_WARN("Running with default configuration; failed to load '{}'", args.config_path);
    } else if (args.skip_validation) {
        DRONE_LOG_WARN("Config validation skipped (--skip-validation)");
    } else {
        if (int rc = validate_or_exit(cfg, schema); rc != 0) {
            return Result<ProcessContext, int>::err(rc);
        }
    }

    DRONE_LOG_INFO("=== {} starting (PID {}) ===", name, getpid());

    auto                 bus = ipc::create_message_bus(cfg);
    ipc::LivelinessToken liveliness(name);

    return Result<ProcessContext, int>::ok(ProcessContext(
        name, std::move(cfg), std::move(bus), running, std::move(liveliness), std::move(args)));
}

}  // namespace drone::util
