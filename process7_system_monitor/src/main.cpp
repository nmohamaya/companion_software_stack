// process7_system_monitor/src/main.cpp
// Process 7 — System Monitor: CPU / memory / thermal monitoring, process
// watchdog.  Publishes ShmSystemHealth.

#include "ipc/message_bus_factory.h"
#include "ipc/shm_reader.h"
#include "ipc/shm_types.h"
#include "ipc/zenoh_liveliness.h"
#include "monitor/iprocess_monitor.h"
#include "monitor/process_manager.h"
#include "monitor/sys_info.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/diagnostic.h"
#include "util/log_config.h"
#include "util/process_graph.h"
#include "util/restart_policy.h"
#include "util/safe_name_copy.h"
#include "util/sd_notify.h"
#include "util/signal_handler.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <spdlog/spdlog.h>
#include <unistd.h>

static std::atomic<bool> g_running{true};

int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "system_monitor");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("system_monitor", LogConfig::resolve_log_dir(), args.log_level, args.json_logs);

    drone::Config cfg;
    if (!cfg.load(args.config_path)) {
        spdlog::warn("Running with default configuration; failed to load '{}'", args.config_path);
    }

    spdlog::info("=== System Monitor starting (PID {}) ===", getpid());

    // ── Supervised mode: fork+exec child processes ──────────
    std::unique_ptr<drone::monitor::ProcessManager> supervisor;
    drone::util::ProcessGraph                       process_graph;
    if (args.supervised) {
        spdlog::info("[Supervisor] Mode ENABLED — will fork+exec child processes");

        // Resolve binary directory from our own /proc/self/exe
        char        self_path[1024] = {};
        ssize_t     len     = ::readlink("/proc/self/exe", self_path, sizeof(self_path) - 1);
        std::string bin_dir = ".";
        if (len > 0) {
            self_path[len] = '\0';
            std::string self(self_path);
            auto        slash = self.rfind('/');
            if (slash != std::string::npos) {
                bin_dir = self.substr(0, slash);
            }
        }

        // Build extra args string (pass through config, log-level, etc.)
        std::string extra_args;
        if (args.config_path != "config/default.json") {
            extra_args += "--config " + args.config_path + " ";
        }
        if (args.log_level != "info") {
            extra_args += "--log-level " + args.log_level + " ";
        }
        if (args.simulation) extra_args += "--sim ";
        if (args.json_logs) extra_args += "--json-logs ";

        // Load per-process configs from "watchdog.processes" section
        auto proc_section = cfg.section("watchdog.processes");

        // Default process list (if config section is absent or empty)
        static const std::vector<std::string> default_process_names = {
            "video_capture", "perception",      "slam_vio_nav",
            "comms",         "mission_planner", "payload_manager",
        };

        // Parse per-process configs
        std::vector<drone::util::ProcessConfig> process_configs;
        if (proc_section.is_object() && !proc_section.empty()) {
            for (auto& [name, block] : proc_section.items()) {
                auto pc = drone::util::ProcessConfig::from_json(name, block);
                // Resolve binary path: if not absolute, prefix with bin_dir
                if (pc.binary.empty() || pc.binary[0] != '/') {
                    pc.binary = bin_dir + "/" + (pc.binary.empty() ? name : pc.binary);
                    // Strip "build/bin/" prefix if present (config uses relative paths)
                    auto prefix = std::string("build/bin/");
                    if (pc.binary.find(prefix) != std::string::npos) {
                        auto pos  = pc.binary.find(prefix);
                        pc.binary = bin_dir + "/" + pc.binary.substr(pos + prefix.size());
                    }
                }
                process_configs.push_back(std::move(pc));
            }
        } else {
            // Fallback: use defaults with hardcoded policies
            spdlog::info("[Supervisor] No watchdog.processes config — using defaults");
            for (const auto& name : default_process_names) {
                drone::util::ProcessConfig pc;
                pc.name   = name;
                pc.binary = bin_dir + "/" + name;
                // Use default RestartPolicy
                process_configs.push_back(std::move(pc));
            }
        }

        // Build the process graph and supervisor
        supervisor = std::make_unique<drone::monitor::ProcessManager>();

        for (const auto& pc : process_configs) {
            process_graph.add_process(pc.name);
        }

        // Wire dependency edges
        for (const auto& pc : process_configs) {
            for (const auto& dep : pc.launch_after) {
                process_graph.add_launch_dep(pc.name, dep);
            }
            for (const auto& target : pc.restart_cascade) {
                process_graph.add_cascade(pc.name, target);
            }
        }

        bool graph_valid = process_graph.validate();
        if (!graph_valid) {
            spdlog::error("[Supervisor] ProcessGraph validation failed — disabling "
                          "cascade logic and using default launch order");
            // Rebuild with populate_defaults() so launch_order() still works
            process_graph = drone::util::ProcessGraph{};
            for (const auto& pc : process_configs) {
                process_graph.add_process(pc.name);
            }
            process_graph.populate_defaults();
        }

        // Register processes in topological launch order
        auto launch_order = process_graph.launch_order();
        if (launch_order.empty()) {
            spdlog::warn("[Supervisor] Empty launch order — falling back to config order");
            for (const auto& pc : process_configs) {
                launch_order.push_back(pc.name);
            }
        }

        // Build name → config lookup
        std::unordered_map<std::string, const drone::util::ProcessConfig*> config_map;
        for (const auto& pc : process_configs) {
            config_map[pc.name] = &pc;
        }

        for (const auto& name : launch_order) {
            auto it = config_map.find(name);
            if (it == config_map.end()) continue;
            const auto& pc = *it->second;
            supervisor->add_process(name.c_str(), pc.binary.c_str(), extra_args.c_str(), pc.policy);
        }

        // Only enable cascade restarts when the graph is valid
        if (graph_valid) {
            supervisor->set_process_graph(&process_graph);
        }

        supervisor->set_death_callback([](const char* name, int exit_code, int signal_num) {
            if (signal_num > 0) {
                spdlog::error("[Supervisor] {} killed by signal {} ({})", name, signal_num,
                              strsignal(signal_num));
            } else {
                spdlog::error("[Supervisor] {} exited with code {}", name, exit_code);
            }
        });

        // Launch all children in topological order
        supervisor->launch_all();
        spdlog::info("[Supervisor] All child processes launched (order: {})",
                     fmt::join(launch_order, " → "));
    }

    // ── Create message bus (config-driven: shm or zenoh) ───
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Declare liveliness token (auto-dropped on exit/crash) ──
    drone::ipc::LivelinessToken liveliness_token("system_monitor");

    auto health_pub =
        bus.advertise<drone::ipc::ShmSystemHealth>(drone::ipc::shm_names::SYSTEM_HEALTH);
    if (!health_pub->is_ready()) {
        spdlog::error("Failed to create system health publisher");
        return 1;
    }

    // ── Optional: subscribe to FC state for battery info ────
    auto fc_sub = bus.subscribe_optional<drone::ipc::ShmFCState>(drone::ipc::shm_names::FC_STATE);

    // ── Process health monitoring via liveliness tokens ─────
    // Critical processes: if these die, flag critical_failure.
    static const std::vector<std::string> critical_processes = {"comms", "slam_vio_nav"};

    // Shared state for liveliness events (updated from callbacks)
    struct ProcessLiveness {
        std::mutex mutex;
        struct Event {
            std::string name;
            bool        alive;
            uint64_t    timestamp_ns;
        };
        std::vector<Event> events;
    };
    auto liveness = std::make_shared<ProcessLiveness>();

    drone::ipc::LivelinessMonitor liveliness_monitor(
        [liveness](const std::string& proc) {
            spdlog::info("[SysMon] Process ALIVE: {}", proc);
            auto now_ns =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
            std::lock_guard<std::mutex> lock(liveness->mutex);
            liveness->events.push_back({proc, true, now_ns});
        },
        [liveness](const std::string& proc) {
            spdlog::error("[SysMon] Process DIED: {}", proc);
            auto now_ns =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
            std::lock_guard<std::mutex> lock(liveness->mutex);
            liveness->events.push_back({proc, false, now_ns});
        });

    // Local map: process name → (alive, last_seen_ns)
    struct ProcessState {
        bool     alive        = false;
        uint64_t last_seen_ns = 0;
    };
    std::unordered_map<std::string, ProcessState> process_alive_map;

    spdlog::info("System Monitor READY");

    // ── systemd readiness + watchdog ─────────────────────────────
    drone::systemd::notify_ready();
    if (drone::systemd::watchdog_enabled()) {
        auto wdog_us = drone::systemd::watchdog_usec();
        spdlog::info("[systemd] Watchdog active: interval={}us ({}s)", wdog_us,
                     wdog_us / 1'000'000);
    }

    // ── Thread heartbeat + watchdog + health publisher ──────
    auto                        health_hb = drone::util::ScopedHeartbeat("health_loop", false);
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub_ch = bus.advertise<drone::ipc::ShmThreadHealth>(
        drone::ipc::shm_names::THREAD_HEALTH_SYSTEM_MONITOR);
    drone::util::ThreadHealthPublisher thread_health_publisher(*thread_health_pub_ch,
                                                               "system_monitor", watchdog);

    // Config-driven thresholds
    const float cpu_warn  = cfg.get<float>("system_monitor.thresholds.cpu_warn_percent", 90.0f);
    const float mem_warn  = cfg.get<float>("system_monitor.thresholds.mem_warn_percent", 90.0f);
    const float temp_warn = cfg.get<float>("system_monitor.thresholds.temp_warn_c", 80.0f);
    const float temp_crit = cfg.get<float>("system_monitor.thresholds.temp_crit_c", 95.0f);
    const float batt_warn = cfg.get<float>("system_monitor.thresholds.battery_warn_percent", 20.0f);
    const float batt_crit = cfg.get<float>("system_monitor.thresholds.battery_crit_percent", 10.0f);
    const float disk_crit = cfg.get<float>("system_monitor.thresholds.disk_crit_percent", 98.0f);
    const int   disk_check_s  = cfg.get<int>("system_monitor.disk_check_interval_s", 10);
    const int   update_rate   = cfg.get<int>("system_monitor.update_rate_hz", 1);
    const int   loop_sleep_ms = std::max(1, update_rate > 0 ? 1000 / update_rate : 1000);

    // Convert disk check interval from seconds to ticks (calls)
    const int disk_interval_ticks = std::max(1, disk_check_s * (update_rate > 0 ? update_rate : 1));

    // Create process monitor via strategy factory (backend from config)
    const std::string monitor_backend = cfg.get<std::string>("system_monitor.backend", "linux");
    auto              monitor         = drone::monitor::create_process_monitor(
        monitor_backend, cpu_warn, mem_warn, temp_warn, temp_crit, disk_crit, batt_warn, batt_crit,
        disk_interval_ticks);
    spdlog::info("Process monitor: {}", monitor->name());

    // Optional fault-injection override reader.
    ShmReader<drone::ipc::ShmFaultOverrides> override_reader;
    (void)override_reader.open(drone::ipc::shm_names::FAULT_OVERRIDES);  // may not exist yet

    uint32_t tick = 0;

    // ── Main loop (1 Hz) ────────────────────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(health_hb.handle());
        drone::systemd::notify_watchdog();
        tick++;

        // Incorporate battery from FC if available
        float                  battery = 100.0f;
        drone::ipc::ShmFCState fc{};
        if (fc_sub->is_connected() && fc_sub->receive(fc) && fc.connected) {
            battery = fc.battery_remaining;
        }
        monitor->set_battery_percent(battery);

        // Collect health via IProcessMonitor strategy
        // (battery thresholds are applied inside the strategy)
        drone::util::FrameDiagnostics diag(tick);
        auto                          health = [&]() {
            drone::util::ScopedDiagTimer timer(diag, "Collect");
            return monitor->collect();
        }();

        // Flag slow collection cycles (>200ms is suspicious at 1Hz)
        // The ScopedDiagTimer already recorded the timing; check it.
        constexpr double kCollectWarnMs = 200.0;
        for (const auto& entry : diag.entries()) {
            if (entry.component == "Collect" && entry.value > kCollectWarnMs) {
                diag.add_warning("Collect", "Slow health collection: " +
                                                std::to_string(static_cast<int>(entry.value)) +
                                                "ms");
            }
        }
        if (diag.has_warnings() || diag.has_errors()) {
            diag.log_summary("SystemMonitor");
        }

        // ── Merge liveliness events into health struct ──────
        {
            std::lock_guard<std::mutex> lock(liveness->mutex);
            for (auto& evt : liveness->events) {
                process_alive_map[evt.name] = {evt.alive, evt.timestamp_ns};
            }
            liveness->events.clear();
        }

        // Populate ProcessHealthEntry array
        health.num_processes    = 0;
        health.critical_failure = false;
        for (auto& [name, state] : process_alive_map) {
            if (health.num_processes < drone::ipc::kMaxTrackedProcesses) {
                auto& entry = health.processes[health.num_processes];
                drone::util::safe_name_copy(entry.name, name.c_str());
                entry.alive        = state.alive;
                entry.last_seen_ns = state.last_seen_ns;
                health.num_processes++;
            }
            if (!state.alive) {
                auto it = std::find(critical_processes.begin(), critical_processes.end(), name);
                if (it != critical_processes.end()) {
                    health.critical_failure = true;
                }
            }
        }

        // Apply fault-injection overrides (if any).
        {
            drone::ipc::ShmFaultOverrides ovr{};
            if (!override_reader.is_open()) {
                (void)override_reader.open(drone::ipc::shm_names::FAULT_OVERRIDES);
            }
            if (override_reader.is_open() && override_reader.read(ovr)) {
                if (ovr.thermal_zone >= 0) {
                    health.thermal_zone = static_cast<uint8_t>(ovr.thermal_zone);
                }
                if (ovr.cpu_temp_override >= 0.0f) {
                    health.cpu_temp_c = ovr.cpu_temp_override;
                    health.max_temp_c = ovr.cpu_temp_override;
                }
            }
        }

        health_pub->publish(health);

        // Log summary
        if (tick % 5 == 0) {
            const char* status_str = "NOMINAL";
            if (health.thermal_zone == 2)
                status_str = "WARNING";
            else if (health.thermal_zone == 3)
                status_str = "CRITICAL";

            spdlog::info(
                "[SysMon] CPU={:.1f}% MEM={:.1f}% TEMP={:.1f}°C "
                "DISK={:.0f}% BATT={:.0f}% thermal_zone={} stack={} => {}",
                health.cpu_usage_percent, health.memory_usage_percent, health.cpu_temp_c,
                health.disk_usage_percent, battery, health.thermal_zone,
                drone::util::to_string(static_cast<drone::util::StackStatus>(health.stack_status)),
                status_str);
        }

        thread_health_publisher.publish_snapshot();

        // ── Supervisor tick: reap zombies, detect deaths, restart ─
        if (supervisor) {
            // Feed current thermal zone to supervisor for thermal gating
            supervisor->set_thermal_zone(health.thermal_zone);
            supervisor->tick();

            // Update stack status and total restarts in health struct
            health.stack_status   = static_cast<uint8_t>(supervisor->compute_stack_status());
            health.total_restarts = supervisor->total_restarts();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    // ── Shutdown ─────────────────────────────────────────────
    drone::systemd::notify_stopping();
    if (supervisor) {
        spdlog::info("[Supervisor] Stopping all child processes...");
        supervisor->stop_all();
        spdlog::info("[Supervisor] All children stopped");
    }

    spdlog::info("=== System Monitor stopped ===");
    return 0;
}
