// process7_system_monitor/include/monitor/process_manager.h
// Phase 3 (#91) + Phase 4 (#92) — Process supervisor with restart policies,
// thermal gating, cascade restarts, and stack status.
//
// The ProcessManager lives inside System Monitor (P7). When launched with
// --supervised, P7 fork+execs the other 6 processes, reaps zombies via
// waitpid(WNOHANG) polling in tick(), and restarts dead children.
//
// Phase 4 additions:
//   - Per-process RestartPolicy with is_critical + thermal_gate
//   - ProcessGraph integration for cascade restarts
//   - StackStatus computation (NOMINAL/DEGRADED/CRITICAL)
//   - Structured log events with correlation IDs
//   - Thermal gating: defers restarts when system is overheating
//
// Design decisions:
//   - fork+exec (not system/popen) for proper PID tracking
//   - waitpid(WNOHANG) polling in tick(), not SIGCHLD handler — simpler,
//     avoids async-signal-safety issues, good enough at 1 Hz scan rate
//   - SIGTERM → children on shutdown, SIGKILL after timeout
//   - Environment inherited from parent (LD_LIBRARY_PATH, Zenoh config)
//   - No allocations in the reap/restart hot path
#pragma once

#include "util/correlation.h"
#include "util/process_graph.h"
#include "util/restart_policy.h"
#include "util/safe_name_copy.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <dirent.h>
#include <signal.h>
#include <spdlog/spdlog.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace drone::monitor {

// ── Process lifecycle states ────────────────────────────────
enum class ProcessState : uint8_t {
    STOPPED,     // Not yet launched or intentionally stopped
    RUNNING,     // PID alive
    RESTARTING,  // In backoff delay before restart
    FAILED,      // Max restarts exhausted
};

inline const char* to_string(ProcessState s) {
    switch (s) {
        case ProcessState::STOPPED: return "STOPPED";
        case ProcessState::RUNNING: return "RUNNING";
        case ProcessState::RESTARTING: return "RESTARTING";
        case ProcessState::FAILED: return "FAILED";
    }
    return "UNKNOWN";
}

// ── Managed process descriptor ──────────────────────────────
struct ManagedProcess {
    char         name[32]               = {};
    char         binary_path[256]       = {};
    char         args[256]              = {};
    pid_t        pid                    = -1;
    ProcessState state                  = ProcessState::STOPPED;
    uint32_t     restart_count          = 0;
    uint64_t     last_restart_ns        = 0;      // steady_clock timestamp
    uint64_t     started_at_ns          = 0;      // steady_clock timestamp
    int          last_exit_code         = 0;      // wstatus from waitpid
    bool         was_signaled           = false;  // true if killed by signal
    int          last_signal            = 0;      // signal number if was_signaled
    bool         thermal_deferred       = false;  // true if restart is deferred due to thermal
    uint64_t     thermal_defer_start_ns = 0;      // when thermal deferral began (#183)
    char         blocked_by[32]         = {};     // cascade: wait for this process to be RUNNING
};

// ── Death callback type ─────────────────────────────────────
// Called when tick() detects a child death.  Args: name, exit_code, signal.
using DeathCallback = std::function<void(const char* name, int exit_code, int signal_num)>;

// ── ProcessManager ──────────────────────────────────────────
class ProcessManager {
public:
    /// Construct with a default policy (backward-compatible Phase 3 API).
    explicit ProcessManager(drone::util::RestartPolicy default_policy = {})
        : default_policy_(default_policy) {}

    ~ProcessManager() { stop_all(); }

    // Non-copyable, non-movable (owns child PIDs)
    ProcessManager(const ProcessManager&)            = delete;
    ProcessManager& operator=(const ProcessManager&) = delete;
    ProcessManager(ProcessManager&&)                 = delete;
    ProcessManager& operator=(ProcessManager&&)      = delete;

    /// Register a process with the default policy (Phase 3 API).
    void add_process(const char* name, const char* binary_path, const char* args = "") {
        add_process(name, binary_path, args, default_policy_);
    }

    /// Register a process with a specific per-process policy (Phase 4 API).
    void add_process(const char* name, const char* binary_path, const char* args,
                     const drone::util::RestartPolicy& policy) {
        ManagedProcess proc{};
        drone::util::safe_name_copy(proc.name, name);
        drone::util::safe_name_copy(proc.binary_path, binary_path);
        drone::util::safe_name_copy(proc.args, args);
        processes_.push_back(proc);
        policies_[proc.name] = policy;
    }

    /// Set callback for death events.
    void set_death_callback(DeathCallback cb) { death_cb_ = std::move(cb); }

    /// Set the process dependency graph for cascade restarts.
    void set_process_graph(const drone::util::ProcessGraph* graph) { graph_ = graph; }

    /// Set the current thermal zone (updated each tick from SystemHealth).
    void set_thermal_zone(uint8_t zone) { thermal_zone_ = zone; }

    /// Get current thermal zone.
    [[nodiscard]] uint8_t thermal_zone() const { return thermal_zone_; }

    /// Fork+exec all registered processes in order.
    void launch_all() {
        for (auto& proc : processes_) {
            if (proc.state == ProcessState::STOPPED) {
                launch_one(proc);
            }
        }
    }

    /// Fork+exec a single process by name.  Returns true if launched.
    bool launch(const char* name) {
        auto* proc = find(name);
        if (!proc) {
            spdlog::error("[Supervisor] Unknown process: {}", name);
            return false;
        }
        return launch_one(*proc);
    }

    /// Stop a process by name (SIGTERM, then SIGKILL after timeout).
    bool stop(const char*               name,
              std::chrono::milliseconds kill_timeout = std::chrono::milliseconds{2000}) {
        auto* proc = find(name);
        if (!proc) return false;
        return stop_one(*proc, kill_timeout);
    }

    /// Stop all managed processes.
    void stop_all(std::chrono::milliseconds kill_timeout = std::chrono::milliseconds{2000}) {
        stopping_ = true;  // Suppress restart scheduling during shutdown

        // Send SIGTERM to all RUNNING processes
        for (auto& proc : processes_) {
            if (proc.state == ProcessState::RUNNING && proc.pid > 0) {
                ::kill(proc.pid, SIGTERM);
            }
        }

        // Wait for graceful shutdown
        auto deadline = std::chrono::steady_clock::now() + kill_timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            reap_children();
            bool any_running = false;
            for (auto& proc : processes_) {
                if (proc.state == ProcessState::RUNNING && proc.pid > 0) {
                    any_running = true;
                }
            }
            if (!any_running) break;
            std::this_thread::sleep_for(std::chrono::milliseconds{50});
        }

        // SIGKILL stragglers
        for (auto& proc : processes_) {
            if (proc.state == ProcessState::RUNNING && proc.pid > 0) {
                spdlog::warn("[Supervisor] SIGKILL {} (PID {})", proc.name, proc.pid);
                ::kill(proc.pid, SIGKILL);
            }
        }
        // Final reap
        reap_children();
        for (auto& proc : processes_) {
            if (proc.pid > 0) {
                ::waitpid(proc.pid, nullptr, 0);
                proc.pid   = -1;
                proc.state = ProcessState::STOPPED;
            }
        }

        stopping_ = false;
    }

    /// Called periodically (~1 Hz): reap zombies, detect deaths, trigger restarts.
    void tick() {
        reap_children();

        for (auto& proc : processes_) {
            // Capture time fresh for each process check
            auto now    = std::chrono::steady_clock::now();
            auto now_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch())
                    .count());

            const auto& policy = get_policy(proc.name);

            // Check if RESTARTING processes are ready to re-launch
            if (proc.state == ProcessState::RESTARTING) {
                // Cascade gate: wait for the source process to recover before
                // restarting a dependant that was cascade-stopped.
                if (proc.blocked_by[0] != '\0') {
                    const auto* source = find(proc.blocked_by);
                    if (source && source->state != ProcessState::RUNNING) {
                        continue;  // Source still down — keep waiting
                    }
                    // Source recovered (or gone) — clear the gate
                    proc.blocked_by[0] = '\0';
                }

                // Thermal gate: defer restart when system is overheating
                if (policy.is_thermal_blocked(thermal_zone_)) {
                    if (!proc.thermal_deferred) {
                        proc.thermal_deferred       = true;
                        proc.thermal_defer_start_ns = now_ns;
                        auto cid                    = drone::util::CorrelationContext::generate();
                        spdlog::warn("[Supervisor] RESTART_DEFERRED_THERMAL process={} "
                                     "thermal_zone={} thermal_gate={} cid={:#018x}",
                                     proc.name, thermal_zone_, policy.thermal_gate, cid);
                    }

                    // Force restart after max deferral timeout (#183)
                    if (policy.max_thermal_defer_s > 0 && proc.thermal_defer_start_ns > 0) {
                        uint64_t defer_ns = now_ns - proc.thermal_defer_start_ns;
                        uint64_t limit_ns = static_cast<uint64_t>(policy.max_thermal_defer_s) *
                                            1'000'000'000ULL;
                        if (defer_ns >= limit_ns) {
                            spdlog::error("[Supervisor] THERMAL_DEFERRAL_TIMEOUT process={} "
                                          "deferred_for={}s max={}s — forcing restart",
                                          proc.name, defer_ns / 1'000'000'000ULL,
                                          policy.max_thermal_defer_s);
                            proc.thermal_deferred       = false;
                            proc.thermal_defer_start_ns = 0;
                            // Fall through to backoff check below
                        } else {
                            continue;  // Still within deferral window
                        }
                    } else {
                        continue;  // No timeout configured — wait indefinitely
                    }
                }

                // Clear thermal deferral flag if temperature dropped
                if (proc.thermal_deferred) {
                    proc.thermal_deferred = false;
                    spdlog::info("[Supervisor] Thermal cleared for {} — restart proceeding",
                                 proc.name);
                }

                uint64_t backoff_ns = compute_backoff_ns(proc, policy);
                if (now_ns - proc.last_restart_ns >= backoff_ns) {
                    auto     cid      = drone::util::CorrelationContext::generate();
                    uint32_t delay_ms = policy.backoff_ms(proc.restart_count);
                    spdlog::warn("[Supervisor] PROCESS_RESTART process={} attempt={}/{} "
                                 "backoff_ms={} exit_code={} signal={} cid={:#018x}",
                                 proc.name, proc.restart_count + 1, policy.max_restarts, delay_ms,
                                 proc.last_exit_code,
                                 proc.was_signaled ? strsignal(proc.last_signal) : "none", cid);
                    launch_one(proc);
                }
                continue;  // Skip cooldown check for just-launched processes
            }

            // Reset restart counter if process has been stable for cooldown_window
            if (proc.state == ProcessState::RUNNING && proc.restart_count > 0 &&
                proc.started_at_ns > 0 && now_ns > proc.started_at_ns) {
                uint64_t stable_ns   = now_ns - proc.started_at_ns;
                uint64_t cooldown_ns = static_cast<uint64_t>(policy.cooldown_window_s) *
                                       1'000'000'000ULL;
                if (stable_ns >= cooldown_ns) {
                    spdlog::info("[Supervisor] {} stable for {}s — resetting restart counter",
                                 proc.name, policy.cooldown_window_s);
                    proc.restart_count = 0;
                }
            }
        }
    }

    /// Reap zombies via waitpid(WNOHANG).
    void reap_children() {
        int   status = 0;
        pid_t pid    = 0;

        while ((pid = ::waitpid(-1, &status, WNOHANG)) > 0) {
            auto* proc = find_by_pid(pid);
            if (!proc) continue;  // Not one of ours

            // Record exit info
            if (WIFEXITED(status)) {
                proc->last_exit_code = WEXITSTATUS(status);
                proc->was_signaled   = false;
                proc->last_signal    = 0;
                spdlog::error("[Supervisor] {} (PID {}) exited with code {}", proc->name, pid,
                              proc->last_exit_code);
            } else if (WIFSIGNALED(status)) {
                proc->last_exit_code = -1;
                proc->was_signaled   = true;
                proc->last_signal    = WTERMSIG(status);
                spdlog::error("[Supervisor] {} (PID {}) killed by signal {} ({})", proc->name, pid,
                              proc->last_signal, strsignal(proc->last_signal));
            }

            proc->pid = -1;

            // Fire death callback
            if (death_cb_) {
                death_cb_(proc->name, proc->last_exit_code, proc->last_signal);
            }

            // During shutdown, set state to STOPPED (suppress restart scheduling)
            if (stopping_) {
                proc->state = ProcessState::STOPPED;
                continue;
            }

            // Handle cascade stops (Phase 4): stop downstream processes
            // that hold stale state from the dead process.
            handle_cascade_stops(proc->name);

            const auto& policy = get_policy(proc->name);

            // Schedule restart or mark FAILED
            if (proc->restart_count >= policy.max_restarts) {
                proc->state = ProcessState::FAILED;
                auto cid    = drone::util::CorrelationContext::generate();
                spdlog::error("[Supervisor] PROCESS_FAILED process={} restarts_exhausted={} "
                              "stack_status={} cid={:#018x}",
                              proc->name, policy.max_restarts,
                              drone::util::to_string(compute_stack_status()), cid);
            } else {
                proc->state = ProcessState::RESTARTING;
                auto now_ns =
                    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::steady_clock::now().time_since_epoch())
                                              .count());
                proc->last_restart_ns = now_ns;
            }
        }
    }

    /// Compute the current stack status from process states.
    [[nodiscard]] drone::util::StackStatus compute_stack_status() const {
        bool has_degraded = false;

        for (const auto& proc : processes_) {
            const auto& policy = get_policy(proc.name);

            if (proc.state == ProcessState::FAILED) {
                if (policy.is_critical) {
                    return drone::util::StackStatus::CRITICAL;
                }
                has_degraded = true;
            } else if (proc.state == ProcessState::RESTARTING) {
                has_degraded = true;
            }
        }

        return has_degraded ? drone::util::StackStatus::DEGRADED
                            : drone::util::StackStatus::NOMINAL;
    }

    /// Get total restart count across all managed processes.
    [[nodiscard]] uint32_t total_restarts() const {
        uint32_t total = 0;
        for (const auto& proc : processes_) {
            total += proc.restart_count;
        }
        return total;
    }

    /// Get current state of all managed processes (copy for thread safety).
    [[nodiscard]] std::vector<ManagedProcess> get_all() const { return processes_; }

    /// Get count of registered processes.
    [[nodiscard]] size_t size() const { return processes_.size(); }

    /// Find a process by name (const).
    [[nodiscard]] const ManagedProcess* find(const char* name) const {
        for (const auto& proc : processes_) {
            if (std::strcmp(proc.name, name) == 0) return &proc;
        }
        return nullptr;
    }

    /// Find a process by name (mutable).
    ManagedProcess* find(const char* name) {
        for (auto& proc : processes_) {
            if (std::strcmp(proc.name, name) == 0) return &proc;
        }
        return nullptr;
    }

    /// Get the policy for a given process.
    [[nodiscard]] const drone::util::RestartPolicy& get_policy(const char* name) const {
        auto key = std::string(name);
        auto it  = policies_.find(key);
        if (it != policies_.end()) return it->second;
        return default_policy_;
    }

private:
    std::vector<ManagedProcess>                                 processes_;
    drone::util::RestartPolicy                                  default_policy_;
    std::unordered_map<std::string, drone::util::RestartPolicy> policies_;
    DeathCallback                                               death_cb_;
    const drone::util::ProcessGraph*                            graph_        = nullptr;
    uint8_t                                                     thermal_zone_ = 0;
    bool                                                        stopping_     = false;

    ManagedProcess* find_by_pid(pid_t pid) {
        for (auto& proc : processes_) {
            if (proc.pid == pid) return &proc;
        }
        return nullptr;
    }

    /// Handle cascade stops: when a process dies, stop all its cascade targets.
    void handle_cascade_stops(const char* dead_process) {
        if (!graph_) return;

        auto targets = graph_->cascade_targets(dead_process);
        if (targets.empty()) return;

        spdlog::info("[Supervisor] Cascade from {} — stopping: {}", dead_process,
                     fmt::join(targets, ", "));

        for (const auto& target_name : targets) {
            auto* target = find(target_name.c_str());
            if (target && target->state == ProcessState::RUNNING) {
                stop_one(*target);
                // Mark as RESTARTING and record the source process name
                // so tick() gates re-launch on the source being RUNNING.
                target->state = ProcessState::RESTARTING;
                drone::util::safe_name_copy(target->blocked_by, dead_process);
                auto now_ns =
                    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::steady_clock::now().time_since_epoch())
                                              .count());
                target->last_restart_ns = now_ns;
            }
        }
    }

    /// Fork+exec a single process.  Returns true on success.
    bool launch_one(ManagedProcess& proc) {
        // Build argv from binary_path + args
        std::vector<std::string> arg_strings;
        arg_strings.push_back(proc.binary_path);
        if (std::strlen(proc.args) > 0) {
            split_args(proc.args, arg_strings);
        }

        std::vector<char*> argv;
        for (auto& s : arg_strings) {
            argv.push_back(s.data());
        }
        argv.push_back(nullptr);

        pid_t pid = ::fork();
        if (pid < 0) {
            spdlog::error("[Supervisor] fork() failed for {}: {}", proc.name, strerror(errno));
            return false;
        }

        if (pid == 0) {
            // ── Child process ───────────────────────────────
            // Reset signal handlers to defaults so children don't
            // inherit the supervisor's handlers.
            struct sigaction sa {};
            sa.sa_handler = SIG_DFL;
            sigemptyset(&sa.sa_mask);
            sigaction(SIGTERM, &sa, nullptr);
            sigaction(SIGINT, &sa, nullptr);
            sigaction(SIGPIPE, &sa, nullptr);

            // Close open FDs above stderr (leave 0,1,2 for stdio).
            // This prevents FD leaks from the parent.
            close_fds_above(2);

            // exec
            ::execvp(argv[0], argv.data());
            // If execvp returns, it failed
            _exit(127);
        }

        // ── Parent process ──────────────────────────────────
        auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                std::chrono::steady_clock::now().time_since_epoch())
                                                .count());

        // Increment restart count if this is a re-launch (not first launch)
        if (proc.state == ProcessState::RESTARTING) {
            proc.restart_count++;
        }

        proc.pid                    = pid;
        proc.state                  = ProcessState::RUNNING;
        proc.started_at_ns          = now_ns;
        proc.thermal_deferred       = false;
        proc.thermal_defer_start_ns = 0;
        proc.blocked_by[0]          = '\0';

        spdlog::info("[Supervisor] Launched {} (PID {})", proc.name, pid);
        return true;
    }

    /// Stop a single process: SIGTERM, wait, SIGKILL if needed.
    bool stop_one(ManagedProcess&           proc,
                  std::chrono::milliseconds kill_timeout = std::chrono::milliseconds{2000}) {
        if (proc.pid <= 0 || proc.state != ProcessState::RUNNING) {
            return false;
        }

        spdlog::info("[Supervisor] Stopping {} (PID {})", proc.name, proc.pid);
        ::kill(proc.pid, SIGTERM);

        // Poll for exit
        auto deadline = std::chrono::steady_clock::now() + kill_timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            int   status = 0;
            pid_t result = ::waitpid(proc.pid, &status, WNOHANG);
            if (result == proc.pid) {
                proc.pid   = -1;
                proc.state = ProcessState::STOPPED;
                spdlog::info("[Supervisor] {} stopped gracefully", proc.name);
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{50});
        }

        // Force kill
        spdlog::warn("[Supervisor] {} did not exit — sending SIGKILL", proc.name);
        ::kill(proc.pid, SIGKILL);
        ::waitpid(proc.pid, nullptr, 0);
        proc.pid   = -1;
        proc.state = ProcessState::STOPPED;
        return true;
    }

    /// Close file descriptors above the given minimum.
    /// Uses /proc/self/fd for efficient enumeration when available,
    /// falls back to brute-force close up to _SC_OPEN_MAX.
    static void close_fds_above(int min_fd) {
        // Fast path: enumerate /proc/self/fd (avoids closing up to 1M fds)
        DIR* dir = ::opendir("/proc/self/fd");
        if (dir) {
            int            dir_fd = ::dirfd(dir);
            struct dirent* entry  = nullptr;
            while ((entry = ::readdir(dir)) != nullptr) {
                // Skip . and ..
                if (entry->d_name[0] == '.') continue;
                int        fd  = 0;
                const auto end = entry->d_name + std::strlen(entry->d_name);
                auto [ptr, ec] = std::from_chars(entry->d_name, end, fd);
                if (ec != std::errc{} || ptr != end) continue;
                if (fd > min_fd && fd != dir_fd) {
                    ::close(fd);
                }
            }
            ::closedir(dir);
            return;
        }

        // Fallback: brute-force close
        int max_fd = static_cast<int>(::sysconf(_SC_OPEN_MAX));
        if (max_fd < 0) max_fd = 1024;
        for (int fd = min_fd + 1; fd < max_fd; ++fd) {
            ::close(fd);
        }
    }

    /// Argument splitter with minimal quoting/escaping support.
    ///
    /// - Unquoted tokens are split on spaces/tabs.
    /// - Text wrapped in single or double quotes is treated as a single argument.
    /// - Inside double quotes, backslash can escape the quote character and backslash.
    static void split_args(const char* args, std::vector<std::string>& out) {
        if (!args) return;

        std::string current;
        const char* p = args;

        while (*p) {
            // Skip leading whitespace between arguments
            while (*p == ' ' || *p == '\t') {
                ++p;
            }
            if (!*p) break;

            current.clear();

            // Handle quoted argument
            if (*p == '"' || *p == '\'') {
                char quote = *p++;
                while (*p && *p != quote) {
                    if (quote == '"' && *p == '\\') {
                        // Minimal escaping inside double quotes
                        ++p;
                        if (!*p) break;
                        current.push_back(*p++);
                    } else {
                        current.push_back(*p++);
                    }
                }
                if (*p == quote) ++p;  // consume closing quote
            } else {
                // Unquoted argument
                while (*p && *p != ' ' && *p != '\t') {
                    current.push_back(*p++);
                }
            }

            if (!current.empty()) {
                out.push_back(current);
            }
        }
    }

    /// Compute backoff in nanoseconds for the current restart attempt.
    uint64_t compute_backoff_ns(const ManagedProcess&             proc,
                                const drone::util::RestartPolicy& policy) const {
        uint32_t backoff_ms = policy.backoff_ms(proc.restart_count);
        return static_cast<uint64_t>(backoff_ms) * 1'000'000ULL;
    }
};

}  // namespace drone::monitor
