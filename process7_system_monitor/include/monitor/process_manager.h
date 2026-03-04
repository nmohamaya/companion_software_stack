// process7_system_monitor/include/monitor/process_manager.h
// Phase 3 (#91) — Process supervisor: fork+exec, crash detection, restart.
//
// The ProcessManager lives inside System Monitor (P7). When launched with
// --supervised, P7 fork+execs the other 6 processes, reaps zombies via
// waitpid(WNOHANG) polling in tick(), and restarts dead children.
//
// Design decisions:
//   - fork+exec (not system/popen) for proper PID tracking
//   - waitpid(WNOHANG) polling in tick(), not SIGCHLD handler — simpler,
//     avoids async-signal-safety issues, good enough at 1 Hz scan rate
//   - SIGTERM → children on shutdown, SIGKILL after timeout
//   - Environment inherited from parent (LD_LIBRARY_PATH, Zenoh config)
//   - No allocations in the reap/restart hot path
#pragma once

#include "util/safe_name_copy.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
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
    char         name[32]         = {};
    char         binary_path[256] = {};
    char         args[256]        = {};
    pid_t        pid              = -1;
    ProcessState state            = ProcessState::STOPPED;
    uint32_t     restart_count    = 0;
    uint64_t     last_restart_ns  = 0;      // steady_clock timestamp
    uint64_t     started_at_ns    = 0;      // steady_clock timestamp
    int          last_exit_code   = 0;      // wstatus from waitpid
    bool         was_signaled     = false;  // true if killed by signal
    int          last_signal      = 0;      // signal number if was_signaled
};

// ── Restart policy (simple for Phase 3; Phase 4 adds config-driven) ──
struct RestartPolicy {
    uint32_t max_restarts       = 5;
    uint32_t cooldown_window_s  = 60;  // Reset counter after N seconds stable
    uint32_t initial_backoff_ms = 500;
    uint32_t max_backoff_ms     = 30000;
};

// ── Death callback type ─────────────────────────────────────
// Called when tick() detects a child death.  Args: name, exit_code, signal.
using DeathCallback = std::function<void(const char* name, int exit_code, int signal_num)>;

// ── ProcessManager ──────────────────────────────────────────
class ProcessManager {
public:
    explicit ProcessManager(RestartPolicy policy = {}) : policy_(policy) {}

    ~ProcessManager() { stop_all(); }

    // Non-copyable, non-movable (owns child PIDs)
    ProcessManager(const ProcessManager&)            = delete;
    ProcessManager& operator=(const ProcessManager&) = delete;
    ProcessManager(ProcessManager&&)                 = delete;
    ProcessManager& operator=(ProcessManager&&)      = delete;

    /// Register a process to be managed.
    void add_process(const char* name, const char* binary_path, const char* args = "") {
        ManagedProcess proc{};
        drone::util::safe_name_copy(proc.name, name);
        drone::util::safe_name_copy(proc.binary_path, binary_path);
        drone::util::safe_name_copy(proc.args, args);
        processes_.push_back(proc);
    }

    /// Set callback for death events.
    void set_death_callback(DeathCallback cb) { death_cb_ = std::move(cb); }

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

            // Check if RESTARTING processes are ready to re-launch
            if (proc.state == ProcessState::RESTARTING) {
                uint64_t backoff_ns = compute_backoff_ns(proc);
                if (now_ns - proc.last_restart_ns >= backoff_ns) {
                    spdlog::info("[Supervisor] Restarting {} (attempt {}/{})", proc.name,
                                 proc.restart_count + 1, policy_.max_restarts);
                    launch_one(proc);
                }
                continue;  // Skip cooldown check for just-launched processes
            }

            // Reset restart counter if process has been stable for cooldown_window
            if (proc.state == ProcessState::RUNNING && proc.restart_count > 0 &&
                proc.started_at_ns > 0 && now_ns > proc.started_at_ns) {
                uint64_t stable_ns   = now_ns - proc.started_at_ns;
                uint64_t cooldown_ns = static_cast<uint64_t>(policy_.cooldown_window_s) *
                                       1'000'000'000ULL;
                if (stable_ns >= cooldown_ns) {
                    spdlog::info("[Supervisor] {} stable for {}s — resetting restart counter",
                                 proc.name, policy_.cooldown_window_s);
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

            // Schedule restart or mark FAILED
            if (proc->restart_count >= policy_.max_restarts) {
                proc->state = ProcessState::FAILED;
                spdlog::error("[Supervisor] {} FAILED — max restarts ({}) exhausted", proc->name,
                              policy_.max_restarts);
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

    /// Get current state of all managed processes (copy for thread safety).
    std::vector<ManagedProcess> get_all() const { return processes_; }

    /// Get count of registered processes.
    size_t size() const { return processes_.size(); }

    /// Find a process by name (const).
    const ManagedProcess* find(const char* name) const {
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

private:
    std::vector<ManagedProcess> processes_;
    RestartPolicy               policy_;
    DeathCallback               death_cb_;
    bool                        stopping_ = false;

    ManagedProcess* find_by_pid(pid_t pid) {
        for (auto& proc : processes_) {
            if (proc.pid == pid) return &proc;
        }
        return nullptr;
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

        proc.pid           = pid;
        proc.state         = ProcessState::RUNNING;
        proc.started_at_ns = now_ns;

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
                int fd = std::atoi(entry->d_name);
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
    uint64_t compute_backoff_ns(const ManagedProcess& proc) const {
        uint32_t backoff_ms = policy_.initial_backoff_ms;
        for (uint32_t i = 0; i < proc.restart_count && backoff_ms < policy_.max_backoff_ms; ++i) {
            backoff_ms = std::min(backoff_ms * 2, policy_.max_backoff_ms);
        }
        return static_cast<uint64_t>(backoff_ms) * 1'000'000ULL;
    }
};

}  // namespace drone::monitor
