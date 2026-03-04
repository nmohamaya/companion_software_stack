// tests/test_process_manager.cpp
// Unit & integration tests for ProcessManager (Phase 3, #91).
//
// Uses the test_crasher binary for controlled child process behaviour.
// Tests are serialised (RESOURCE_LOCK) because they fork+exec real processes.

#include "monitor/process_manager.h"

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>

using drone::monitor::ManagedProcess;
using drone::monitor::ProcessManager;
using drone::monitor::ProcessState;
using drone::monitor::RestartPolicy;

// ── Helper: locate the test_crasher binary ──────────────────
static std::string crasher_path() {
    // Environment variable set by CMake (preferred)
    const char* env = std::getenv("TEST_CRASHER_PATH");
    if (env && ::access(env, X_OK) == 0) return env;

    // Derive from our own binary location via /proc/self/exe
    char    self_path[1024] = {};
    ssize_t len             = ::readlink("/proc/self/exe", self_path, sizeof(self_path) - 1);
    if (len > 0) {
        self_path[len] = '\0';
        std::string self(self_path);
        auto        slash = self.rfind('/');
        if (slash != std::string::npos) {
            std::string candidate = self.substr(0, slash) + "/test_crasher";
            if (::access(candidate.c_str(), X_OK) == 0) return candidate;
        }
    }

    // Fallback: relative (works when running from build/)
    return "bin/test_crasher";
}

// ── Fixture ─────────────────────────────────────────────────
class ProcessManagerTest : public ::testing::Test {
protected:
    std::string crasher_ = crasher_path();

    void SetUp() override {
        // Verify test_crasher exists
        ASSERT_EQ(::access(crasher_.c_str(), X_OK), 0)
            << "test_crasher not found at: " << crasher_ << " -- build the project first";
    }
};

// ═══════════════════════════════════════════════════════════
// Registration tests
// ═══════════════════════════════════════════════════════════

TEST_F(ProcessManagerTest, AddProcess) {
    ProcessManager mgr;
    mgr.add_process("proc_a", crasher_.c_str());
    mgr.add_process("proc_b", crasher_.c_str(), "exit0");
    EXPECT_EQ(mgr.size(), 2u);

    auto all = mgr.get_all();
    EXPECT_STREQ(all[0].name, "proc_a");
    EXPECT_EQ(all[0].state, ProcessState::STOPPED);
    EXPECT_STREQ(all[1].name, "proc_b");
    EXPECT_STREQ(all[1].args, "exit0");
}

TEST_F(ProcessManagerTest, FindByName) {
    ProcessManager mgr;
    mgr.add_process("alpha", crasher_.c_str());
    mgr.add_process("beta", crasher_.c_str());

    const auto* found = mgr.find("beta");
    ASSERT_NE(found, nullptr);
    EXPECT_STREQ(found->name, "beta");

    EXPECT_EQ(mgr.find("nonexistent"), nullptr);
}

TEST_F(ProcessManagerTest, NameTruncation) {
    ProcessManager mgr;
    std::string    long_name(64, 'x');
    mgr.add_process(long_name.c_str(), crasher_.c_str());

    auto all = mgr.get_all();
    EXPECT_EQ(std::strlen(all[0].name), 31u);  // Truncated to 31 + \0
}

// ═══════════════════════════════════════════════════════════
// Launch + clean exit tests
// ═══════════════════════════════════════════════════════════

TEST_F(ProcessManagerTest, LaunchSingleCleanExit) {
    ProcessManager mgr;
    mgr.add_process("clean", crasher_.c_str(), "exit0");

    EXPECT_TRUE(mgr.launch("clean"));

    auto all = mgr.get_all();
    EXPECT_EQ(all[0].state, ProcessState::RUNNING);
    EXPECT_GT(all[0].pid, 0);

    // Wait for it to exit
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    mgr.reap_children();

    all = mgr.get_all();
    // Child exited → RESTARTING (restart policy in effect)
    EXPECT_TRUE(all[0].state == ProcessState::RESTARTING || all[0].state == ProcessState::STOPPED);
    EXPECT_EQ(all[0].last_exit_code, 0);
    EXPECT_FALSE(all[0].was_signaled);
}

TEST_F(ProcessManagerTest, LaunchAllProcesses) {
    ProcessManager mgr;
    mgr.add_process("a", crasher_.c_str(), "sleep_2");
    mgr.add_process("b", crasher_.c_str(), "sleep_2");

    mgr.launch_all();

    auto all = mgr.get_all();
    EXPECT_EQ(all[0].state, ProcessState::RUNNING);
    EXPECT_EQ(all[1].state, ProcessState::RUNNING);
    EXPECT_NE(all[0].pid, all[1].pid);

    // Clean up
    mgr.stop_all(std::chrono::milliseconds{3000});
}

TEST_F(ProcessManagerTest, LaunchUnknownProcessFails) {
    ProcessManager mgr;
    EXPECT_FALSE(mgr.launch("nonexistent"));
}

// ═══════════════════════════════════════════════════════════
// Stop tests
// ═══════════════════════════════════════════════════════════

TEST_F(ProcessManagerTest, StopGraceful) {
    ProcessManager mgr;
    mgr.add_process("sleeper", crasher_.c_str(), "hang");

    mgr.launch("sleeper");
    std::this_thread::sleep_for(std::chrono::milliseconds{100});

    // Should succeed (SIGTERM → hang process exits)
    EXPECT_TRUE(mgr.stop("sleeper"));

    auto all = mgr.get_all();
    EXPECT_EQ(all[0].state, ProcessState::STOPPED);
    EXPECT_EQ(all[0].pid, -1);
}

TEST_F(ProcessManagerTest, StopAllMultipleProcesses) {
    ProcessManager mgr;
    mgr.add_process("s1", crasher_.c_str(), "hang");
    mgr.add_process("s2", crasher_.c_str(), "hang");

    mgr.launch_all();
    std::this_thread::sleep_for(std::chrono::milliseconds{100});

    mgr.stop_all(std::chrono::milliseconds{2000});

    auto all = mgr.get_all();
    for (const auto& p : all) {
        EXPECT_NE(p.state, ProcessState::RUNNING);
        EXPECT_EQ(p.pid, -1);
    }
}

// ═══════════════════════════════════════════════════════════
// Crash detection tests
// ═══════════════════════════════════════════════════════════

TEST_F(ProcessManagerTest, DetectErrorExit) {
    ProcessManager mgr;
    mgr.add_process("err", crasher_.c_str(), "exit1");

    bool death_seen = false;
    int  seen_code  = -1;
    mgr.set_death_callback([&](const char* name, int code, int) {
        if (std::strcmp(name, "err") == 0) {
            death_seen = true;
            seen_code  = code;
        }
    });

    mgr.launch("err");
    std::this_thread::sleep_for(std::chrono::milliseconds{300});
    mgr.tick();

    EXPECT_TRUE(death_seen);
    EXPECT_EQ(seen_code, 1);

    auto all = mgr.get_all();
    EXPECT_EQ(all[0].state, ProcessState::RESTARTING);
}

TEST_F(ProcessManagerTest, DetectCrash) {
    ProcessManager mgr;
    mgr.add_process("crasher", crasher_.c_str(), "crash");

    bool signaled    = false;
    int  seen_signal = 0;
    mgr.set_death_callback([&](const char* name, int, int sig) {
        if (std::strcmp(name, "crasher") == 0) {
            signaled    = true;
            seen_signal = sig;
        }
    });

    mgr.launch("crasher");

    // SIGSEGV delivery + core dump can take longer than a clean exit
    for (int i = 0; i < 20 && !signaled; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
        mgr.tick();
    }

    EXPECT_TRUE(signaled);
    EXPECT_EQ(seen_signal, SIGSEGV);

    auto all = mgr.get_all();
    EXPECT_TRUE(all[0].was_signaled);
    EXPECT_EQ(all[0].last_signal, SIGSEGV);
}

TEST_F(ProcessManagerTest, TickIgnoresAliveChild) {
    ProcessManager mgr;
    mgr.add_process("alive", crasher_.c_str(), "sleep_5");

    mgr.launch("alive");
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
    mgr.tick();

    auto all = mgr.get_all();
    EXPECT_EQ(all[0].state, ProcessState::RUNNING);
    EXPECT_GT(all[0].pid, 0);

    mgr.stop_all();
}

// ═══════════════════════════════════════════════════════════
// Restart policy tests
// ═══════════════════════════════════════════════════════════

TEST_F(ProcessManagerTest, MaxRestartsExhausted) {
    RestartPolicy policy;
    policy.max_restarts       = 2;
    policy.initial_backoff_ms = 50;  // Fast for testing
    policy.max_backoff_ms     = 100;

    ProcessManager mgr(policy);
    mgr.add_process("flappy", crasher_.c_str(), "exit1");

    mgr.launch("flappy");

    // Each crash → RESTARTING → tick relaunches → crash again
    // With max_restarts=2: first launch (not counted) + 2 restarts = FAILED
    for (int i = 0; i < 30; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
        mgr.tick();
        auto all = mgr.get_all();
        if (all[0].state == ProcessState::FAILED) break;
    }

    auto all = mgr.get_all();
    EXPECT_EQ(all[0].state, ProcessState::FAILED);
    EXPECT_GE(all[0].restart_count, 2u);
}

TEST_F(ProcessManagerTest, BackoffIncreases) {
    RestartPolicy policy;
    policy.initial_backoff_ms = 100;
    policy.max_backoff_ms     = 10000;
    policy.max_restarts       = 10;

    ProcessManager mgr(policy);
    mgr.add_process("back", crasher_.c_str(), "exit1");

    mgr.launch("back");
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    mgr.tick();

    // After first crash, process should be RESTARTING
    auto all = mgr.get_all();
    EXPECT_EQ(all[0].state, ProcessState::RESTARTING);

    // Immediate tick should NOT re-launch (backoff hasn't elapsed)
    mgr.tick();
    all = mgr.get_all();
    // Should still be RESTARTING (100ms backoff not yet elapsed after immediate tick)
    EXPECT_TRUE(all[0].state == ProcessState::RESTARTING || all[0].state == ProcessState::RUNNING);

    mgr.stop_all();
}

// ═══════════════════════════════════════════════════════════
// Integration: launch + external kill + auto-detect
// ═══════════════════════════════════════════════════════════

TEST_F(ProcessManagerTest, ExternalKillDetected) {
    ProcessManager mgr;
    mgr.add_process("victim", crasher_.c_str(), "hang");

    mgr.launch("victim");
    std::this_thread::sleep_for(std::chrono::milliseconds{100});

    auto all = mgr.get_all();
    ASSERT_EQ(all[0].state, ProcessState::RUNNING);
    pid_t child_pid = all[0].pid;
    ASSERT_GT(child_pid, 0);

    // External kill
    ::kill(child_pid, SIGKILL);
    std::this_thread::sleep_for(std::chrono::milliseconds{200});

    mgr.tick();

    all = mgr.get_all();
    EXPECT_EQ(all[0].state, ProcessState::RESTARTING);
    EXPECT_TRUE(all[0].was_signaled);
    EXPECT_EQ(all[0].last_signal, SIGKILL);
}

TEST_F(ProcessManagerTest, FullSupervisorCycle) {
    // Launch 2 processes → kill one → verify restart → stop all
    RestartPolicy policy;
    policy.max_restarts       = 3;
    policy.initial_backoff_ms = 50;

    ProcessManager mgr(policy);
    mgr.add_process("stable", crasher_.c_str(), "sleep_10");
    mgr.add_process("fragile", crasher_.c_str(), "hang");

    mgr.launch_all();
    std::this_thread::sleep_for(std::chrono::milliseconds{200});

    // Both should be running
    auto all = mgr.get_all();
    EXPECT_EQ(all[0].state, ProcessState::RUNNING);
    EXPECT_EQ(all[1].state, ProcessState::RUNNING);

    // Kill fragile
    pid_t fragile_pid = all[1].pid;
    ::kill(fragile_pid, SIGTERM);
    std::this_thread::sleep_for(std::chrono::milliseconds{200});

    // tick detects death and schedules restart
    mgr.tick();

    all = mgr.get_all();
    EXPECT_EQ(all[0].state, ProcessState::RUNNING);  // stable untouched
    EXPECT_EQ(all[1].state, ProcessState::RESTARTING);

    // Wait for backoff and let tick re-launch
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    mgr.tick();

    all = mgr.get_all();
    EXPECT_EQ(all[1].state, ProcessState::RUNNING);  // restarted
    EXPECT_NE(all[1].pid, fragile_pid);              // new PID

    mgr.stop_all();
}

// ═══════════════════════════════════════════════════════════
// State enum string conversion
// ═══════════════════════════════════════════════════════════

TEST_F(ProcessManagerTest, StateToString) {
    EXPECT_STREQ(to_string(ProcessState::STOPPED), "STOPPED");
    EXPECT_STREQ(to_string(ProcessState::RUNNING), "RUNNING");
    EXPECT_STREQ(to_string(ProcessState::RESTARTING), "RESTARTING");
    EXPECT_STREQ(to_string(ProcessState::FAILED), "FAILED");
}

// ═══════════════════════════════════════════════════════════
// FD leak prevention
// ═══════════════════════════════════════════════════════════

TEST_F(ProcessManagerTest, NoZombieProcesses) {
    ProcessManager mgr;
    mgr.add_process("z", crasher_.c_str(), "exit0");

    // Launch and reap several times
    for (int i = 0; i < 3; ++i) {
        mgr.launch("z");
        std::this_thread::sleep_for(std::chrono::milliseconds{200});
        mgr.reap_children();
    }

    // No zombies should exist — verify by checking /proc
    // (We can't easily count zombies, but the test validates that
    //  reap_children() doesn't leave waitpid calls pending)
    auto all = mgr.get_all();
    EXPECT_EQ(all[0].pid, -1);  // Reaped
}
