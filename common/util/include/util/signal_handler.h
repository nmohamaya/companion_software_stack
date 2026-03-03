// common/util/include/util/signal_handler.h
// Async-signal-safe shutdown via atomic flag + sigaction.
#pragma once
#include <atomic>
#include <csignal>
#include <cstdio>

class SignalHandler {
public:
    /// Install signal handlers for SIGTERM, SIGINT, SIGPIPE.
    static void install(std::atomic<bool>& running_flag) {
        s_running_ = &running_flag;

        struct sigaction sa {};
        sa.sa_handler = handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;  // no SA_RESTART — we want blocked calls to return

        sigaction(SIGTERM, &sa, nullptr);
        sigaction(SIGINT, &sa, nullptr);

        // Suppress SIGPIPE (broken serial/UDP pipe)
        struct sigaction sa_ignore {};
        sa_ignore.sa_handler = SIG_IGN;
        sigaction(SIGPIPE, &sa_ignore, nullptr);
    }

private:
    static inline std::atomic<bool>* s_running_ = nullptr;

    static void handler(int sig) {
        // Only async-signal-safe operations here
        if (s_running_) {
            s_running_->store(false, std::memory_order_relaxed);
        }
        const char*           msg = "Signal received, shutting down...\n";
        [[maybe_unused]] auto r   = write(STDOUT_FILENO, msg, 35);
        (void)sig;
    }
};
