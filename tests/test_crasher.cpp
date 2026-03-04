// tests/test_crasher.cpp
// Minimal test binary for ProcessManager tests.
//
// Behaviour controlled by first CLI argument:
//   "exit0"     → exit(0)                  (clean exit)
//   "exit1"     → exit(1)                  (error exit)
//   "crash"     → raise(SIGSEGV)           (crash)
//   "hang"      → sleep forever            (stuck process)
//   "sleep_N"   → sleep N seconds, exit(0) (timed exit)
//   (none)      → exit(0)                  (default: clean exit)
//
// This avoids testing ProcessManager with the real stack processes.

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <thread>

int main(int argc, char* argv[]) {
    if (argc < 2) return 0;

    const char* mode = argv[1];

    if (std::strcmp(mode, "exit0") == 0) {
        return 0;
    }
    if (std::strcmp(mode, "exit1") == 0) {
        return 1;
    }
    if (std::strcmp(mode, "crash") == 0) {
        std::raise(SIGSEGV);
        return 99;  // Should not reach here
    }
    if (std::strcmp(mode, "hang") == 0) {
        while (true) {
            std::this_thread::sleep_for(std::chrono::hours(1));
        }
    }
    // sleep_N — sleep for N seconds, then exit cleanly
    if (std::strncmp(mode, "sleep_", 6) == 0) {
        int seconds = std::atoi(mode + 6);
        if (seconds > 0) {
            std::this_thread::sleep_for(std::chrono::seconds(seconds));
        }
        return 0;
    }

    // Unknown mode
    return 2;
}
