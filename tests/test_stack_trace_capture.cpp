// tests/test_stack_trace_capture.cpp
//
// Issue #765 — StackTraceCapture unit tests.
//
// Covers the SIGUSR1 capture protocol: install idempotency, capture on a
// live (spinning) thread, capture on a thread blocked in a condition-
// variable wait, the timeout path (target thread masks SIGUSR1 — a
// deterministic stand-in for the unreproducible D-state case), the
// late-handler-after-timeout race regression, dead-tid ESRCH handling,
// and the get_clock()-driven rate limiter.
//
// Sanitizer note: TSan intercepts signal delivery and may delay handler
// execution substantially.  Wait budgets are multiplied under sanitizer
// builds (test_helpers.h::is_sanitizer_build()); correctness assertions
// are NOT skipped (per the test_helpers.h contract).

#include "test_helpers.h"
#include "util/mock_clock.h"
#include "util/stack_trace_capture.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <mutex>
#include <thread>

#include <gtest/gtest.h>
#include <sys/syscall.h>
#include <unistd.h>

using drone::util::ScopedMockClock;
using drone::util::StackTraceCapture;
using drone::util::TraceCaptureConfig;
using drone::util::TraceCaptureStatus;

namespace {

/// Sanitizer-aware wait budget: signal delivery under TSan can be slow.
constexpr int kWaitMultiplier = drone::test::is_sanitizer_build() ? 8 : 1;

TraceCaptureConfig test_config() {
    TraceCaptureConfig cfg;
    cfg.wait_timeout = std::chrono::milliseconds(250 * kWaitMultiplier);
    cfg.min_interval = std::chrono::seconds(30);
    return cfg;
}

/// Helper thread that publishes its kernel tid then spins until told to
/// stop.  Gives tests a live signal-able target with a known tid.
class SpinningThread {
public:
    SpinningThread() {
        thread_ = std::thread([this] {
            tid_.store(static_cast<pid_t>(::syscall(SYS_gettid)), std::memory_order_release);
            while (!stop_.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
        });
        // Wait for the tid to be published.
        while (tid_.load(std::memory_order_acquire) == 0) {
            std::this_thread::yield();
        }
    }

    ~SpinningThread() {
        stop_.store(true, std::memory_order_release);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    [[nodiscard]] pid_t tid() const { return tid_.load(std::memory_order_acquire); }

private:
    std::thread        thread_;
    std::atomic<pid_t> tid_{0};
    std::atomic<bool>  stop_{false};
};

class StackTraceCaptureTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto& cap = StackTraceCapture::instance();
        cap.reset_for_testing();
        ASSERT_TRUE(cap.install(test_config()));
    }

    void TearDown() override { StackTraceCapture::instance().reset_for_testing(); }
};

// NOTE (#765 review #6): the kNotInstalled status path is intentionally
// NOT unit-tested.  install() registers a PROCESS-WIDE SIGUSR1 disposition
// and sets installed_=true; reset_for_testing() deliberately does NOT
// uninstall (you cannot cleanly revert a process's signal disposition
// mid-run without races against other tests in the same binary).  Once any
// test in this fixture installs, installed_ stays true, so a kNotInstalled
// assertion is unreachable here.  The branch is trivial (one acquire-load +
// early return) and covered by inspection; isolating it would require a
// separate single-test binary, which is not worth the build complexity.

// ─── Install ────────────────────────────────────────────────────────

TEST_F(StackTraceCaptureTest, InstallIsIdempotent) {
    auto& cap = StackTraceCapture::instance();
    EXPECT_TRUE(cap.install(test_config()));
    EXPECT_TRUE(cap.install(test_config()));  // second call: config refresh only
}

// ─── Capture on live threads ────────────────────────────────────────

TEST_F(StackTraceCaptureTest, CaptureOnLiveSpinningThread) {
    SpinningThread target;
    const auto status = StackTraceCapture::instance().capture_and_log(target.tid(), "spin_thread");
    EXPECT_EQ(status, TraceCaptureStatus::kOk);
}

TEST_F(StackTraceCaptureTest, CaptureOnThreadBlockedInCvWait) {
    std::mutex              mtx;
    std::condition_variable cv;
    bool                    release = false;
    std::atomic<pid_t>      tid{0};

    std::thread blocked([&] {
        tid.store(static_cast<pid_t>(::syscall(SYS_gettid)), std::memory_order_release);
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [&] { return release; });
    });
    while (tid.load(std::memory_order_acquire) == 0) {
        std::this_thread::yield();
    }
    // Give the thread a moment to actually enter the futex wait.
    std::this_thread::sleep_for(std::chrono::milliseconds(20 * kWaitMultiplier));

    const auto status = StackTraceCapture::instance().capture_and_log(
        tid.load(std::memory_order_acquire), "cv_blocked_thread");
    EXPECT_EQ(status, TraceCaptureStatus::kOk);

    {
        std::lock_guard<std::mutex> lock(mtx);
        release = true;
    }
    cv.notify_one();
    blocked.join();
}

// ─── Timeout path (deterministic D-state stand-in via sigmask) ──────

/// Helper: a thread that blocks SIGUSR1 (deterministic stand-in for a
/// D-state thread the signal can never reach), publishes its tid, spins
/// until told to stop, then DRAINS any pending SIGUSR1 via sigtimedwait
/// (handler never runs) before unmasking and exiting.
class MaskedThread {
public:
    MaskedThread() {
        thread_ = std::thread([this] {
            sigset_t set;
            sigemptyset(&set);
            sigaddset(&set, SIGUSR1);
            mask_ok_.store(pthread_sigmask(SIG_BLOCK, &set, nullptr) == 0,
                           std::memory_order_release);
            tid_.store(static_cast<pid_t>(::syscall(SYS_gettid)), std::memory_order_release);
            while (!stop_.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            // Drain the pending SIGUSR1 so the handler cannot fire late on
            // this thread (it stays pending while masked).
            const timespec ts{0, 0};
            (void)sigtimedwait(&set, nullptr, &ts);
            pthread_sigmask(SIG_UNBLOCK, &set, nullptr);
        });
        while (tid_.load(std::memory_order_acquire) == 0) {
            std::this_thread::yield();
        }
    }

    ~MaskedThread() {
        stop_.store(true, std::memory_order_release);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    [[nodiscard]] pid_t tid() const { return tid_.load(std::memory_order_acquire); }
    [[nodiscard]] bool  mask_ok() const { return mask_ok_.load(std::memory_order_acquire); }

private:
    std::thread        thread_;
    std::atomic<pid_t> tid_{0};
    std::atomic<bool>  stop_{false};
    std::atomic<bool>  mask_ok_{false};
};

TEST_F(StackTraceCaptureTest, TimeoutWhenTargetMasksSigusr1) {
    MaskedThread masked;
    ASSERT_TRUE(masked.mask_ok());

    // Short timeout so the test stays fast.
    TraceCaptureConfig cfg = test_config();
    cfg.wait_timeout       = std::chrono::milliseconds(50 * kWaitMultiplier);
    ASSERT_TRUE(StackTraceCapture::instance().install(cfg));

    const auto status = StackTraceCapture::instance().capture_and_log(masked.tid(),
                                                                      "masked_thread");
    EXPECT_EQ(status, TraceCaptureStatus::kTimeout);
}

TEST_F(StackTraceCaptureTest, TimedOutSlotIsReclaimedByNextCapture) {
    // Regression for the wedge bug: a target whose handler NEVER arrives
    // (D-state analogue) leaves the state machine at kTimedOut.  Without
    // reclamation, every later capture — including of healthy threads —
    // would return kBusy forever, silencing the diagnostic exactly when
    // it's needed.
    TraceCaptureConfig cfg = test_config();
    cfg.wait_timeout       = std::chrono::milliseconds(50 * kWaitMultiplier);
    cfg.min_interval       = std::chrono::seconds(0);
    ASSERT_TRUE(StackTraceCapture::instance().install(cfg));

    auto& cap = StackTraceCapture::instance();
    {
        MaskedThread wedged;
        ASSERT_TRUE(wedged.mask_ok());
        ASSERT_EQ(cap.capture_and_log(wedged.tid(), "wedged_thread"), TraceCaptureStatus::kTimeout);
        // wedged's destructor drains the pending signal — handler never ran.
    }

    // State is kTimedOut with no handler coming.  A capture on a healthy
    // thread must reclaim the slot and succeed.
    SpinningThread healthy;
    EXPECT_EQ(cap.capture_and_log(healthy.tid(), "healthy_after_wedge"), TraceCaptureStatus::kOk);
}

TEST_F(StackTraceCaptureTest, LateHandlerAfterTimeoutDoesNotCorruptNextCapture) {
    // Non-vacuous regression for the late-handler path (#765 review #4): the
    // earlier version asserted only the Phase-3 kOk, which passes whether
    // the late handler took the intended "lost the slot" branch OR the
    // signal simply never arrived.  This version positively confirms the
    // handler RAN (handler_ran flag set inside the unmask call that delivers
    // the pending signal) AND that it left the state machine at kTimedOut
    // (it did NOT win the kRequested→kBusyWriting CAS, did NOT write kDone,
    // did NOT corrupt the slot) — exactly the single-writer guarantee.
    std::atomic<pid_t> tid{0};
    std::atomic<bool>  unblock{false};
    std::atomic<bool>  handler_ran{false};
    std::atomic<bool>  stop{false};
    std::atomic<bool>  mask_ok{false};

    std::thread target([&] {
        sigset_t set;
        sigemptyset(&set);
        sigaddset(&set, SIGUSR1);
        mask_ok.store(pthread_sigmask(SIG_BLOCK, &set, nullptr) == 0, std::memory_order_release);
        tid.store(static_cast<pid_t>(::syscall(SYS_gettid)), std::memory_order_release);
        while (!unblock.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        // Unmask — the pending SIGUSR1 is delivered (and the handler runs to
        // completion) DURING this call, i.e. the "late handler" fires after
        // the watchdog already timed out.  Setting the flag AFTER the call
        // returns means main can deterministically wait for "handler done".
        pthread_sigmask(SIG_UNBLOCK, &set, nullptr);
        handler_ran.store(true, std::memory_order_release);
        while (!stop.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
    });
    while (tid.load(std::memory_order_acquire) == 0) {
        std::this_thread::yield();
    }
    ASSERT_TRUE(mask_ok.load(std::memory_order_acquire));

    TraceCaptureConfig cfg = test_config();
    cfg.wait_timeout       = std::chrono::milliseconds(50 * kWaitMultiplier);
    cfg.min_interval       = std::chrono::seconds(0);  // no rate limit for this test
    ASSERT_TRUE(StackTraceCapture::instance().install(cfg));

    auto& cap = StackTraceCapture::instance();
    ASSERT_EQ(cap.capture_and_log(tid.load(std::memory_order_acquire), "late_handler_thread"),
              TraceCaptureStatus::kTimeout);
    // Watchdog forced the timeout; the late signal is still pending+masked.
    ASSERT_TRUE(cap.is_timed_out_for_testing());

    // Phase 2: release the pending signal — handler fires, fails its
    // kRequested→kBusyWriting CAS (state is kTimedOut), returns WITHOUT
    // touching the slot.  Wait deterministically for it to have run.
    unblock.store(true, std::memory_order_release);
    while (!handler_ran.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
    // Positive assertion: the handler ran yet the slot is STILL kTimedOut —
    // proves it took the lost-the-slot branch, not a buffer write.  Without
    // the kBusyWriting claim this would be kDone (and a torn buffer).
    EXPECT_TRUE(cap.is_timed_out_for_testing());

    // Phase 3: a fresh capture on a healthy thread reclaims the leftover
    // slot and succeeds — the diagnostic is not wedged.
    SpinningThread healthy;
    EXPECT_EQ(cap.capture_and_log(healthy.tid(), "post_race_thread"), TraceCaptureStatus::kOk);

    stop.store(true, std::memory_order_release);
    target.join();
}

// ─── Rate limiting applies to the timeout path (#765 review #2) ──────

TEST_F(StackTraceCaptureTest, RateLimitAppliesToTimeoutPathNotJustSuccess) {
    // Regression for the bug where note_capture() ran ONLY on kOk, so a
    // never-responding (D-state) thread — which ALWAYS times out — was never
    // rate-limited and got re-signalled every watchdog scan (~1 Hz) forever.
    // The fix records the attempt on the timeout path too, so a second
    // capture of the same wedged tid within min_interval is suppressed.
    TraceCaptureConfig cfg = test_config();
    cfg.wait_timeout       = std::chrono::milliseconds(50 * kWaitMultiplier);
    cfg.min_interval       = std::chrono::seconds(30);  // real floor
    ASSERT_TRUE(StackTraceCapture::instance().install(cfg));

    auto&        cap = StackTraceCapture::instance();
    MaskedThread wedged;  // D-state stand-in: SIGUSR1 masked → always times out
    ASSERT_TRUE(wedged.mask_ok());

    EXPECT_EQ(cap.capture_and_log(wedged.tid(), "wedged"), TraceCaptureStatus::kTimeout);
    // Second attempt within the 30 s floor must be suppressed — NOT another
    // tgkill + 50 ms busy-poll + ERROR log.
    EXPECT_EQ(cap.capture_and_log(wedged.tid(), "wedged"), TraceCaptureStatus::kRateLimited);
}

// ─── Concurrent capture returns kBusy (#765 review #5) ──────────────

TEST_F(StackTraceCaptureTest, ConcurrentCaptureWhileOneInFlightReturnsBusy) {
    // A capture against a masked target sits in kRequested for the full
    // wait_timeout.  A concurrent capture (second "consumer") must observe
    // the slot is taken and return kBusy rather than corrupting the
    // in-flight capture.
    TraceCaptureConfig cfg = test_config();
    cfg.wait_timeout       = std::chrono::milliseconds(200 * kWaitMultiplier);
    cfg.min_interval       = std::chrono::seconds(0);
    ASSERT_TRUE(StackTraceCapture::instance().install(cfg));

    auto&        cap = StackTraceCapture::instance();
    MaskedThread wedged;
    ASSERT_TRUE(wedged.mask_ok());

    std::atomic<bool> first_started{false};
    std::thread       first([&] {
        first_started.store(true, std::memory_order_release);
        (void)cap.capture_and_log(wedged.tid(), "in_flight");  // blocks ~200 ms in kRequested
    });
    while (!first_started.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
    // Give the first capture a moment to claim the slot (kIdle→kRequested).
    std::this_thread::sleep_for(std::chrono::milliseconds(20 * kWaitMultiplier));

    SpinningThread other;
    EXPECT_EQ(cap.capture_and_log(other.tid(), "concurrent"), TraceCaptureStatus::kBusy);

    first.join();
}

// ─── Dead-tid handling ──────────────────────────────────────────────

TEST_F(StackTraceCaptureTest, DeadTidReturnsSignalSendFailed) {
    // Deliberately synthetic: a REAL exited thread's tid can be recycled
    // by a new in-process thread between join and tgkill (observed as a
    // 1-in-N flake in full-suite runs), making that variant inherently
    // racy.  tgkill(getpid(), tid, sig) only matches tasks in OUR thread
    // group, so tid 1 (init's main thread, never ours in a test run)
    // exercises the identical ESRCH code path deterministically.  This
    // models the production case: a stale heartbeat slot whose thread is
    // gone (slots are never unregistered).
    const auto status = StackTraceCapture::instance().capture_and_log(1, "dead_thread");
    EXPECT_EQ(status, TraceCaptureStatus::kSignalSendFailed);
}

// ─── Rate limiting (mock-clock driven) ──────────────────────────────

TEST_F(StackTraceCaptureTest, RateLimitSuppressesRepeatCaptureUntilIntervalElapses) {
    // ScopedMockClock declared BEFORE any capture so the rate-limiter's
    // get_clock() reads hit the mock (CPP_PATTERNS_GUIDE §5.7 ordering).
    ScopedMockClock clock;

    SpinningThread target;
    auto&          cap = StackTraceCapture::instance();

    ASSERT_EQ(cap.capture_and_log(target.tid(), "rate_limited_thread"), TraceCaptureStatus::kOk);

    // Immediately again — inside the 30 s floor.
    EXPECT_EQ(cap.capture_and_log(target.tid(), "rate_limited_thread"),
              TraceCaptureStatus::kRateLimited);

    // Advance past the floor — capture allowed again.
    clock.mock().advance_ms(31'000);
    EXPECT_EQ(cap.capture_and_log(target.tid(), "rate_limited_thread"), TraceCaptureStatus::kOk);
}

TEST_F(StackTraceCaptureTest, RateLimitIsPerThread) {
    ScopedMockClock clock;

    SpinningThread first;
    SpinningThread second;
    auto&          cap = StackTraceCapture::instance();

    ASSERT_EQ(cap.capture_and_log(first.tid(), "thread_a"), TraceCaptureStatus::kOk);
    // A different tid is not affected by thread_a's floor.
    EXPECT_EQ(cap.capture_and_log(second.tid(), "thread_b"), TraceCaptureStatus::kOk);
    // But thread_a itself is still limited.
    EXPECT_EQ(cap.capture_and_log(first.tid(), "thread_a"), TraceCaptureStatus::kRateLimited);
}

}  // namespace
