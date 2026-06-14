// common/util/include/util/stack_trace_capture.h
//
// Issue #765 — capture a stack trace of a STUCK thread from the watchdog.
//
// When ThreadWatchdog detects a stuck thread (e.g. the 31-second
// planning_loop stall that motivated #765), knowing WHERE the thread is
// blocked is the difference between a root-cause and a shrug.  This
// utility delivers SIGUSR1 to the stuck thread via tgkill(); the signal
// handler — which executes in the *stuck thread's* context — writes a
// raw backtrace into a static pre-allocated buffer; the watchdog thread
// then symbolises and logs it.
//
// Async-signal-safety contract (the load-bearing design constraint)
// ─────────────────────────────────────────────────────────────────
//   The SIGUSR1 handler may only touch:
//     - lock-free std::atomic (asserted via is_always_lock_free below) —
//       async-signal-safe per [support.signal]/3,
//     - ::backtrace() into a static buffer.  POSIX does not list
//       backtrace() as async-signal-safe; glibc's implementation is safe
//       PROVIDED the lazy libgcc_s dlopen (which mallocs) has already
//       happened.  install() forces that init with one throwaway
//       backtrace() call outside signal context — the standard,
//       documented mitigation (see glibc backtrace(3) NOTES).  Validate
//       on the target's glibc at hardware bring-up.
//   Everything that allocates or locks (backtrace_symbols, logging,
//   /proc reads) runs on the CALLING (watchdog) thread only.
//
// Why tgkill() and not pthread_kill()
// ───────────────────────────────────
//   ThreadHeartbeatRegistry slots are never unregistered, so a slot can
//   outlive its thread.  pthread_kill() on an exited thread's pthread_t
//   is undefined behaviour per POSIX; tgkill() on a dead tid fails with
//   ESRCH cleanly.  Worst case under tid reuse is a trace of the wrong
//   thread, which the log output makes identifiable (thread name +
//   tid are printed together).
//
// Why SA_RESTART (deliberate deviation from SignalHandler's sa_flags=0)
// ─────────────────────────────────────────────────────────────────────
//   SignalHandler (signal_handler.h, CPP_PATTERNS_GUIDE §2.8) installs
//   SIGTERM/SIGINT with sa_flags=0 because there interruption IS the
//   goal: blocked syscalls must wake for shutdown.  Here the opposite
//   holds: returning EINTR into whatever syscall the stuck thread is
//   blocked in would inject a rarely-tested error path into an
//   unaudited call site mid-flight — a state-corruption risk disguised
//   as a rescue.  With SA_RESTART the handler still runs (we get the
//   trace) and the interrupted syscall resumes transparently; the
//   deliberate, tested recovery path remains FaultManager's LOITER
//   escalation (PR #775).  If deliberate unsticking is ever wanted, it
//   must be its own reviewed decision, not a diagnostics side effect.
//
// Observability-on-flight-critical-threads compliance (CLAUDE.md)
// ───────────────────────────────────────────────────────────────
//   The handler executes on the (possibly flight-critical) stuck
//   thread.  This is acceptable because: (1) it is lock-free and
//   allocation-free — no priority-inversion channel; (2) it only ever
//   fires after the thread has already been stuck past the watchdog
//   threshold (default 5 s) — there is no useful control work left to
//   contaminate; (3) bounded work (~µs); (4) config-gated and
//   rate-limited.  Full analysis: DESIGN_RATIONALE.md (DR entry added
//   with the planner wiring PR).
//
// Concurrency state machine (watchdog thread ⇄ signal handler)
// ────────────────────────────────────────────────────────────
//   kIdle ─claim(watchdog CAS)→ kRequested ─claim(handler CAS)→ kBusyWriting
//     ▲                             │                                │
//     │                      timeout│(watchdog CAS, kRequested only)  │handler
//     │                             ▼                                 ▼ writes
//     │◄────reclaimable────── kTimedOut                             kDone
//     │  (next capture CASes        ▲                                 │
//     │   kTimedOut|kDone→kRequested)└──── (also reclaimable) ────────┘
//     └── watchdog consumes kDone, validates writer, stores kIdle
//
//   Three guarantees (hardened after #765 pre-commit review):
//   1. Single-writer: the handler must WIN an exclusive CAS
//      kRequested→kBusyWriting BEFORE it touches s_frames.  A parked/late
//      handler that lost the slot returns without writing — so s_frames has
//      exactly one writer and the watchdog (reads only after kDone, with
//      acquire) never races a concurrent write.  This closes the TOCTOU
//      that a bare "check s_target_tid then write later" left open: the
//      check and the write are no longer separable from the slot claim.
//   2. No-misattribution: the winning handler stamps s_writer_tid; the
//      watchdog re-validates s_writer_tid == requested tid before consuming
//      a kDone result.  A stale handler that wins a CAS on a *reclaimed*
//      slot stamps its OWN tid and is discarded, never logged as the new
//      target's stack.
//   3. No-wedge: a never-waking (D-state) target never enters the handler,
//      so the watchdog forces kRequested→kTimedOut and the next capture
//      reclaims kTimedOut (or a leftover kDone).  The watchdog forces a
//      timeout ONLY from kRequested — never from kBusyWriting — so it can
//      never reclaim a slot whose handler is mid-write.
//   The cheap gettid()==s_target_tid check at handler entry remains as a
//   fast reject of obviously-stale deliveries, but it is NOT the safety
//   mechanism — guarantees 1 and 2 are.
#pragma once

#include "util/iclock.h"
#include "util/ilogger.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>

#include <execinfo.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

namespace drone::util {

/// Outcome of a capture attempt.  Every failure mode is distinct so the
/// watchdog log tells the operator exactly why no trace was produced.
enum class TraceCaptureStatus : uint8_t {
    kOk,                ///< Trace captured, symbolised, and logged.
    kNotInstalled,      ///< install() never ran (feature disabled by config).
    kSignalSendFailed,  ///< tgkill failed (ESRCH = thread gone, stale slot).
    kTimeout,           ///< Handler did not complete within wait_timeout.
    kEmptyTrace,        ///< Handler ran but backtrace() returned 0 frames.
    kBusy,              ///< Another capture is in flight — skipped.
    kRateLimited,       ///< Same thread traced < min_interval ago — skipped.
};

/// Tunables — populated from drone::Config by the installing process
/// (see config_keys.h watchdog::stack_trace namespace).
struct TraceCaptureConfig {
    std::chrono::milliseconds wait_timeout{250};  ///< Handler-completion wait.
    std::chrono::seconds      min_interval{30};   ///< Per-tid re-capture floor.
};

/// Singleton — signal handlers require static state, and the registry /
/// watchdog precedent (ThreadHeartbeatRegistry::instance()) already
/// established the documented-singleton pattern for this layer.
class StackTraceCapture {
public:
    static StackTraceCapture& instance() {
        static StackTraceCapture cap;
        return cap;
    }

    /// Install the SIGUSR1 handler.  Call once at process startup BEFORE
    /// worker threads spawn (so no thread observes a half-installed
    /// disposition).  Idempotent: repeat calls are no-ops and return true —
    /// config is fixed at first install (NOT refreshed) to avoid a data
    /// race against a concurrent capture_and_log() (#765 review finding #3).
    ///
    /// Performs one throwaway backtrace() on the calling thread to force
    /// glibc's lazy libgcc_s init (which mallocs) outside signal context.
    [[nodiscard]] bool install(TraceCaptureConfig cfg = {}) {
        if (installed_.load(std::memory_order_acquire)) {
            // Already installed — config is FIXED at first install.  Earlier
            // revisions assigned config_ on every call ("refresh"), which is
            // a data race if a later install() runs concurrently with a
            // capture_and_log() on the watchdog thread reading config_.  The
            // one-shot contract ("install once before worker threads spawn")
            // makes the single first-install write safe: it is published by
            // the installed_ release-store below, which capture_and_log()'s
            // installed_ acquire-load synchronises-with.  (Pre-commit review
            // #765, finding #3.)
            return true;
        }
        config_ = cfg;  // written before installed_.store(release) → published h-b
        // Preload glibc's unwinder support — first backtrace() call may
        // dlopen/malloc; do it here, never in the signal handler.
        std::array<void*, 4> warmup{};
        (void)::backtrace(warmup.data(), static_cast<int>(warmup.size()));

        struct sigaction sa = {};
        sa.sa_handler       = &StackTraceCapture::signal_handler;
        sigemptyset(&sa.sa_mask);
        // SA_RESTART: do NOT perturb the stuck thread's blocked syscall —
        // see header comment.  Deviation from SignalHandler is deliberate.
        sa.sa_flags = SA_RESTART;
        if (::sigaction(SIGUSR1, &sa, nullptr) != 0) {
            DRONE_LOG_ERROR("[StackTrace] sigaction(SIGUSR1) failed: errno={}", errno);
            return false;
        }
        installed_.store(true, std::memory_order_release);
        return true;
    }

    /// Signal `tid`, wait (bounded) for the handler to fill the static
    /// frame buffer, symbolise on the CALLING thread, and log the result.
    ///
    /// SINGLE-CONSUMER CONTRACT: call from the watchdog scan thread ONLY.
    /// The state-machine handoff with the signal handler is fully atomic,
    /// but the rate-limiter state (`recent_`) and `config_` are plain
    /// non-atomic members read/written here without synchronisation — that
    /// is sound ONLY because exactly one thread ever calls this.  (The
    /// signal handler never touches `recent_`/`config_`.)  The `kBusy`
    /// return is a defensive guard against a contract violation, NOT a
    /// concurrency-safety guarantee: two concurrent callers would still
    /// race `recent_`/`config_` even though the slot CAS serialises the
    /// buffer.  Do not call this from multiple threads.
    [[nodiscard]] TraceCaptureStatus capture_and_log(pid_t tid, const char* thread_name) {
        if (!installed_.load(std::memory_order_acquire)) {
            return TraceCaptureStatus::kNotInstalled;
        }
        if (is_rate_limited(tid)) {
            DRONE_LOG_DEBUG("[StackTrace] thread '{}' (tid {}) traced recently — rate-limited",
                            thread_name, tid);
            return TraceCaptureStatus::kRateLimited;
        }

        // Claim the capture slot.  kIdle is the normal path.  kTimedOut and
        // kDone are LEFTOVER states from a prior capture that the watchdog
        // abandoned (timed out, or completed-too-late then never consumed):
        // both are reclaimable, otherwise a never-waking (D-state) target
        // would wedge the machine and block captures of OTHER threads
        // forever.  kRequested / kBusyWriting mean a capture is genuinely in
        // flight (or a handler is mid-write) → skip.  A stale handler that
        // wins a CAS on a reclaimed slot is caught by the s_writer_tid
        // validation after kDone, so reclamation cannot misattribute.
        uint32_t expected = kIdle;
        if (!s_state.compare_exchange_strong(expected, kRequested, std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
            if ((expected == kTimedOut || expected == kDone) &&
                s_state.compare_exchange_strong(expected, kRequested, std::memory_order_acq_rel,
                                                std::memory_order_acquire)) {
                DRONE_LOG_WARN("[StackTrace] reclaiming slot abandoned by a prior capture "
                               "(leftover state, previous target tid {})",
                               s_target_tid.load(std::memory_order_acquire));
            } else {
                DRONE_LOG_WARN("[StackTrace] capture already in flight (state={}) — skipping "
                               "'{}'",
                               expected, thread_name);
                return TraceCaptureStatus::kBusy;
            }
        }
        // Clear the writer stamp, then publish the target — both BEFORE
        // sending the signal.  Clearing first means a slow handler from a
        // prior capture cannot leave a stamp that masquerades as this one.
        s_writer_tid.store(0, std::memory_order_release);
        s_target_tid.store(tid, std::memory_order_release);

        if (::syscall(SYS_tgkill, ::getpid(), tid, SIGUSR1) != 0) {
            const int err = errno;
            s_state.store(kIdle, std::memory_order_release);  // release the slot
            // tid gone (ESRCH) won't recur, so it is NOT rate-limit-recorded.
            DRONE_LOG_ERROR(
                "[StackTrace] tgkill(tid={}) for '{}' failed: errno={}{}", tid, thread_name, err,
                err == ESRCH ? " (thread no longer exists — stale heartbeat slot)" : "");
            return TraceCaptureStatus::kSignalSendFailed;
        }

        // Bounded wait for the handler.  Wait while it has neither finished
        // (kDone) nor failed to start — i.e. while kRequested (not yet
        // entered) or kBusyWriting (entered, writing the buffer).  1 ms poll
        // is cheap and fine-grained vs the 250 ms default budget.
        const auto deadline = std::chrono::steady_clock::now() + config_.wait_timeout;
        for (uint32_t st                                = s_state.load(std::memory_order_acquire);
             st == kRequested || st == kBusyWriting; st = s_state.load(std::memory_order_acquire)) {
            if (std::chrono::steady_clock::now() >= deadline) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (s_state.load(std::memory_order_acquire) != kDone) {
            // Force a timeout, but ONLY from kRequested (handler never
            // entered — the classic D-state stall).  We deliberately do NOT
            // reclaim a kBusyWriting slot: a handler is actively writing the
            // buffer and must be left to finish (backtrace() is bounded CPU
            // work, no syscalls); it will reach kDone and the slot becomes a
            // reclaimable leftover for the next call.
            expected = kRequested;
            if (s_state.compare_exchange_strong(expected, kTimedOut, std::memory_order_acq_rel,
                                                std::memory_order_acquire)) {
                // FIX #2: record the attempt so a never-responding (D-state)
                // thread is NOT re-signalled every ~1 s scan forever — the
                // rate-limit floor applies to timeouts too, not just kOk.
                note_capture(tid);
                log_timeout_diagnostics(tid, thread_name);
                return TraceCaptureStatus::kTimeout;
            }
            if (s_state.load(std::memory_order_acquire) != kDone) {
                // kBusyWriting that did not finish within the budget (writer
                // preempted mid-backtrace — pathologically rare).  Abandon
                // this round without reclaiming; the writer will complete to
                // kDone and the next call reclaims it.
                note_capture(tid);
                log_timeout_diagnostics(tid, thread_name);
                return TraceCaptureStatus::kTimeout;
            }
            // else: handler completed in the race window — fall through.
        }

        // kDone — validate the writer before trusting the buffer.  A stale
        // handler that won a CAS on a reclaimed slot stamped its OWN tid,
        // not ours; discard rather than misattribute its stack to `tid`.
        if (s_writer_tid.load(std::memory_order_acquire) != tid) {
            s_state.store(kIdle, std::memory_order_release);
            note_capture(tid);
            DRONE_LOG_WARN("[StackTrace] thread '{}' (tid {}): discarding trace written by a "
                           "different (stale) handler — no valid capture this round",
                           thread_name, tid);
            return TraceCaptureStatus::kEmptyTrace;
        }

        // Copy frames out, release the slot, then symbolise.
        const int                     frame_count = s_frame_count.load(std::memory_order_acquire);
        std::array<void*, kMaxFrames> frames{};
        std::copy(std::begin(s_frames),
                  std::begin(s_frames) + std::clamp(frame_count, 0, static_cast<int>(kMaxFrames)),
                  frames.begin());
        s_state.store(kIdle, std::memory_order_release);
        note_capture(tid);  // recorded on every post-signal outcome (fix #2)

        if (frame_count <= 0) {
            DRONE_LOG_ERROR("[StackTrace] thread '{}' (tid {}): handler ran but captured 0 "
                            "frames",
                            thread_name, tid);
            return TraceCaptureStatus::kEmptyTrace;
        }

        symbolize_and_log(frames.data(), frame_count, tid, thread_name);
        return TraceCaptureStatus::kOk;
    }

    /// Reset all static state — only for testing.  NOT thread-safe; call
    /// with no captures in flight.  Does not uninstall the signal handler
    /// (process-wide disposition; tests tolerate it staying installed).
    void reset_for_testing() {
        s_state.store(kIdle, std::memory_order_release);
        s_frame_count.store(0, std::memory_order_release);
        s_target_tid.store(0, std::memory_order_release);
        s_writer_tid.store(0, std::memory_order_release);
        for (auto& entry : recent_) {
            entry = RecentCapture{};
        }
    }

    /// Test seams (read-only) — let tests positively confirm WHICH protocol
    /// branch ran, so a regression test cannot pass vacuously via a
    /// different code path (#765 review finding #4).  Not for production use.
    [[nodiscard]] bool is_idle_for_testing() const {
        return s_state.load(std::memory_order_acquire) == kIdle;
    }
    [[nodiscard]] bool is_timed_out_for_testing() const {
        return s_state.load(std::memory_order_acquire) == kTimedOut;
    }
    [[nodiscard]] pid_t target_tid_for_testing() const {
        return s_target_tid.load(std::memory_order_acquire);
    }

private:
    StackTraceCapture()                                    = default;
    StackTraceCapture(const StackTraceCapture&)            = delete;
    StackTraceCapture& operator=(const StackTraceCapture&) = delete;

    // ── Capture-protocol state (static: shared with the signal handler) ──
    //
    //   kIdle ─claim(CAS)→ kRequested ─handler claims(CAS)→ kBusyWriting
    //                          │                                  │
    //                   timeout│(CAS, only from kRequested)       │handler
    //                          ▼                                  ▼ writes
    //                     kTimedOut                             kDone
    //
    //   kBusyWriting (added by #765 review finding #1) closes the TOCTOU
    //   the bare s_target_tid guard left open: the handler must WIN an
    //   exclusive CAS kRequested→kBusyWriting BEFORE it touches s_frames,
    //   so only one handler ever writes the buffer (no handler-vs-handler
    //   data race) and a parked/stale handler that lost the slot returns
    //   without writing.  s_writer_tid (also new) is stamped by the winning
    //   handler and re-validated by the watchdog before it consumes a
    //   kDone result — defence-in-depth against a stale handler that wins
    //   a CAS on a *reclaimed* slot and would otherwise misattribute its
    //   own stack to the new target.  The watchdog only ever forces a
    //   timeout from kRequested (never from kBusyWriting), so it can never
    //   reclaim a slot whose handler is mid-write.
    static constexpr uint32_t kIdle        = 0;
    static constexpr uint32_t kRequested   = 1;
    static constexpr uint32_t kBusyWriting = 2;
    static constexpr uint32_t kDone        = 3;
    static constexpr uint32_t kTimedOut    = 4;

    /// Frame-buffer geometry — compile-time constant (buffer must be
    /// statically allocated for signal-safety), not a runtime tunable.
    static constexpr size_t kMaxFrames = 64;

    static inline void*                 s_frames[kMaxFrames] = {};
    static inline std::atomic<int32_t>  s_frame_count{0};
    static inline std::atomic<uint32_t> s_state{kIdle};
    static inline std::atomic<pid_t>    s_target_tid{0};
    // tid of the handler that actually wrote s_frames for the current kDone
    // result.  Watchdog validates this == requested tid before consuming —
    // a stale handler that won a CAS on a reclaimed slot stamps its own
    // (wrong) tid and is discarded rather than misattributed (#765 #1).
    static inline std::atomic<pid_t> s_writer_tid{0};

    // Lock-free atomics are async-signal-safe per C++17 [support.signal]/3;
    // anything else in the handler would be UB.
    static_assert(std::atomic<uint32_t>::is_always_lock_free,
                  "signal handler requires lock-free u32 atomics");
    static_assert(std::atomic<int32_t>::is_always_lock_free,
                  "signal handler requires lock-free i32 atomics");
    static_assert(std::atomic<pid_t>::is_always_lock_free,
                  "signal handler requires lock-free pid_t atomics");

    /// SIGUSR1 handler — executes in the TARGET (stuck) thread's context.
    /// Async-signal-safe body only: errno save/restore (TSan flags
    /// handlers that spoil errno), raw gettid syscall, backtrace() into
    /// the static buffer, lock-free atomics.  No logging, no malloc, no
    /// locks.
    static void signal_handler(int /*sig*/) {
        const int saved_errno = errno;
        // gettid is a raw syscall → async-signal-safe.
        const pid_t self = static_cast<pid_t>(::syscall(SYS_gettid));

        // Fast reject of an obviously-stale delivery (signal queued for an
        // earlier request, target since changed).  Cheap; NOT the safety
        // mechanism — that is the exclusive CAS below + s_writer_tid stamp.
        if (self != s_target_tid.load(std::memory_order_acquire)) {
            errno = saved_errno;
            return;
        }

        // Claim EXCLUSIVE write ownership BEFORE touching s_frames.  This
        // is the fix for the TOCTOU the bare tid-guard left open (#765 #1):
        // a parked handler that lost the slot fails this CAS and returns
        // without writing, so s_frames has a single writer and the watchdog
        // (which only reads after kDone) never races a concurrent write.
        uint32_t expected = kRequested;
        if (!s_state.compare_exchange_strong(expected, kBusyWriting, std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
            // Lost the slot (watchdog timed out → kTimedOut, or another
            // handler is already writing → kBusyWriting, or already kDone).
            // Do NOT touch the buffer.
            errno = saved_errno;
            return;
        }

        // Exclusive owner from here.  Write frames + count + writer tid,
        // then publish via the release-store of kDone (the watchdog's
        // acquire-load of kDone synchronises-with it, making all three
        // writes visible).
        const int n = ::backtrace(s_frames, static_cast<int>(kMaxFrames));
        s_frame_count.store(n, std::memory_order_relaxed);
        s_writer_tid.store(self, std::memory_order_relaxed);
        s_state.store(kDone, std::memory_order_release);
        errno = saved_errno;
    }

    /// Symbolise + log — CALLING (watchdog) thread only; backtrace_symbols
    /// mallocs, which is fine off the signal path and off hot loops.
    static void symbolize_and_log(void* const* frames, int count, pid_t tid,
                                  const char* thread_name) {
        DRONE_LOG_ERROR("[StackTrace] ── stuck thread '{}' (tid {}) — {} frames ──", thread_name,
                        tid, count);
        // unique_ptr+free: backtrace_symbols returns one malloc'd block;
        // libc-malloc/libc-free pairing is unavoidable here and contained.
        const std::unique_ptr<char*, decltype(&::free)> symbols(::backtrace_symbols(frames, count),
                                                                &::free);
        for (int i = 0; i < count; ++i) {
            // Raw address printed alongside the symbol so offline
            // `addr2line -e <binary>` works for frames the dynamic symtab
            // cannot name (static/inlined functions).  ptr→uintptr_t is
            // the standard-blessed integer view of a pointer for printing.
            const auto addr = reinterpret_cast<uintptr_t>(frames[i]);
            if (symbols != nullptr) {
                // Symbol names need -rdynamic for non-static functions.
                DRONE_LOG_ERROR("[StackTrace]   #{:02d} {} [{:#x}]", i, symbols.get()[i], addr);
            } else {
                DRONE_LOG_ERROR("[StackTrace]   #{:02d} [{:#x}]", i, addr);
            }
        }
    }

    /// Timeout path: the likeliest cause is the thread being blocked in
    /// uninterruptible sleep (D state) — plausibly the actual #765
    /// culprit.  Read its /proc state so the log says so explicitly.
    void log_timeout_diagnostics(pid_t tid, const char* thread_name) const {
        const char state = read_proc_thread_state(tid);
        DRONE_LOG_ERROR(
            "[StackTrace] thread '{}' (tid {}): handler did not run within {} ms "
            "(proc state '{}'){}",
            thread_name, tid,
            std::chrono::duration_cast<std::chrono::milliseconds>(config_.wait_timeout).count(),
            state,
            state == 'D' ? " — uninterruptible sleep: userspace trace impossible; "
                           "inspect /proc/<pid>/task/<tid>/stack as root"
                         : "");
    }

    /// Read field 3 (state: R/S/D/Z/T) of /proc/self/task/<tid>/stat.
    /// Returns '?' on any failure.  Watchdog thread only — fopen is fine.
    static char read_proc_thread_state(pid_t tid) {
        char path[64] = {};
        std::snprintf(path, sizeof(path), "/proc/self/task/%d/stat", static_cast<int>(tid));
        std::FILE* f = std::fopen(path, "re");
        if (f == nullptr) {
            return '?';
        }
        char state = '?';
        // Format: "<tid> (<comm>) <state> ..." — comm may contain spaces
        // but not ')', so scan past the last ')'.
        char buf[256] = {};
        if (std::fgets(buf, sizeof(buf), f) != nullptr) {
            const char* close = std::strrchr(buf, ')');
            if (close != nullptr && close[1] == ' ' && close[2] != '\0') {
                state = close[2];
            }
        }
        std::fclose(f);
        return state;
    }

    // ── Per-tid rate limiting ──
    // The watchdog stuck-callback fires on EVERY ~1 s scan while a thread
    // stays stuck (only the watchdog's log line is transition-gated), so
    // the capturer enforces its own re-capture floor.  Sized to the
    // heartbeat registry capacity; uses get_clock() so ScopedMockClock
    // drives it deterministically in tests.
    struct RecentCapture {
        pid_t    tid             = 0;
        uint64_t last_capture_ns = 0;
    };

    [[nodiscard]] bool is_rate_limited(pid_t tid) const {
        const uint64_t interval_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(config_.min_interval).count());
        const uint64_t now_ns = get_clock().now_ns();
        for (const auto& entry : recent_) {
            if (entry.tid == tid && entry.last_capture_ns != 0 && now_ns >= entry.last_capture_ns &&
                (now_ns - entry.last_capture_ns) < interval_ns) {
                return true;
            }
        }
        return false;
    }

    void note_capture(pid_t tid) {
        const uint64_t now_ns = get_clock().now_ns();
        // Update existing entry for this tid, else take the oldest slot.
        RecentCapture* target = &recent_[0];
        for (auto& entry : recent_) {
            if (entry.tid == tid) {
                target = &entry;
                break;
            }
            if (entry.last_capture_ns < target->last_capture_ns) {
                target = &entry;
            }
        }
        target->tid             = tid;
        target->last_capture_ns = now_ns;
    }

    // 16 entries matches kMaxThreads in thread_heartbeat.h (not included
    // by name to keep this header free-standing for non-heartbeat users).
    std::array<RecentCapture, 16> recent_{};
    TraceCaptureConfig            config_{};
    std::atomic<bool>             installed_{false};
};

}  // namespace drone::util
