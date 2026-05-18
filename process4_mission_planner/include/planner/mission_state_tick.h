// process4_mission_planner/include/planner/mission_state_tick.h
// Per-tick state machine logic for the mission planner.
// Handles all FSM states: PREFLIGHT, TAKEOFF, SURVEY, NAVIGATE,
// NAVIGATE_UNSTUCK (Issue #503), COLLISION_RECOVERY (Issue #226), RTL, LAND.
// IDLE / EMERGENCY are explicit no-op cases.
//
// Extracted from main.cpp as part of Issue #154.
// Updated in Issue #158: AStarPathPlanner* → IGridPlanner*.
#pragma once

#include "ipc/ipc_types.h"
#include "ipc/ipublisher.h"
#include "planner/gcs_command_handler.h"
#include "planner/grid_planner_base.h"
#include "planner/iobstacle_avoider.h"
#include "planner/ipath_planner.h"
#include "planner/mission_fsm.h"
#include "planner/static_obstacle_layer.h"
#include "util/diagnostic.h"
#include "util/iclock.h"
#include "util/ilogger.h"
#include "util/math_constants.h"
#include "util/scoped_timer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

namespace drone::planner {

/// Configuration for MissionStateTick tunables.
struct StateTickConfig {
    float takeoff_alt_m{10.0f};
    float rtl_acceptance_m{1.5f};
    float landed_alt_m{0.5f};
    int   rtl_min_dwell_s{5};
    float survey_duration_s{0.0f};  // Post-takeoff obstacle survey duration (0 = skip)
    float survey_yaw_rate{0.3f};    // Yaw rate during survey (rad/s, ~0.3 = full 360 in ~21s)

    // Collision recovery (Issue #226)
    bool  collision_recovery_enabled{true};
    float collision_climb_delta_m{3.0f};     // altitude gain during recovery climb
    float collision_hover_duration_s{2.0f};  // hover-in-place duration before climb

    // Stuck detector (Issue #503) — flags NAVIGATE hangs where the avoider
    // is producing correction but the vehicle isn't moving.
    StuckDetector::Config stuck_detector{};
    // Cached avoider influence radius, so the NAVIGATE DIAG line and any
    // "objects within influence radius" calculation use the same value the
    // avoider actually uses.  Populated by main.cpp at startup from
    // mission_planner.obstacle_avoidance.influence_radius_m.
    float avoider_influence_radius_m = 5.0f;

    // Issue #716 — PREFLIGHT ARM gating timings.  Defaults match the values
    // empirically validated on scenario 18:
    //   - 3 s ARM retry: short enough to recover within a few ticks if PX4
    //     drops an ARM message; long enough not to flood MAVLink.
    //   - 1 s wait log: visible to operators without spamming the log.
    //
    // **Currently NOT runtime-configurable.**  Despite the `mission_planner.
    // preflight.*` shape that an operator might infer from the field names,
    // these two are read straight from the struct defaults — `main.cpp` does
    // not call `cfg.get<>()` for them, no entries exist in `config/default.
    // json`, and no constants exist in `config_keys.h`.  Wiring them through
    // `drone::Config` (so they can be tuned per-scenario) is tracked in
    // `docs/tracking/IMPROVEMENTS.md` ("Sibling `cfg_key` constants for
    // `preflight_arm_retry_s` / `preflight_wait_log_s` absent" — filed from
    // the PR #741 review).  Surfaced again by the PR #763 (Layer 4) review:
    // the new `takeoff_settle_*` keys land alongside as genuinely
    // config-backed, making the contrast stark.
    int preflight_arm_retry_s{3};
    int preflight_wait_log_s{1};

    // Issue #740 (epic #727 Layer 1) — debounce window on `fc_state.armable`
    // before sending ARM.  Required because PX4's `health_all_ok` flickers
    // true momentarily on Gazebo cold-start while EKF2 attitude estimate is
    // still settling (gyro/accel bias estimates wandering).  Arming on a
    // single-tick flicker causes asymmetric mixer commands → asymmetric
    // rotor spin-up → drone tips on ground.  Requiring N consecutive seconds
    // of continuous `armable=true` ensures EKF2 has actually converged before
    // we hand control to PX4's arming flow.  3 s default chosen empirically:
    // long enough to wait out the worst-case settling window observed across
    // scenarios 02, 17, 18, 25, 26 (#727 reproduction matrix); short enough
    // to keep mission startup latency acceptable on well-conditioned boots.
    float preflight_armable_stable_s{3.0f};

    // Issue #740 (epic #727 Layer 4) — post-ARM, pre-TAKEOFF settle gate.
    // The #741 `armable` debounce (above) proved necessary-but-insufficient:
    // the #746 smoke sweep showed PX4 can report armed with a marginal EKF2
    // attitude estimate, and commanding TAKEOFF onto that state makes PX4's
    // attitude controller fight a phantom tilt error → asymmetric rotor
    // spin-up → the drone skids sideways instead of climbing.  Layer 4 holds
    // after `fc_state.armed` until the FC's *own* attitude + velocity
    // estimate proves stable: `|roll|` and `|pitch|` within
    // `takeoff_max_tilt_deg`, `sqrt(vx²+vy²+vz²)` within
    // `takeoff_max_velocity_mps`, for `takeoff_settle_observations`
    // *consecutive* FCState observations.  Any excursion resets the counter
    // — the estimate must settle continuously, not cumulatively.
    //
    // **Observation-counted, not wall-timed** — deliberately.  The companion
    // runs on wall-clock; Gazebo+PX4-SITL run on sim-time.  A wall-timed gate
    // under real-time-factor < 1 would under-wait in sim-time.  Counting
    // FCState observations is RTF-immune: PX4 paces FCState publication in
    // sim-time, so N observations is N observations regardless of RTF, and
    // the gate behaves identically in SITL and on real hardware.
    //
    // **Default:** 30 observations (~0.6 s at a 50 Hz FCState rate).
    // **Disable:** `takeoff_settle_observations = 0` collapses to legacy
    // immediate-takeoff-on-armed — used by headless dev configs and the
    // unit-test fixture so tests exercising post-TAKEOFF behaviour don't
    // need to feed a settled FCState attitude stream.
    int   takeoff_settle_observations{30};
    float takeoff_max_tilt_deg{5.0f};
    float takeoff_max_velocity_mps{0.3f};

    // Issue #718 — PREFLIGHT timeout escalation.  If either Layer 1
    // (`fc_state.armable` never stably true) or Layer 4 (post-ARM
    // attitude/velocity never settle) holds the FSM in PREFLIGHT for
    // more than `preflight_timeout_s` wall-clock seconds, the planner:
    //   1. emits FCCommandType::DISARM (cheap action — drone is on the
    //      ground or motors at idle, NEVER in flight from PREFLIGHT),
    //   2. transitions FSM → IDLE via `fsm.on_landed()` (the existing
    //      IDLE entry path),
    //   3. raises a single-shot event consumed by main.cpp on the next
    //      `FaultManager.evaluate()` to set `FAULT_FC_PREFLIGHT_TIMEOUT`
    //      for GCS visibility.  PR #775 review fix: the FaultManager
    //      setter is OR-latched (see fault_manager.h::set_preflight_timeout)
    //      so the bit persists in `active_faults` across all subsequent
    //      evaluate() calls until `reset()` — GCS polling at 500 ms–1 s
    //      reliably observes the fault despite the planner ticking at
    //      10 Hz.  Pre-fix it was visible for one tick only.
    //
    // `preflight_warn_s` (must be < `preflight_timeout_s`) promotes
    // the per-tick "Waiting for FC preflight" log from INFO to WARN so
    // operators see the escalation approaching before it fires.
    //
    // Both keys default to "0 disables the escalation entirely" so
    // legacy unit-test fixtures (and headless dev configs that don't
    // want timeouts firing during interactive debugging) can keep the
    // existing hold-forever behaviour.  Production configs MUST set
    // a non-zero timeout — see CLAUDE.md §"Asymmetric pre-conditions
    // for asymmetric-cost actions": a stuck-in-PREFLIGHT drone with
    // no operator alert is a silent-failure mode.
    int preflight_warn_s{30};
    int preflight_timeout_s{60};
};

/// Per-tick state machine logic for the mission planner.
/// Owns tracking variables and implements the FSM tick for each state.
class MissionStateTick {
public:
    explicit MissionStateTick(const StateTickConfig& config)
        : config_(config), stuck_detector_(config.stuck_detector) {}

    /// Execute one tick of the state machine.
    void tick(MissionFSM& fsm, const drone::ipc::Pose& pose, const drone::ipc::FCState& fc_state,
              const drone::ipc::DetectedObjectList& objects, IPathPlanner& planner,
              IGridPlanner* grid_planner, IObstacleAvoider& avoider,
              StaticObstacleLayer&                                obstacle_layer,
              drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>&  traj_pub,
              drone::ipc::IPublisher<drone::ipc::PayloadCommand>& payload_pub,
              const FCSendFn& send_fc, uint64_t correlation_id,
              drone::util::FrameDiagnostics& diag) {
        // Record home position from the first real pose, regardless of state.
        try_record_home(pose);

        // Pause promotion during RTL/LAND — the drone is descending to a
        // known-safe location and ground-feature detections would pollute the
        // static layer, blocking the landing approach (Issue #340).
        //
        // Issue #638 P1-B from PR #641 review: also clear per-instance
        // observation counters on the *transition* into RTL/LAND.  After
        // P2 restarts the perception context, fresh instance_id values
        // would otherwise collide with stale `instances_[N]` counters
        // that already crossed the promotion threshold — defeating the
        // gate's confirmation guarantee.  Clearing on the entering edge
        // (was_landing_=false → landing=true) is one-shot and matches
        // the same FSM-transition trigger as `set_promotion_paused`.
        if (grid_planner != nullptr) {
            const bool landing = (fsm.state() == MissionState::RTL ||
                                  fsm.state() == MissionState::LAND);
            // Issue #645 review fix (#661 P1, SAFETY-CRITICAL): use the
            // dedicated landing-pause channel instead of `promotion_paused`.
            // The previous call to `set_promotion_paused(landing)` would
            // overwrite the voxel_input scenario's startup setting (which
            // calls `set_promotion_paused(true)` once at boot for "PATH A
            // is sole source" semantics) — leaving the grid permanently
            // un-paused after the first RTL completes.  More importantly,
            // it didn't unconditionally block promotion: with #661's radar
            // bypass enabled, radar returns could promote during landing,
            // voiding the Issue #340 landing-approach protection.
            // `set_landing_pause` blocks ALL promotion (including radar)
            // and is independent of the voxel_input pause.
            grid_planner->set_landing_pause(landing);
            if (landing && !was_landing_) {
                grid_planner->clear_instance_state();
            }
            was_landing_ = landing;
        }

        // Clear stuck-detector mission-scoped state when back in IDLE
        // (Issue #503 review): the counter and FAULT_STUCK flag must not
        // persist across missions.  Covers every path to IDLE — disarm,
        // landing, mission abort — without needing per-transition hooks.
        if (fsm.state() == MissionState::IDLE) {
            if (stuck_transition_count_ != 0 || flight_state_.stuck_fault_active) {
                stuck_transition_count_          = 0;
                flight_state_.stuck_fault_active = false;
            }
        }

        // Issue #718 — reset PREFLIGHT timeout tracking on the trailing
        // edge of any PREFLIGHT exit (success via takeoff, or GCS abort,
        // or our own timeout-induced abort).  Without this, a future
        // re-entry to PREFLIGHT would inherit a stale `preflight_held_since_ns_`
        // and fire the timeout almost immediately.
        if (last_tick_state_ == MissionState::PREFLIGHT && fsm.state() != MissionState::PREFLIGHT) {
            reset_preflight_timer();
        }
        last_tick_state_ = fsm.state();

        // No `default:` — exhaustive switch enables -Wswitch to catch new
        // MissionState values the moment they're added (CLAUDE.md safety rule).
        switch (fsm.state()) {
            case MissionState::PREFLIGHT: tick_preflight(fsm, fc_state, send_fc); break;
            case MissionState::TAKEOFF: tick_takeoff(fsm, pose, fc_state, send_fc); break;
            case MissionState::SURVEY:
                tick_survey(fsm, pose, objects, grid_planner, traj_pub);
                break;
            case MissionState::NAVIGATE:
                tick_navigate(fsm, pose, fc_state, objects, planner, grid_planner, avoider,
                              obstacle_layer, traj_pub, payload_pub, send_fc, correlation_id, diag);
                break;
            case MissionState::NAVIGATE_UNSTUCK:
                tick_navigate_unstuck(fsm, pose, fc_state, traj_pub);
                break;
            case MissionState::COLLISION_RECOVERY:
                tick_collision_recovery(fsm, pose, fc_state, grid_planner, traj_pub);
                break;
            case MissionState::RTL: tick_rtl(fsm, pose, fc_state, send_fc); break;
            case MissionState::LAND: tick_land(fsm, fc_state); break;
            case MissionState::IDLE:
            case MissionState::LOITER:
            case MissionState::EMERGENCY: break;  // no per-tick work
        }
    }

    /// Access shared flight state (for GCS handler + fault executor).
    [[nodiscard]] SharedFlightState&       flight_state() { return flight_state_; }
    [[nodiscard]] const SharedFlightState& flight_state() const { return flight_state_; }

    /// Whether a fault-state reset happened this tick (caller reads + clears).
    [[nodiscard]] bool consume_fault_reset() {
        bool v            = fault_exec_reset_;
        fault_exec_reset_ = false;
        return v;
    }

    /// Issue #718 — single-shot event flag.  Set by `tick_preflight()`
    /// when the PREFLIGHT-stuck timer fires (after DISARMing + FSM → IDLE).
    /// main.cpp reads + clears this each tick and passes the boolean into
    /// `FaultManager::set_preflight_timeout()` so the next `evaluate()`
    /// raises FAULT_FC_PREFLIGHT_TIMEOUT for GCS visibility.
    [[nodiscard]] bool consume_preflight_timeout_event() {
        bool v                   = preflight_timeout_fired_;
        preflight_timeout_fired_ = false;
        return v;
    }

private:
    StateTickConfig   config_;
    SharedFlightState flight_state_;

    // Tracking variables
    bool     takeoff_sent_     = false;
    float    home_x_           = 0.0f;
    float    home_y_           = 0.0f;
    float    home_z_           = 0.0f;
    bool     home_recorded_    = false;
    bool     home_warn_logged_ = false;
    bool     fault_exec_reset_ = false;
    uint64_t diag_tick_        = 0;  // Throttle counter for periodic DIAG log in tick_navigate.

    // Survey state
    bool                                  survey_started_ = false;
    std::chrono::steady_clock::time_point survey_start_time_{};
    float                                 survey_start_yaw_ = 0.0f;
    uint64_t                              survey_log_tick_  = 0;

    // Issue #716 + #740 — PREFLIGHT timing state.  All three throttles below
    // share a single clock domain (`drone::util::get_clock().now_ns()`) so
    // unit tests with `ScopedMockClock` can drive every PREFLIGHT timer
    // deterministically.  Sentinel `0` means "never fired / not currently
    // tracking" — interpreted as "infinitely far in the past" by the elapsed
    // checks, so the first observation always passes the retry / wait-log
    // thresholds.  Issue #740 PR-A review (PR #743) consolidated these from
    // a mix of `std::chrono::steady_clock` and `drone::util::get_clock()` to
    // close the test-mockability gap flagged by 4 of 9 review agents.
    uint64_t last_arm_time_ns_      = 0;  // last time ARM command was emitted
    uint64_t last_wait_log_time_ns_ = 0;  // last time the "waiting for FC" log fired

    // Issue #740 — first observation of `fc_state.armable == true` since the
    // last `armable == false` transition.  Used to debounce the ARM trigger
    // against momentary `health_all_ok` flickers on Gazebo cold-start.  Zero
    // means "not currently observing armable=true" (either we never have, or
    // it dropped back to false and reset the timer).
    uint64_t armable_first_seen_ns_ = 0;

    // Issue #740 (epic #727 Layer 4) — count of consecutive armed-state
    // FCState observations where attitude + velocity have been within the
    // takeoff-settle thresholds.  Incremented per settled tick.  Reset to 0
    // on:
    //   (a) any excursion (attitude/velocity outside thresholds, or
    //       non-finite from the FC),
    //   (b) `fc_state.armed` going false (disarm / re-PREFLIGHT), and
    //   (c) the gate-disabled path firing takeoff
    //       (`takeoff_settle_observations == 0`) — for documentation
    //       completeness; the counter is a no-op in that branch since it
    //       never had a chance to grow.
    // Takeoff fires once it reaches `config_.takeoff_settle_observations`.
    uint32_t armed_settle_count_ = 0;

    // Issue #777 — Layer 4 diagnostic counter.  Total Layer 4 evaluations
    // within the current armed cycle (incremented on every observation
    // reaching the settle check, reset whenever `fc_state.armed` is false
    // or PREFLIGHT exits).  Used to throttle the periodic-snapshot log so
    // operators can see the actual roll/pitch/|v| values the gate is
    // measuring, instead of inferring "never settled" from the absence of
    // a release log.  Without this, run-4 of PR #776 sweep produced 17s of
    // armed=true with zero release AND zero excursion logs (the existing
    // `count > 0` guard suppresses the excursion log when the counter
    // never accumulates a single settled observation).
    uint32_t layer4_eval_count_ = 0;

    // Issue #718 — PREFLIGHT timeout escalation tracking.
    // `preflight_held_since_ns_` = wall time of the first tick the FSM
    // entered (or returned to) PREFLIGHT.  Zero = "not currently in a
    // PREFLIGHT hold."  Reset on (a) successful release via
    // `fsm.on_takeoff()`, (b) the timeout firing (we transition to IDLE),
    // and (c) the outer-tick state-exit detector below (if the FSM
    // leaves PREFLIGHT for any other reason — GCS abort, etc.).
    // `preflight_last_warn_log_ns_` throttles the WARN-promoted log so
    // it fires once per `preflight_wait_log_s` after the warn threshold.
    // `preflight_timeout_fired_` is a single-shot event flag for
    // `consume_preflight_timeout_event()` — main.cpp reads + clears it
    // each tick and passes the boolean into `FaultManager.set_preflight_timeout()`.
    uint64_t preflight_held_since_ns_    = 0;
    uint64_t preflight_last_warn_log_ns_ = 0;
    bool     preflight_timeout_fired_    = false;

    // Issue #718 — tracks the previous tick's FSM state so we can detect
    // PREFLIGHT exit on the *trailing* edge and clear `preflight_held_since_ns_`.
    // Without this, a GCS-induced abort (PREFLIGHT → IDLE) would leave
    // the timer set, and a future re-entry to PREFLIGHT would inherit a
    // stale elapsed window and fire the timeout immediately.
    MissionState last_tick_state_ = MissionState::IDLE;

    // Collision recovery state (Issue #226)
    enum class RecoveryPhase : uint8_t { HOVER = 0, CLIMB = 1, REPLAN = 2 };
    bool                                  recovery_started_ = false;
    RecoveryPhase                         recovery_phase_   = RecoveryPhase::HOVER;
    std::chrono::steady_clock::time_point recovery_start_time_{};
    float                                 recovery_target_alt_ = 0.0f;

    // Stuck detector / NAVIGATE_UNSTUCK (Issue #503)
    StuckDetector                         stuck_detector_;
    std::chrono::steady_clock::time_point unstuck_start_time_{};
    float                                 unstuck_vx_ = 0.0f;  // backoff velocity at entry
    float                                 unstuck_vy_ = 0.0f;
    float                                 unstuck_vz_ = 0.0f;
    // STUCK transition counter — when it exceeds stuck.max_stuck_count we
    // escalate to LOITER with fault_triggered so the operator sees repeated
    // stall in health telemetry instead of silent oscillation.  Reset on
    // waypoint advance or mission restart (IDLE re-entry).
    uint32_t stuck_transition_count_ = 0;

    // Issue #638 P1-B (PR #641 review): tracks the previous-tick value
    // of `landing` so we can call `clear_instance_state()` exactly once
    // on the entering edge into RTL/LAND, not every tick.
    bool was_landing_ = false;

    // Extract yaw (rad) from pose quaternion [w, x, y, z].  Used in three
    // places that previously had identical inline atan2 blocks.  Keeps the
    // sign convention in one location so future frame changes only touch here.
    static float yaw_from_pose(const drone::ipc::Pose& pose) {
        const double qw = pose.quaternion[0], qx = pose.quaternion[1];
        const double qy = pose.quaternion[2], qz = pose.quaternion[3];
        return static_cast<float>(
            std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));
    }

    // Issue #740 / PR #743 review (code-quality P2): the PREFLIGHT block
    // had three near-identical inline throttle checks of the form
    //   if (last_ns == 0 || now_ns < last_ns || (now_ns - last_ns) >= period_ns)
    // — sentinel-0 "never fired", backward-clock guard (for mock-clock
    // resets / host clock skew), and elapsed-period.  Extracted to one
    // helper so the four-condition contract (including the `last_ns =
    // now_ns` update) is documented in one place.
    //
    // Note: this is NOT the same as the `armable_first_seen_ns_` window-
    // start pattern in `tick_preflight` — that one starts a window with
    // just sentinel-0 + backward-clock guards (no elapsed-period check
    // because the window is the point of the gate, not a throttle).
    //
    // Returns true and updates `last_ns` to `now_ns` if the throttle should
    // fire this tick.  Returns false (without modifying `last_ns`) otherwise.
    static bool fire_throttled(uint64_t& last_ns, uint64_t now_ns, uint64_t period_ns) {
        if (last_ns == 0 || now_ns < last_ns || (now_ns - last_ns) >= period_ns) {
            last_ns = now_ns;
            return true;
        }
        return false;
    }

    // Issue #718 / PR #775 review (code-quality P2): the two PREFLIGHT
    // timeout-tracking fields move together — reset one, reset the other,
    // or the warn-log throttle gets out of sync with the held-since
    // timestamp.  Extracted to a one-liner so all four reset sites
    // (outer-tick state-exit detector, timeout-fired path, gate-disabled
    // takeoff success, Layer 4 settle success) can't accidentally diverge
    // if a third field is added later.
    void reset_preflight_timer() {
        preflight_held_since_ns_    = 0;
        preflight_last_warn_log_ns_ = 0;
    }

    // ── PREFLIGHT: wait for FC readiness, then ARM (retry on configurable
    //               interval as a safety net for dropped MAVLink messages) ──
    //
    // Issue #716 — gate ARM on `fc_state.armable` to avoid the cold-start
    // `Arming denied: Resolve system health failures first` race on Gazebo
    // SITL boots.  PX4's `Telemetry::health_all_ok` reports true once EKF2
    // has converged, sensors are initialized, and GPS lock is acquired;
    // sending ARM before that just generates log spam and (in degraded
    // boot timing) can produce sloppy / crashing takeoffs once health
    // later flickers through OK.  The `preflight_arm_retry_s` retry stays
    // as a safety net for the rare case PX4 drops the ARM message after
    // armable went high.
    //
    // Issue #740 (epic #727) — momentary `armable=true` is not enough.
    // PX4's `health_all_ok` can flicker through OK while EKF2 attitude is
    // still settling (gyro/accel bias estimates wandering for the first
    // 1-15 s after spawn).  Arming on a single-tick flicker produces
    // asymmetric mixer commands and the drone tips on the ground at
    // takeoff.  We require `preflight_armable_stable_s` of *continuous*
    // `armable=true` before sending ARM.  Any drop back to false resets
    // `armable_first_seen_ns_` and restarts the stability window.
    //
    // **Default:** `preflight_armable_stable_s = 3.0 s` (production tuning).
    // **Disable:** setting `preflight_armable_stable_s = 0.0` collapses the
    // gate to legacy single-tick behaviour (ARM fires on first `armable=true`
    // tick) — used by headless dev configs and the existing unit-test fixture
    // (`make_default_test_config()` in `tests/test_mission_state_tick.cpp`)
    // so tests that exercise post-PREFLIGHT behaviour don't need to drive
    // a mock clock.  Production configs MUST keep the gate enabled (≥1 s) —
    // see CLAUDE.md §"FSM transitions emitting physical FC commands must be
    // debounced".
    //
    // **No timeout / no fault escalation** (review-fault-recovery P2):
    // if `armable` never becomes stably true (e.g. PX4 EKF stuck, MAVSDK
    // subscription silently lost, hardware sensor genuinely faulty), the
    // FSM remains in PREFLIGHT indefinitely with only the 1 Hz "Waiting
    // for FC preflight" INFO log.  FaultManager evaluates `fc_connected`
    // and pose-stale timestamps but does not yet have an armable-stuck
    // fault.  Adding a configurable max-PREFLIGHT-wait + escalation to
    // FAULT_FC_PREFLIGHT_TIMEOUT is tracked as #718.
    void tick_preflight(MissionFSM& fsm, const drone::ipc::FCState& fc_state,
                        const FCSendFn& send_fc) {
        // PR #741 review (4 agents convergent): use a single clock domain
        // (`drone::util::get_clock()`) for ALL PREFLIGHT timing so unit tests
        // with `ScopedMockClock` can drive every throttle deterministically.
        // One `now_ns` capture per tick — cheap (one atomic load + virtual
        // call), and `tick_preflight()` exits PREFLIGHT after a few seconds
        // so this isn't a sustained hot path.
        const uint64_t now_ns = drone::util::get_clock().now_ns();

        // ── Issue #718 — PREFLIGHT timeout escalation ──────────
        // Track continuous time held in PREFLIGHT and DISARM + abort
        // to IDLE if `preflight_timeout_s` elapses without release.
        // Covers BOTH Layer 1 (`armable` never stably true) and Layer 4
        // (post-ARM attitude/velocity never settle) — neither layer
        // released, no point in keeping the FSM held with no operator
        // visibility past the configured budget.  Per CLAUDE.md
        // "Asymmetric pre-conditions for asymmetric-cost actions",
        // the recovery action is DISARM (cheap: drone is on the
        // ground or motors at idle — NEVER in flight from PREFLIGHT)
        // rather than LOITER (which is the mid-flight response and
        // makes no sense pre-takeoff).
        //
        // Sentinel `0` = "not currently tracking" — set on first tick
        // in PREFLIGHT, reset by the outer-tick state-exit detector
        // (in `tick()`) and by the takeoff-success paths below.
        // Clock-rewind guard mirrors the `armable_first_seen_ns_`
        // pattern (PR #743 review): a MockClock manual reset must
        // restart the window, not produce a wrapped-uint underflow.
        if (preflight_held_since_ns_ == 0 || now_ns < preflight_held_since_ns_) {
            preflight_held_since_ns_    = now_ns;
            preflight_last_warn_log_ns_ = 0;  // fresh window, fresh WARN throttle
        }
        const uint64_t held_ns = now_ns - preflight_held_since_ns_;
        const uint64_t timeout_ns =
            static_cast<uint64_t>(std::max(0, config_.preflight_timeout_s)) * 1'000'000'000ULL;
        const uint64_t warn_ns = static_cast<uint64_t>(std::max(0, config_.preflight_warn_s)) *
                                 1'000'000'000ULL;

        if (timeout_ns > 0 && held_ns > timeout_ns) {
            // Timeout fired — abort PREFLIGHT.  Emit DISARM (cheap: pre-
            // takeoff), transition FSM → IDLE via the existing IDLE entry
            // path, and raise the single-shot event for main.cpp to
            // propagate as FAULT_FC_PREFLIGHT_TIMEOUT on the next
            // `FaultManager.evaluate()`.
            DRONE_LOG_ERROR("[Planner] PREFLIGHT timeout — held for {:.1f}s without release "
                            "(armable={}, armed={}); DISARMing and aborting to IDLE. "
                            "FAULT_FC_PREFLIGHT_TIMEOUT will be raised for GCS.",
                            static_cast<double>(held_ns) * 1e-9, fc_state.armable, fc_state.armed);
            send_fc(drone::ipc::FCCommandType::DISARM, 0.0f);
            fsm.on_landed();  // FSM → IDLE (existing entry path)
            preflight_timeout_fired_ = true;
            reset_preflight_timer();  // avoid immediate re-fire on re-entry
            armable_first_seen_ns_ = 0;
            armed_settle_count_    = 0;
            layer4_eval_count_     = 0;  // #777 — reset eval counter on timeout abort
            return;
        }

        // Warn threshold reached but not yet timeout — promote the
        // per-tick wait log from INFO to WARN so operators see the
        // escalation approaching.  Throttled on `preflight_wait_log_s`
        // (separate from the Layer 1 `last_wait_log_time_ns_` INFO
        // throttle so an operator monitoring at WARN sees both the
        // promotion edge and the per-tick state without flooding).
        if (warn_ns > 0 && held_ns > warn_ns) {
            const uint64_t warn_log_interval_ns =
                static_cast<uint64_t>(std::max(0, config_.preflight_wait_log_s)) * 1'000'000'000ULL;
            if (fire_throttled(preflight_last_warn_log_ns_, now_ns, warn_log_interval_ns)) {
                DRONE_LOG_WARN(
                    "[Planner] PREFLIGHT held for {:.1f}s (warn at {}s, timeout at {}s) — "
                    "armable={}, armed={}",
                    static_cast<double>(held_ns) * 1e-9, config_.preflight_warn_s,
                    config_.preflight_timeout_s, fc_state.armable, fc_state.armed);
            }
        }

        if (fc_state.armed) {
            // Issue #740 (epic #727) Layer 4 — post-ARM, pre-TAKEOFF settle
            // gate.  The #746 smoke sweep proved the #741 `armable` debounce
            // is necessary-but-insufficient: PX4 can report armed with a
            // marginal EKF2 attitude estimate, and commanding TAKEOFF onto
            // that state makes PX4's attitude controller fight a phantom
            // tilt → asymmetric rotor spin-up → the drone skids sideways
            // instead of climbing.  Hold until the FC's own attitude +
            // velocity estimate proves stable for N *consecutive* FCState
            // observations.  Observation-counted (not wall-timed) → RTF-immune
            // in SITL (see StateTickConfig::takeoff_settle_observations).
            //
            // PR #763 review (Copilot): clear `armable_first_seen_ns_` here.
            // The pre-ARM debounce window has served its purpose by the time
            // we observe `armed=true`, so leaving it set leaves a stale
            // timestamp that — if the drone disarms while still in the Layer
            // 4 settle gate (operator intervention, mid-PREFLIGHT fault) —
            // would fall back to the armable code below with `(now -
            // first_seen) >= window` immediately satisfied, letting the next
            // ARM fire on a single armable=true tick without re-debouncing.
            // Resetting on the armed-observed edge closes that re-arm path.
            armable_first_seen_ns_ = 0;

            const int settle_target = config_.takeoff_settle_observations;
            if (settle_target <= 0) {
                // Gate disabled by config — legacy immediate takeoff.
                DRONE_LOG_INFO("[Planner] Vehicle armed — initiating takeoff");
                fsm.on_takeoff();
                takeoff_sent_       = false;
                armed_settle_count_ = 0;
                layer4_eval_count_  = 0;  // #777 — reset eval counter on disabled-gate takeoff
                // PR #775 review (code-quality P2.6): redundant
                // `armable_first_seen_ns_ = 0` removed — already cleared
                // at the top of the `if (fc_state.armed)` block (see
                // "Copilot fix" comment ~30 lines up).
                //
                // Issue #718 — release on successful takeoff fires.
                // Defence-in-depth: the outer-tick state-exit detector
                // in `tick()` would catch the PREFLIGHT → TAKEOFF edge
                // on the next tick, but resetting here makes the intent
                // explicit and avoids relying on the dispatch order.
                reset_preflight_timer();
                return;
            }

            // Issue #777 — first-evaluation diagnostic.  Emit one INFO at
            // the start of every Layer 4 armed cycle so operators (and log
            // archaeologists) can see the gate was actually reached with
            // current threshold + target values.  Helps disambiguate the
            // "no Layer 4 logs at all" case (= gate skipped or never
            // entered) from the "evaluated but never settled" case (=
            // genuine threshold / FC issue).
            //
            // PR #779 Copilot review fix: emitted *after* the
            // `settle_target <= 0` early-return so we don't advertise
            // thresholds + target that won't be applied (the gate-disabled
            // legacy path takes off immediately above and never reaches
            // here).
            if (layer4_eval_count_ == 0) {
                DRONE_LOG_INFO(
                    "[Layer4] evaluation started — thresholds tilt<={:.1f}° vel<={:.2f}m/s, "
                    "target {} consecutive observations",
                    config_.takeoff_max_tilt_deg, config_.takeoff_max_velocity_mps, settle_target);
            }

            const float tilt_limit = std::max(0.0f, config_.takeoff_max_tilt_deg);
            const float vel_limit  = std::max(0.0f, config_.takeoff_max_velocity_mps);
            const float roll_deg   = std::abs(fc_state.roll) * drone::util::kRadToDeg;
            const float pitch_deg  = std::abs(fc_state.pitch) * drone::util::kRadToDeg;
            const float vel_mag = std::sqrt(fc_state.vx * fc_state.vx + fc_state.vy * fc_state.vy +
                                            fc_state.vz * fc_state.vz);
            const bool  attitude_settled = std::isfinite(roll_deg) && std::isfinite(pitch_deg) &&
                                          std::isfinite(vel_mag) && roll_deg <= tilt_limit &&
                                          pitch_deg <= tilt_limit && vel_mag <= vel_limit;

            // Issue #777 — periodic snapshot diagnostic.  Throttled per
            // evaluation count (not wall-time) so the cadence is fixed in
            // observations-per-log, independent of FCState publish rate /
            // RTF.  Fires every 10th evaluation, including the first
            // (count==0).  Emits the actual roll/pitch/|v| values the gate
            // is measuring, so when run-4-class failure (60s held, no
            // release, no excursion log) reproduces, we can immediately
            // tell whether the FC's attitude was genuinely above thresholds
            // (config-tune territory) or some other failure mode is at
            // play (code/data-plumbing investigation).
            //
            // PR #779 Copilot review fix: field is named
            // `prev_settled_count` because `armed_settle_count_` is
            // captured *before* the `++` below.  So at eval #0 with
            // `this_obs_settled=true`, the log reads
            // `prev_settled_count=0/30` and the post-tick value is 1.
            // Operators reading traces in chronological order can subtract
            // to derive the current count, but the name avoids the
            // off-by-one confusion of calling it just "settled_count".
            if (layer4_eval_count_ % 10 == 0) {
                DRONE_LOG_INFO("[Layer4] eval #{} roll={:.1f}° pitch={:.1f}° |v|={:.2f}m/s — "
                               "prev_settled_count={}/{} (this_obs_settled={})",
                               layer4_eval_count_, roll_deg, pitch_deg, vel_mag,
                               armed_settle_count_, settle_target, attitude_settled);
            }
            ++layer4_eval_count_;

            if (attitude_settled) {
                ++armed_settle_count_;
                if (armed_settle_count_ >= static_cast<uint32_t>(settle_target)) {
                    DRONE_LOG_INFO("[Planner] Armed + attitude settled ({} obs: roll={:.1f}° "
                                   "pitch={:.1f}° |v|={:.2f}m/s) — initiating takeoff",
                                   armed_settle_count_, roll_deg, pitch_deg, vel_mag);
                    fsm.on_takeoff();
                    takeoff_sent_       = false;
                    armed_settle_count_ = 0;
                    layer4_eval_count_  = 0;  // #777 — reset eval counter on settled takeoff
                    // PR #775 review (code-quality P2.6): redundant
                    // `armable_first_seen_ns_ = 0` removed — already
                    // cleared at the top of the `if (fc_state.armed)`
                    // block 50+ lines up.
                    //
                    // Issue #718 — release on successful Layer 4 takeoff.
                    reset_preflight_timer();
                    return;
                }
                // Still accumulating consecutive settled observations.
                return;
            }

            // Excursion — attitude/velocity outside thresholds (or non-finite).
            // Reset the counter: the FC estimate must settle *continuously*,
            // not cumulatively.  Log once per excursion-onset (count > 0) so a
            // sustained-unsettled FC doesn't spam, but operators see why
            // takeoff is being withheld.
            //
            // PR #763 review (fault-recovery P2): logged at WARN — the drone
            // is armed (motors potentially spinning at idle) and the FC's
            // attitude estimate is moving outside safety thresholds.  That's
            // a degraded condition, not routine status.  An operator
            // monitoring at INFO would otherwise miss the excursion pattern
            // entirely.
            if (armed_settle_count_ > 0) {
                DRONE_LOG_WARN(
                    "[Planner] Armed but attitude not settled (roll={:.1f}° pitch={:.1f}° "
                    "|v|={:.2f}m/s; limits tilt={:.1f}° vel={:.2f}m/s) — settle counter reset",
                    roll_deg, pitch_deg, vel_mag, tilt_limit, vel_limit);
            }
            armed_settle_count_ = 0;
            return;
        }
        // Not armed — clear the Layer 4 settle counter so a future arm starts
        // a fresh consecutive-observation window (covers disarm + re-PREFLIGHT).
        // `now_ns` captured at top of function (see PR #741 review comment).
        armed_settle_count_ = 0;
        layer4_eval_count_  = 0;  // #777 — reset eval counter on disarm/re-PREFLIGHT
        if (!fc_state.armable) {
            // FC preflight not yet clear (EKF2 converging, sensors warming,
            // GPS lock acquiring, etc.).  Reset the stability tracker — any
            // future `armable=true` will start a fresh window.  Log on the
            // configurable wait-log throttle so operators can see we are
            // waiting on the FC and not stuck.  Uses a separate throttle
            // from `last_arm_time_ns_` so the first ARM fires promptly once
            // `armable` transitions to true and stays true through the
            // stability window.
            armable_first_seen_ns_ = 0;
            const uint64_t wait_log_interval_ns =
                static_cast<uint64_t>(std::max(0, config_.preflight_wait_log_s)) * 1'000'000'000ULL;
            if (fire_throttled(last_wait_log_time_ns_, now_ns, wait_log_interval_ns)) {
                DRONE_LOG_INFO("[Planner] Waiting for FC preflight (armable=false)");
            }
            return;
        }

        // armable == true — apply the #740 stability debounce.  First-time
        // observation starts the window; subsequent observations check
        // elapsed time against the configured threshold.
        const uint64_t window_ns = static_cast<uint64_t>(
            std::max(0.0, static_cast<double>(config_.preflight_armable_stable_s) * 1e9));

        // First observation of armable=true since the last reset.  Start
        // the window.  Guard against the unsigned-subtraction trap if the
        // clock runs backward (e.g. ScopedMockClock manual reset or
        // pathological host clock skew) by treating it as a fresh start.
        if (armable_first_seen_ns_ == 0 || now_ns < armable_first_seen_ns_) {
            armable_first_seen_ns_ = now_ns;
            if (window_ns > 0) {
                DRONE_LOG_INFO(
                    "[Planner] FC armable=true — beginning {:.1f}s stability gate before ARM",
                    config_.preflight_armable_stable_s);
                return;  // wait for the window to elapse on a subsequent tick
            }
            // window == 0: gate disabled by config — fall through and ARM
            // on this very tick (preserves legacy single-tick behaviour for
            // headless dev / unit tests that don't exercise the debounce).
        } else if ((now_ns - armable_first_seen_ns_) < window_ns) {
            // Still within stability window — defer ARM.
            return;
        }

        // Stability window satisfied — apply the existing ARM-retry throttle
        // (Issue #716) so duplicate ARMs aren't issued within `preflight_arm_
        // retry_s` of each other.
        const uint64_t retry_interval_ns =
            static_cast<uint64_t>(std::max(0, config_.preflight_arm_retry_s)) * 1'000'000'000ULL;
        if (fire_throttled(last_arm_time_ns_, now_ns, retry_interval_ns)) {
            DRONE_LOG_INFO("[Planner] FC armable stable for {:.1f}s — sending ARM command",
                           static_cast<double>(now_ns - armable_first_seen_ns_) * 1e-9);
            send_fc(drone::ipc::FCCommandType::ARM, 0.0f);
        }
    }

    // ── Record home from the first real pose (any state) ──────
    void try_record_home(const drone::ipc::Pose& pose) {
        if (home_recorded_) return;
        // Only accept poses with a non-zero timestamp (i.e. actually received
        // from the SLAM/VIO pipeline, not a default-constructed Pose{}).
        if (pose.timestamp_ns == 0) return;
        if (!std::isfinite(pose.translation[0]) || !std::isfinite(pose.translation[1])) return;

        home_x_        = static_cast<float>(pose.translation[0]);
        home_y_        = static_cast<float>(pose.translation[1]);
        home_z_        = 0.0f;
        home_recorded_ = true;
        DRONE_LOG_INFO("[Planner] Home position recorded: ({:.1f}, {:.1f}, {:.1f})", home_x_,
                       home_y_, home_z_);
    }

    // ── TAKEOFF: send TAKEOFF, wait for altitude + home ─────────
    void tick_takeoff(MissionFSM&                fsm, const drone::ipc::Pose& /*pose*/,
                      const drone::ipc::FCState& fc_state, const FCSendFn& send_fc) {
        if (!takeoff_sent_) {
            DRONE_LOG_INFO("[Planner] Sending TAKEOFF to {:.1f}m", config_.takeoff_alt_m);
            send_fc(drone::ipc::FCCommandType::TAKEOFF, config_.takeoff_alt_m);
            takeoff_sent_ = true;
        }
        if (fc_state.rel_alt >= config_.takeoff_alt_m * 0.9f) {
            if (!home_recorded_) {
                if (!home_warn_logged_) {
                    DRONE_LOG_WARN("[Planner] Takeoff altitude reached but no valid pose "
                                   "received yet — deferring NAVIGATE until home is recorded");
                    home_warn_logged_ = true;
                }
                return;
            }
            if (config_.survey_duration_s > 0.0f) {
                DRONE_LOG_INFO("[Planner] Takeoff complete (alt={:.1f}m) — SURVEY for {:.0f}s",
                               fc_state.rel_alt, config_.survey_duration_s);
                fsm.on_survey();
            } else {
                DRONE_LOG_INFO("[Planner] Takeoff complete (alt={:.1f}m) — NAVIGATE",
                               fc_state.rel_alt);
                fsm.on_navigate();
                DRONE_LOG_INFO("[FSM] EXECUTING — navigating {} waypoints", fsm.total_waypoints());
            }
        }
    }

    // ── SURVEY: hover + slow yaw rotation to detect obstacles ────
    void tick_survey(MissionFSM& fsm, const drone::ipc::Pose& pose,
                     const drone::ipc::DetectedObjectList& objects, IGridPlanner* grid_planner,
                     drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& traj_pub) {
        auto now = std::chrono::steady_clock::now();
        if (!survey_started_) {
            survey_start_time_ = now;
            survey_started_    = true;
            survey_start_yaw_  = yaw_from_pose(pose);
            DRONE_LOG_INFO("[Survey] Start yaw={:.2f} rad, rotating at {:.2f} rad/s for {:.0f}s",
                           survey_start_yaw_, config_.survey_yaw_rate, config_.survey_duration_s);
        }

        // Feed detections into the occupancy grid during survey
        if (grid_planner) {
            grid_planner->update_obstacles(objects, pose);
        }

        float elapsed_s = std::chrono::duration<float>(now - survey_start_time_).count();

        // Publish hover command with incrementing yaw for slow rotation.
        // IFCLink::send_trajectory only accepts absolute yaw (no yaw_rate),
        // so we compute the target yaw from elapsed time × yaw rate.
        // Two-phase survey: first half CW, second half CCW — sees each obstacle
        // from opposite directions, improving radar angular coverage.
        const float half_dur = config_.survey_duration_s * 0.5f;
        float       target_yaw;
        if (elapsed_s <= half_dur) {
            // Phase 1: clockwise
            target_yaw = survey_start_yaw_ + config_.survey_yaw_rate * elapsed_s;
        } else {
            // Phase 2: counter-clockwise from where phase 1 ended
            const float phase1_end = survey_start_yaw_ + config_.survey_yaw_rate * half_dur;
            target_yaw             = phase1_end - config_.survey_yaw_rate * (elapsed_s - half_dur);
        }
        // Wrap to [-π, π] to avoid sending unbounded angles to the FC.
        constexpr float kPiF = 3.14159265358979323846f;
        target_yaw           = std::fmod(target_yaw + kPiF, 2.0f * kPiF);
        if (target_yaw < 0.0f) target_yaw += 2.0f * kPiF;
        target_yaw -= kPiF;

        drone::ipc::TrajectoryCmd cmd{};
        cmd.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        cmd.valid      = true;
        cmd.velocity_x = 0.0f;
        cmd.velocity_y = 0.0f;
        cmd.velocity_z = 0.0f;
        cmd.target_yaw = target_yaw;
        cmd.target_x   = static_cast<float>(pose.translation[0]);
        cmd.target_y   = static_cast<float>(pose.translation[1]);
        cmd.target_z   = static_cast<float>(pose.translation[2]);
        traj_pub.publish(cmd);

        // Log progress periodically
        if (survey_log_tick_++ % 30 == 0) {
            int promoted = grid_planner ? grid_planner->grid_promoted_count() : 0;
            int static_n = grid_planner ? static_cast<int>(grid_planner->grid_static_count()) : 0;
            DRONE_LOG_INFO(
                "[Survey] {:.0f}/{:.0f}s — {} static cells ({} promoted), yaw_rate={:.2f}",
                elapsed_s, config_.survey_duration_s, static_n, promoted, config_.survey_yaw_rate);
        }

        // Survey complete — transition to NAVIGATE
        if (elapsed_s >= config_.survey_duration_s) {
            int promoted = grid_planner ? grid_planner->grid_promoted_count() : 0;
            int static_n = grid_planner ? static_cast<int>(grid_planner->grid_static_count()) : 0;
            DRONE_LOG_INFO("[Planner] Survey complete ({:.0f}s) — {} static obstacles ({} promoted)"
                           " — NAVIGATE",
                           elapsed_s, static_n, promoted);
            fsm.on_navigate();
            DRONE_LOG_INFO("[FSM] EXECUTING — navigating {} waypoints", fsm.total_waypoints());
        }
    }

    // ── NAVIGATE: waypoint tracking, collision detect, plan+avoid ─
    void tick_navigate(MissionFSM& fsm, const drone::ipc::Pose& pose,
                       const drone::ipc::FCState&            fc_state,
                       const drone::ipc::DetectedObjectList& objects, IPathPlanner& planner,
                       IGridPlanner* grid_planner, IObstacleAvoider& avoider,
                       StaticObstacleLayer&                                obstacle_layer,
                       drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>&  traj_pub,
                       drone::ipc::IPublisher<drone::ipc::PayloadCommand>& payload_pub,
                       const FCSendFn& send_fc, uint64_t correlation_id,
                       drone::util::FrameDiagnostics& diag) {
        // Detect unexpected disarm mid-navigation.
        // A disarmed vehicle cannot execute recovery commands (hover/climb), so
        // transition to IDLE rather than COLLISION_RECOVERY. Recovery is only
        // useful when triggered by proximity detection while still armed.
        if (flight_state_.nav_was_armed && !fc_state.armed) {
            DRONE_LOG_WARN("[Planner] Vehicle unexpectedly disarmed during navigation — "
                           "cannot recover without motors, transitioning to IDLE");
            publish_stop_trajectory(traj_pub, correlation_id);
            flight_state_.nav_was_armed = fc_state.armed;
            fsm.on_landed();
            return;
        }
        flight_state_.nav_was_armed = fc_state.armed;

        // Proximity collision detection
        {
            const float px        = static_cast<float>(pose.translation[0]);
            const float py        = static_cast<float>(pose.translation[1]);
            const float pz        = static_cast<float>(pose.translation[2]);
            bool        collision = obstacle_layer.check_collision(px, py, pz,
                                                                   std::chrono::steady_clock::now());
            if (collision && config_.collision_recovery_enabled) {
                DRONE_LOG_INFO("[Planner] Proximity collision — entering COLLISION_RECOVERY");
                publish_stop_trajectory(traj_pub, correlation_id);
                recovery_started_ = false;
                fsm.on_collision_recovery();
                return;
            }
        }

        // Update planner obstacle grid
        if (grid_planner) {
            drone::util::ScopedDiagTimer t(diag, "GridUpdate");
            grid_planner->update_obstacles(objects, pose);
        }

        // Warn about approaching unconfirmed obstacles
        {
            const float px = static_cast<float>(pose.translation[0]);
            const float py = static_cast<float>(pose.translation[1]);
            obstacle_layer.check_unconfirmed_approach(px, py, std::chrono::steady_clock::now());
        }

        const Waypoint* wp = fsm.current_waypoint();
        if (!wp) return;

        {
            auto planned = [&]() {
                drone::util::ScopedDiagTimer t(diag, "PathPlan");
                return planner.plan(pose, *wp);
            }();
            if (grid_planner && grid_planner->using_direct_fallback()) {
                // Despite the legacy `using_direct_fallback()` name, the planner
                // hovers when search fails with no cached path — it does NOT
                // fly a direct line through obstacles.  Surface the accurate
                // behaviour and the cumulative hover-fallback count so an
                // operator (or replay) can tell when the grid has become
                // over-occupied for the current waypoint (Plan A — Issue #645
                // follow-up; see `GridPlannerBase::plan()` for the hover
                // branch).
                diag.add_warning("PathPlan",
                                 "Planner: no obstacle-free path — hovering in place (no "
                                 "cached path to follow); cumulative hover-fallback count=" +
                                     std::to_string(grid_planner->hover_fallback_count()));
            }
            auto traj = [&]() {
                drone::util::ScopedDiagTimer t(diag, "ObstacleAvoid");
                return avoider.avoid(planned, pose, objects);
            }();

            // ── Issue #624 — post-avoider yaw-towards-velocity refresh ────
            // #337 wired yaw-towards-velocity inside the planner using the
            // planner's smoothed velocity (pre-avoidance).  When the avoider
            // deflects sideways to route around an obstacle, target_yaw is
            // left pointing at the planner's straight-line heading — camera
            // and forward-facing sensors lose the obstacle to the drone's
            // blind spot, so the drone drifts back into it from an angle
            // it never had voxels for.  Refresh target_yaw from the
            // avoider's post-deflection velocity so the camera tracks the
            // actual flight direction.
            //
            // No-op when the planner's `yaw_towards_velocity` flag is off
            // (every scenario that doesn't opt in).  Reuses the planner's
            // own threshold so the two yaw paths agree on "too slow to
            // yaw safely" — prevents oscillation during hover.
            //
            // Measured on scenario 33 pre-fix: drone body yaw of +9° while
            // actual motion heading -91° (= 100° misalignment) during the
            // cube-approach sidestep; drone collided head-on.
            if (grid_planner && grid_planner->yaw_towards_velocity_enabled()) {
                const float vx      = traj.velocity_x;
                const float vy      = traj.velocity_y;
                const float v_xy_sq = vx * vx + vy * vy;
                const float thr     = grid_planner->yaw_velocity_threshold_mps();
                if (v_xy_sq > thr * thr) {
                    traj.target_yaw = std::atan2(vy, vx);
                }
            }

            // Diagnostic every 10 ticks (~1s at 10Hz) — gated by logger runtime level.
            if (::drone::log::logger().should_log(::drone::log::Level::Debug) &&
                diag_tick_++ % 10 == 0) {
                const float dpx        = static_cast<float>(pose.translation[0]);
                const float dpy        = static_cast<float>(pose.translation[1]);
                const float dpz        = static_cast<float>(pose.translation[2]);
                float       dvx        = traj.velocity_x - planned.velocity_x;
                float       dvy        = traj.velocity_y - planned.velocity_y;
                float       dvz        = traj.velocity_z - planned.velocity_z;
                float       dmag       = std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz);
                float       dist_to_wp = std::sqrt((dpx - wp->x) * (dpx - wp->x) +
                                                   (dpy - wp->y) * (dpy - wp->y) +
                                                   (dpz - wp->z) * (dpz - wp->z));
                int occ = grid_planner ? static_cast<int>(grid_planner->grid_occupied_count()) : -1;
                int stat = grid_planner ? static_cast<int>(grid_planner->grid_static_count()) : -1;
                int prom = grid_planner ? grid_planner->grid_promoted_count() : -1;
                bool fb  = grid_planner && grid_planner->using_direct_fallback();

                // Active-object count (Issue #503): obstacles within the avoider
                // influence radius at current pose.  Useful to cross-reference
                // when the avoider emits no correction despite obstacles nearby.
                int active_obj = 0;
                for (uint32_t i = 0; i < objects.num_objects; ++i) {
                    const auto& o  = objects.objects[i];
                    const float ox = o.position_x - dpx;
                    const float oy = o.position_y - dpy;
                    const float oz = o.position_z - dpz;
                    const float d  = std::sqrt(ox * ox + oy * oy + oz * oz);
                    if (d < config_.avoider_influence_radius_m) ++active_obj;
                }

                DRONE_LOG_DEBUG("[DIAG] pos=({:.1f},{:.1f},{:.1f}) wp{}/{}=({:.0f},{:.0f},{:.0f})"
                                " dist={:.1f}m plan_v=({:.2f},{:.2f},{:.2f})"
                                " avoid_v=({:.2f},{:.2f},{:.2f}) |delta|={:.2f}"
                                " grid: occ={} static={} promoted={} fallback={}"
                                " active_obj={}",
                                dpx, dpy, dpz, fsm.current_wp_index() + 1, fsm.total_waypoints(),
                                wp->x, wp->y, wp->z, dist_to_wp, planned.velocity_x,
                                planned.velocity_y, planned.velocity_z, traj.velocity_x,
                                traj.velocity_y, traj.velocity_z, dmag, occ, stat, prom, fb,
                                active_obj);
            }

            // Stuck detector (Issue #503) — feed before publish so any stuck
            // transition is recognised before the next replan cycle.  Only
            // fires in NAVIGATE; the enclosing switch guarantees that.  The
            // detector intentionally does NOT gate on avoider activity —
            // when the drone collides with geometry, LiDAR returns drop
            // (mesh intersects drone body) and the avoider goes inactive
            // precisely when we most need to detect the stall.  Pose-based
            // "not moving in NAVIGATE" is the authoritative signal.
            {
                const float sx        = static_cast<float>(pose.translation[0]);
                const float sy        = static_cast<float>(pose.translation[1]);
                const float sz        = static_cast<float>(pose.translation[2]);
                const auto  now_clock = std::chrono::steady_clock::now();
                stuck_detector_.push_sample(now_clock, sx, sy, sz);
                if (stuck_detector_.is_stuck(now_clock)) {
                    // Rate-cap: if we've re-triggered STUCK max_stuck_count
                    // times this mission, escalate to LOITER with
                    // fault_triggered so the operator sees persistent stall
                    // in health telemetry.  Silent oscillation is the worst
                    // outcome: the drone keeps bouncing off the obstacle but
                    // the fault-recovery chain never engages.
                    const auto& stuck_cfg = stuck_detector_.config();
                    ++stuck_transition_count_;
                    if (stuck_cfg.max_stuck_count > 0 &&
                        stuck_transition_count_ > stuck_cfg.max_stuck_count) {
                        DRONE_LOG_WARN("[FSM] STUCK count {} exceeded cap {} — escalating to "
                                       "LOITER at ({:.1f},{:.1f},{:.1f})",
                                       stuck_transition_count_, stuck_cfg.max_stuck_count, sx, sy,
                                       sz);
                        publish_stop_trajectory(traj_pub, correlation_id);
                        stuck_detector_.reset();
                        fsm.set_fault_triggered(true);
                        // Surface FAULT_STUCK in the published fault bitmask
                        // so GCS + P7 health monitor register the stall.
                        // Cleared on resume to NAVIGATE or transition to IDLE.
                        flight_state_.stuck_fault_active = true;
                        fsm.on_loiter();
                        return;
                    }
                    DRONE_LOG_INFO("[FSM] STUCK detected ({}/{}) at ({:.1f},{:.1f},{:.1f}) "
                                   "— backing off",
                                   stuck_transition_count_, stuck_cfg.max_stuck_count, sx, sy, sz);
                    // Remember the last meaningful velocity so unstuck can
                    // back off along the opposite direction.
                    unstuck_vx_         = planned.velocity_x;
                    unstuck_vy_         = planned.velocity_y;
                    unstuck_vz_         = planned.velocity_z;
                    unstuck_start_time_ = now_clock;
                    stuck_detector_.reset();
                    fsm.on_stuck();
                    return;
                }
            }

            traj_pub.publish(traj);

            const float px = static_cast<float>(pose.translation[0]);
            const float py = static_cast<float>(pose.translation[1]);
            const float pz = static_cast<float>(pose.translation[2]);

            // When the planner snapped the goal away from the original waypoint
            // (because it was occupied), check acceptance against the snapped
            // position — otherwise the drone reaches the navigation target but
            // can never satisfy the radius check.  See Issue #394.
            // Uses IGridPlanner interface — no dynamic_cast needed (Issue #398).
            const std::optional<std::array<float, 3>> snap_opt =
                (grid_planner && grid_planner->has_snapped_goal())
                    ? grid_planner->snapped_goal_xyz()
                    : std::nullopt;
            const std::array<float, 3>* snap_xyz = snap_opt.has_value() ? &snap_opt.value()
                                                                        : nullptr;

            const bool reached  = fsm.waypoint_reached(px, py, pz, *wp, snap_xyz);
            const bool overshot = fsm.waypoint_overshot(px, py, pz, snap_opt);
            if (reached || overshot) {
                DRONE_LOG_INFO("[Planner] Waypoint {} {}!", fsm.current_wp_index() + 1,
                               overshot ? "overshot" : "reached");

                if (wp->trigger_payload) {
                    drone::ipc::PayloadCommand pay_cmd{};
                    pay_cmd.timestamp_ns   = traj.timestamp_ns;
                    pay_cmd.correlation_id = correlation_id;
                    pay_cmd.action         = drone::ipc::PayloadAction::CAMERA_CAPTURE;
                    pay_cmd.gimbal_pitch   = -90.0f;
                    pay_cmd.gimbal_yaw     = 0.0f;
                    pay_cmd.valid          = true;
                    payload_pub.publish(pay_cmd);
                }

                // Progress → clear STUCK counter. Each waypoint gets a fresh
                // allowance; repeated stalls against the SAME obstacle still
                // escalate, but a legitimate wider mission isn't punished.
                stuck_transition_count_ = 0;
                if (!fsm.advance_waypoint()) {
                    DRONE_LOG_INFO("[Planner] Mission complete — RTL");
                    send_fc(drone::ipc::FCCommandType::RTL, 0.0f);
                    publish_stop_trajectory(traj_pub, correlation_id);
                    flight_state_.rtl_start_time = std::chrono::steady_clock::now();
                    flight_state_.nav_was_armed  = true;
                    fsm.on_rtl();
                } else if (grid_planner) {
                    // Invalidate snap cache on waypoint advance — the snap from
                    // the previous waypoint must not carry over to the next one.
                    // Without this, waypoint_reached() checks the new waypoint
                    // against the old snap position until the next replan cycle
                    // (~1s), causing false "reached" and rapid advancement.
                    grid_planner->invalidate_path();
                }
            }
        }
    }

    // ── COLLISION_RECOVERY: hover → climb → replan → NAVIGATE (Issue #226) ──
    void tick_collision_recovery(MissionFSM& fsm, const drone::ipc::Pose& pose,
                                 const drone::ipc::FCState& fc_state, IGridPlanner* grid_planner,
                                 drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& traj_pub) {
        // Safety: abort recovery if disarmed or FC disconnected
        if (!fc_state.armed || !fc_state.connected) {
            DRONE_LOG_WARN("[Recovery] FC {} during recovery — aborting to IDLE",
                           !fc_state.armed ? "disarmed" : "disconnected");
            recovery_started_ = false;
            fsm.on_landed();
            return;
        }

        auto now = std::chrono::steady_clock::now();
        if (!recovery_started_) {
            recovery_start_time_ = now;
            recovery_phase_      = RecoveryPhase::HOVER;
            recovery_target_alt_ = static_cast<float>(pose.translation[2]) +
                                   config_.collision_climb_delta_m;
            recovery_started_ = true;
            DRONE_LOG_INFO("[Recovery] Phase HOVER — holding position for {:.1f}s, "
                           "then climbing to {:.1f}m",
                           config_.collision_hover_duration_s, recovery_target_alt_);
        }

        const float px        = static_cast<float>(pose.translation[0]);
        const float py        = static_cast<float>(pose.translation[1]);
        const float pz        = static_cast<float>(pose.translation[2]);
        float       elapsed_s = std::chrono::duration<float>(now - recovery_start_time_).count();

        const float current_yaw = yaw_from_pose(pose);

        switch (recovery_phase_) {
            case RecoveryPhase::HOVER: {
                // Publish hover-in-place command
                drone::ipc::TrajectoryCmd cmd{};
                cmd.timestamp_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch())
                        .count());
                cmd.valid      = true;
                cmd.target_x   = px;
                cmd.target_y   = py;
                cmd.target_z   = pz;
                cmd.target_yaw = current_yaw;
                cmd.velocity_x = 0.0f;
                cmd.velocity_y = 0.0f;
                cmd.velocity_z = 0.0f;
                traj_pub.publish(cmd);

                if (elapsed_s >= config_.collision_hover_duration_s) {
                    recovery_phase_ = RecoveryPhase::CLIMB;
                    DRONE_LOG_INFO("[Recovery] Phase CLIMB — ascending {:.1f}m to {:.1f}m",
                                   config_.collision_climb_delta_m, recovery_target_alt_);
                }
                break;
            }
            case RecoveryPhase::CLIMB: {
                // Publish climb command — move to recovery_target_alt_
                drone::ipc::TrajectoryCmd cmd{};
                cmd.timestamp_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch())
                        .count());
                cmd.valid      = true;
                cmd.target_x   = px;
                cmd.target_y   = py;
                cmd.target_z   = recovery_target_alt_;
                cmd.target_yaw = current_yaw;
                cmd.velocity_x = 0.0f;
                cmd.velocity_y = 0.0f;
                cmd.velocity_z = 1.0f;  // gentle climb rate
                traj_pub.publish(cmd);

                constexpr float kAltTolerance = 0.5f;
                if (pz >= recovery_target_alt_ - kAltTolerance) {
                    recovery_phase_ = RecoveryPhase::REPLAN;
                    DRONE_LOG_INFO("[Recovery] Phase REPLAN — altitude {:.1f}m reached, "
                                   "resuming navigation",
                                   pz);
                }
                break;
            }
            case RecoveryPhase::REPLAN: {
                // Invalidate cached path to force a full replan after collision
                if (grid_planner) {
                    grid_planner->invalidate_path();
                }
                DRONE_LOG_INFO("[Recovery] Complete — transitioning to NAVIGATE");
                recovery_started_ = false;
                fsm.on_recovery_complete();
                break;
            }
        }
    }

    // ── NAVIGATE_UNSTUCK: back off opposite planned velocity, then resume ──
    // Issue #503 — when the stuck detector fires, we know the avoider was
    // pushing but the vehicle hadn't moved in window_s.  Commanding a short
    // backoff (typically away from the planner-desired direction) breaks the
    // deadlock so the next NAVIGATE replan picks a wider route.
    void tick_navigate_unstuck(MissionFSM& fsm, const drone::ipc::Pose& pose,
                               const drone::ipc::FCState&                         fc_state,
                               drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& traj_pub) {
        // Abort gracefully if the vehicle disarms mid-maneuver.
        if (!fc_state.armed) {
            DRONE_LOG_WARN("[Unstuck] Disarmed during backoff — transitioning to IDLE");
            // Reset stuck state so a fresh mission arm starts with a clean
            // counter/flag (Issue #503 review: stale counter across missions).
            stuck_transition_count_          = 0;
            flight_state_.stuck_fault_active = false;
            fsm.on_landed();
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (unstuck_start_time_ == std::chrono::steady_clock::time_point{}) {
            unstuck_start_time_ = now;
        }
        const float elapsed_s = std::chrono::duration<float>(now - unstuck_start_time_).count();

        // Invert the planned velocity and clamp its magnitude to
        // backoff_speed_mps.  This pushes the drone opposite to the direction
        // that got it stuck.
        float       vx  = -unstuck_vx_;
        float       vy  = -unstuck_vy_;
        float       vz  = -unstuck_vz_;
        const float mag = std::sqrt(vx * vx + vy * vy + vz * vz);
        const float cap = config_.stuck_detector.backoff_speed_mps;
        if (mag > cap && mag > 1e-3f) {
            const float s = cap / mag;
            vx *= s;
            vy *= s;
            vz *= s;
        } else if (mag < 1e-3f) {
            // Fallback — if we had no planned velocity to invert, hover.
            vx = vy = vz = 0.0f;
        }

        const float current_yaw = yaw_from_pose(pose);

        drone::ipc::TrajectoryCmd cmd{};
        cmd.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        cmd.valid      = true;
        cmd.velocity_x = vx;
        cmd.velocity_y = vy;
        cmd.velocity_z = vz;
        cmd.target_x   = static_cast<float>(pose.translation[0]);
        cmd.target_y   = static_cast<float>(pose.translation[1]);
        cmd.target_z   = static_cast<float>(pose.translation[2]);
        cmd.target_yaw = current_yaw;
        traj_pub.publish(cmd);

        // Exit when backoff duration expires.  Resume NAVIGATE and clear
        // detector history so the first seconds of new motion aren't counted
        // as "stuck".
        if (elapsed_s >= config_.stuck_detector.backoff_duration_s) {
            DRONE_LOG_INFO("[Unstuck] Backoff complete ({:.1f}s) — resuming NAVIGATE", elapsed_s);
            unstuck_start_time_ = std::chrono::steady_clock::time_point{};
            stuck_detector_.reset();
            // Backoff completing is "we got out" — the stuck-escalated LOITER
            // fault (if any) is no longer applicable.  Clear the wire-format
            // flag so GCS + P7 monitor see the drone as healthy again.
            flight_state_.stuck_fault_active = false;
            fsm.on_unstuck();
        }
    }

    // ── RTL: detect disarm, monitor position for LAND ─────────
    void tick_rtl(MissionFSM& fsm, const drone::ipc::Pose& pose,
                  const drone::ipc::FCState& fc_state, const FCSendFn& send_fc) {
        if (flight_state_.nav_was_armed && !fc_state.armed) {
            DRONE_LOG_WARN("[Planner] Vehicle disarmed during RTL — mission IDLE");
            fsm.on_landed();
            return;
        }
        flight_state_.nav_was_armed = fc_state.armed;

        if (home_recorded_) {
            auto rtl_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                   std::chrono::steady_clock::now() - flight_state_.rtl_start_time)
                                   .count();
            float dx         = static_cast<float>(pose.translation[0]) - home_x_;
            float dy         = static_cast<float>(pose.translation[1]) - home_y_;
            float horiz_dist = std::sqrt(dx * dx + dy * dy);
            if (rtl_elapsed >= config_.rtl_min_dwell_s && horiz_dist < config_.rtl_acceptance_m) {
                DRONE_LOG_INFO("[Planner] Near home ({:.1f}m, {}s in RTL) — sending LAND",
                               horiz_dist, rtl_elapsed);
                send_fc(drone::ipc::FCCommandType::LAND, 0.0f);
                flight_state_.land_sent = true;
                fsm.on_land();
            }
        }
    }

    // ── LAND: wait for touchdown ──────────────────────────────
    void tick_land(MissionFSM& fsm, const drone::ipc::FCState& fc_state) {
        if (fc_state.rel_alt < config_.landed_alt_m && flight_state_.land_sent) {
            DRONE_LOG_INFO("[Planner] Landed (alt={:.2f}m) — mission IDLE", fc_state.rel_alt);
            fsm.on_landed();
            fault_exec_reset_ = true;
        }
    }

    // ── Utility: publish an immediate zero-velocity stop trajectory ──
    // Sends a valid command with zero velocities so P5 forwards it to the FC.
    // (P5 skips commands with valid=false, so we must send valid=true to stop.)
    static void publish_stop_trajectory(drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& pub,
                                        uint64_t correlation_id = 0) {
        drone::ipc::TrajectoryCmd stop{};
        stop.valid          = true;
        stop.velocity_x     = 0.0f;
        stop.velocity_y     = 0.0f;
        stop.velocity_z     = 0.0f;
        stop.correlation_id = correlation_id;
        stop.timestamp_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        pub.publish(stop);
    }
};

}  // namespace drone::planner
