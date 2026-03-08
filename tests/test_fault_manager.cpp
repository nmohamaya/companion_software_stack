// tests/test_fault_manager.cpp
// Unit tests for FaultManager graceful-degradation engine.
#include "planner/fault_manager.h"

#include <cstring>

#include <gtest/gtest.h>

using namespace drone::planner;
using drone::ipc::ShmSystemHealth;
using drone::ipc::ShmFCState;

// ═══════════════════════════════════════════════════════════
// Helpers — build fresh health / fc_state structs
// ═══════════════════════════════════════════════════════════
namespace {

constexpr uint64_t MS = 1'000'000ULL;
constexpr uint64_t S  = 1'000'000'000ULL;

/// Return a nominal ShmSystemHealth (all OK).
ShmSystemHealth make_healthy() {
    ShmSystemHealth h{};
    h.timestamp_ns      = 1'000 * S;
    h.cpu_usage_percent = 30.0f;
    h.thermal_zone      = 0;
    h.critical_failure  = false;
    h.num_processes     = 0;
    return h;
}

/// Return a nominal ShmFCState (connected, battery full).
ShmFCState make_fc_ok() {
    ShmFCState fc{};
    fc.timestamp_ns      = 1'000 * S;
    fc.connected         = true;
    fc.battery_remaining = 80.0f;
    fc.battery_voltage   = 16.0f;
    fc.armed             = true;
    return fc;
}

/// Add a process entry to health.
void add_process(ShmSystemHealth& h, const char* name, bool alive) {
    ASSERT_LT(h.num_processes, drone::ipc::kMaxTrackedProcesses)
        << "add_process() would overflow processes[]";
    auto& p = h.processes[h.num_processes];
    std::strncpy(p.name, name, sizeof(p.name) - 1);
    p.alive        = alive;
    p.last_seen_ns = h.timestamp_ns;
    ++h.num_processes;
}

/// Default config for tests.
FaultConfig default_cfg() {
    FaultConfig c;
    c.pose_stale_timeout_ns        = 500 * MS;  // 500 ms
    c.battery_warn_percent         = 30.0f;
    c.battery_rtl_percent          = 20.0f;
    c.battery_crit_percent         = 10.0f;
    c.fc_link_lost_timeout_ns      = 3 * S;   // 3 s
    c.fc_link_rtl_timeout_ns       = 15 * S;  // 15 s
    c.loiter_escalation_timeout_ns = 30 * S;  // 30 s
    return c;
}

}  // namespace

// ═══════════════════════════════════════════════════════════
// Nominal — no faults
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, NominalReturnsNone) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    auto         fc     = make_fc_ok();
    uint64_t     now    = 1'000 * S + 100 * MS;  // 100 ms after health ts

    auto result = mgr.evaluate(health, fc, now - 50 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::NONE);
    EXPECT_EQ(result.active_faults, FAULT_NONE);
    EXPECT_EQ(mgr.high_water_mark(), FaultAction::NONE);
}

// ═══════════════════════════════════════════════════════════
// Critical process death → LOITER
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, CriticalProcessDeath) {
    FaultManager mgr(default_cfg());
    auto         health     = make_healthy();
    health.critical_failure = true;
    auto     fc             = make_fc_ok();
    uint64_t now            = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::LOITER);
    EXPECT_TRUE(result.active_faults & FAULT_CRITICAL_PROCESS);
}

// ═══════════════════════════════════════════════════════════
// Pose staleness → LOITER
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, PoseStaleTriggersLoiter) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    auto         fc     = make_fc_ok();
    uint64_t     now    = 1'000 * S;

    // Pose is 600 ms old — exceeds 500 ms threshold
    auto result = mgr.evaluate(health, fc, now - 600 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::LOITER);
    EXPECT_TRUE(result.active_faults & FAULT_POSE_STALE);
}

TEST(FaultManagerTest, PoseNotStaleBelowTimeout) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    auto         fc     = make_fc_ok();
    uint64_t     now    = 1'000 * S;

    // Pose is 400 ms old — below 500 ms threshold
    auto result = mgr.evaluate(health, fc, now - 400 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::NONE);
    EXPECT_EQ(result.active_faults & FAULT_POSE_STALE, 0u);
}

// ═══════════════════════════════════════════════════════════
// Battery low → RTL
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, BatteryLowTriggersRTL) {
    FaultManager mgr(default_cfg());
    auto         health  = make_healthy();
    auto         fc      = make_fc_ok();
    fc.battery_remaining = 15.0f;  // below 20% RTL threshold
    uint64_t now         = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::RTL);
    EXPECT_TRUE(result.active_faults & FAULT_BATTERY_RTL);
    EXPECT_FALSE(result.active_faults & FAULT_BATTERY_CRITICAL);
}

// ═══════════════════════════════════════════════════════════
// Battery critical → EMERGENCY_LAND
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, BatteryCriticalTriggersEmergencyLand) {
    FaultManager mgr(default_cfg());
    auto         health  = make_healthy();
    auto         fc      = make_fc_ok();
    fc.battery_remaining = 8.0f;  // below 10% crit
    uint64_t now         = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::EMERGENCY_LAND);
    EXPECT_TRUE(result.active_faults & FAULT_BATTERY_CRITICAL);
    // battery_critical is checked first, so BATTERY_LOW should not be set
    EXPECT_FALSE(result.active_faults & FAULT_BATTERY_LOW);
}

// ═══════════════════════════════════════════════════════════
// Thermal warning → WARN
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, ThermalWarningTriggersWarn) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    health.thermal_zone = 2;
    auto     fc         = make_fc_ok();
    uint64_t now        = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::WARN);
    EXPECT_TRUE(result.active_faults & FAULT_THERMAL_WARNING);
}

// ═══════════════════════════════════════════════════════════
// Thermal critical → RTL
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, ThermalCriticalTriggersRTL) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    health.thermal_zone = 3;
    auto     fc         = make_fc_ok();
    uint64_t now        = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::RTL);
    EXPECT_TRUE(result.active_faults & FAULT_THERMAL_CRITICAL);
}

// ═══════════════════════════════════════════════════════════
// Perception dead → WARN
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, PerceptionDeadTriggersWarn) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    add_process(health, "perception", false);  // dead
    auto     fc  = make_fc_ok();
    uint64_t now = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::WARN);
    EXPECT_TRUE(result.active_faults & FAULT_PERCEPTION_DEAD);
}

TEST(FaultManagerTest, PerceptionAliveNoFault) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    add_process(health, "perception", true);  // alive
    auto     fc  = make_fc_ok();
    uint64_t now = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::NONE);
    EXPECT_FALSE(result.active_faults & FAULT_PERCEPTION_DEAD);
}

// ═══════════════════════════════════════════════════════════
// FC link lost → LOITER
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, FCLinkLostTriggersLoiter) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    auto         fc     = make_fc_ok();
    fc.connected        = false;
    fc.timestamp_ns     = 1'000 * S;
    uint64_t now        = 1'000 * S + 4 * S;  // 4 s with no FC → exceeds 3 s

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::LOITER);
    EXPECT_TRUE(result.active_faults & FAULT_FC_LINK_LOST);
}

TEST(FaultManagerTest, FCLinkLostBriefNoFault) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    auto         fc     = make_fc_ok();
    fc.connected        = false;
    fc.timestamp_ns     = 1'000 * S;
    uint64_t now        = 1'000 * S + 2 * S;  // only 2 s — below 3 s threshold

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::NONE);
    EXPECT_FALSE(result.active_faults & FAULT_FC_LINK_LOST);
}

// ═══════════════════════════════════════════════════════════
// Escalation-only policy — can never downgrade
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, EscalationOnlyNeverDowngrades) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    auto         fc     = make_fc_ok();
    uint64_t     now    = 1'000 * S;

    // First: battery low → RTL
    fc.battery_remaining = 15.0f;
    auto r1              = mgr.evaluate(health, fc, now - 10 * MS, now);
    EXPECT_EQ(r1.recommended_action, FaultAction::RTL);
    EXPECT_EQ(mgr.high_water_mark(), FaultAction::RTL);

    // Second: battery recovers (hypothetical) — should stay RTL
    fc.battery_remaining = 80.0f;
    auto r2              = mgr.evaluate(health, fc, now - 10 * MS, now + 1 * S);
    EXPECT_EQ(r2.recommended_action, FaultAction::RTL);
    EXPECT_EQ(mgr.high_water_mark(), FaultAction::RTL);
}

// ═══════════════════════════════════════════════════════════
// Loiter timeout escalation → RTL after 30 s
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, LoiterEscalatesToRTLAfterTimeout) {
    FaultConfig cfg                  = default_cfg();
    cfg.loiter_escalation_timeout_ns = 5 * S;  // 5 s for test speed
    FaultManager mgr(cfg);

    auto health             = make_healthy();
    health.critical_failure = true;  // → LOITER
    auto fc                 = make_fc_ok();

    // t=0: LOITER starts
    uint64_t t0 = 1'000 * S;
    auto     r1 = mgr.evaluate(health, fc, t0 - 10 * MS, t0);
    EXPECT_EQ(r1.recommended_action, FaultAction::LOITER);

    // t=3s: still LOITER
    auto r2 = mgr.evaluate(health, fc, t0 + 3 * S - 10 * MS, t0 + 3 * S);
    EXPECT_EQ(r2.recommended_action, FaultAction::LOITER);

    // t=6s: exceeds 5 s → RTL
    auto r3 = mgr.evaluate(health, fc, t0 + 6 * S - 10 * MS, t0 + 6 * S);
    EXPECT_EQ(r3.recommended_action, FaultAction::RTL);
    EXPECT_EQ(mgr.high_water_mark(), FaultAction::RTL);
}

// ═══════════════════════════════════════════════════════════
// LOITER cause cleared but high-water mark keeps LOITER → RTL after timeout
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, LoiterCauseClearedStillEscalatesViaHighWaterMark) {
    FaultConfig cfg                  = default_cfg();
    cfg.loiter_escalation_timeout_ns = 5 * S;
    FaultManager mgr(cfg);

    auto health_fault             = make_healthy();
    health_fault.critical_failure = true;            // → LOITER
    auto health_ok                = make_healthy();  // nominal
    auto fc                       = make_fc_ok();

    // t=0: LOITER starts (cause active)
    uint64_t t0 = 1'000 * S;
    auto     r1 = mgr.evaluate(health_fault, fc, t0 - 10 * MS, t0);
    EXPECT_EQ(r1.recommended_action, FaultAction::LOITER);

    // t=2s: cause clears — high-water mark should keep LOITER
    auto r2 = mgr.evaluate(health_ok, fc, t0 + 2 * S - 10 * MS, t0 + 2 * S);
    EXPECT_EQ(r2.recommended_action, FaultAction::LOITER)
        << "high-water mark should maintain LOITER even after cause clears";

    // t=6s: exceeds 5 s total — should escalate to RTL
    auto r3 = mgr.evaluate(health_ok, fc, t0 + 6 * S - 10 * MS, t0 + 6 * S);
    EXPECT_EQ(r3.recommended_action, FaultAction::RTL)
        << "loiter timeout should escalate to RTL even when original cause cleared";
    EXPECT_EQ(mgr.high_water_mark(), FaultAction::RTL);
}

// ═══════════════════════════════════════════════════════════
// Reset clears high-water mark
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, ResetClearsHighWaterMark) {
    FaultManager mgr(default_cfg());
    auto         health  = make_healthy();
    auto         fc      = make_fc_ok();
    fc.battery_remaining = 15.0f;  // → RTL
    uint64_t now         = 1'000 * S;

    (void)mgr.evaluate(health, fc, now - 10 * MS, now);  // side-effect: sets high_water_mark
    EXPECT_EQ(mgr.high_water_mark(), FaultAction::RTL);

    mgr.reset();
    EXPECT_EQ(mgr.high_water_mark(), FaultAction::NONE);

    // After reset, nominal input → NONE
    fc.battery_remaining = 80.0f;
    auto result          = mgr.evaluate(health, fc, now + 1 * S - 10 * MS, now + 1 * S);
    EXPECT_EQ(result.recommended_action, FaultAction::NONE);
}

// ═══════════════════════════════════════════════════════════
// Multiple simultaneous faults — highest wins
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, MultipleFaultsHighestActionWins) {
    FaultManager mgr(default_cfg());
    auto         health     = make_healthy();
    health.critical_failure = true;  // → LOITER
    health.thermal_zone     = 3;     // → RTL
    auto fc                 = make_fc_ok();
    fc.battery_remaining    = 8.0f;  // → EMERGENCY_LAND
    uint64_t now            = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    // EMERGENCY_LAND is highest
    EXPECT_EQ(result.recommended_action, FaultAction::EMERGENCY_LAND);

    // All faults should be flagged
    EXPECT_TRUE(result.active_faults & FAULT_CRITICAL_PROCESS);
    EXPECT_TRUE(result.active_faults & FAULT_THERMAL_CRITICAL);
    EXPECT_TRUE(result.active_faults & FAULT_BATTERY_CRITICAL);
}

// ═══════════════════════════════════════════════════════════
// Battery critical overrides battery low
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, BatteryCritOverridesBatteryLow) {
    FaultManager mgr(default_cfg());
    auto         health  = make_healthy();
    auto         fc      = make_fc_ok();
    fc.battery_remaining = 5.0f;  // well below both thresholds
    uint64_t now         = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    // Only BATTERY_CRITICAL should be set, not BATTERY_LOW
    // (because they are in if/else in evaluate())
    EXPECT_TRUE(result.active_faults & FAULT_BATTERY_CRITICAL);
    EXPECT_FALSE(result.active_faults & FAULT_BATTERY_LOW);
    EXPECT_EQ(result.recommended_action, FaultAction::EMERGENCY_LAND);
}

// ═══════════════════════════════════════════════════════════
// Pose timestamp zero → not treated as stale
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, PoseTimestampZeroIsNotStale) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    auto         fc     = make_fc_ok();
    uint64_t     now    = 1'000 * S;

    // pose_timestamp_ns = 0 → skip staleness check
    auto result = mgr.evaluate(health, fc, 0, now);

    EXPECT_EQ(result.recommended_action, FaultAction::NONE);
    EXPECT_FALSE(result.active_faults & FAULT_POSE_STALE);
}

// ═══════════════════════════════════════════════════════════
// FC connected but low battery_remaining = 0 → no false alarm
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, BatteryRemainingZeroNoFalseAlarm) {
    FaultManager mgr(default_cfg());
    auto         health  = make_healthy();
    auto         fc      = make_fc_ok();
    fc.battery_remaining = 0.0f;  // not yet read from FC
    uint64_t now         = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    // battery_remaining == 0 should be ignored (guard: > 0.0f)
    EXPECT_FALSE(result.active_faults & FAULT_BATTERY_LOW);
    EXPECT_FALSE(result.active_faults & FAULT_BATTERY_CRITICAL);
}

// ═══════════════════════════════════════════════════════════
// fault_action_name coverage
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, FaultActionNames) {
    EXPECT_STREQ(fault_action_name(FaultAction::NONE), "NONE");
    EXPECT_STREQ(fault_action_name(FaultAction::WARN), "WARN");
    EXPECT_STREQ(fault_action_name(FaultAction::LOITER), "LOITER");
    EXPECT_STREQ(fault_action_name(FaultAction::RTL), "RTL");
    EXPECT_STREQ(fault_action_name(FaultAction::EMERGENCY_LAND), "EMERGENCY_LAND");
}

// ═══════════════════════════════════════════════════════════
// Untracked process name → assumed alive, no fault
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, UntrackedProcessAssumedAlive) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    // No processes registered — perception not tracked
    auto     fc  = make_fc_ok();
    uint64_t now = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_FALSE(result.active_faults & FAULT_PERCEPTION_DEAD);
}

// ═══════════════════════════════════════════════════════════
// FC disconnected but timestamp_ns == 0 → no FC-link-lost fault
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, FCDisconnectedNoTimestampNoFault) {
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    auto         fc     = make_fc_ok();
    fc.connected        = false;
    fc.timestamp_ns     = 0;  // never received
    uint64_t now        = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_FALSE(result.active_faults & FAULT_FC_LINK_LOST);
}

// ═══════════════════════════════════════════════════════════
// FaultConfig defaults
// ═══════════════════════════════════════════════════════════
TEST(FaultManagerTest, DefaultConfigValues) {
    FaultConfig cfg;
    EXPECT_EQ(cfg.pose_stale_timeout_ns, 500'000'000ULL);
    EXPECT_FLOAT_EQ(cfg.battery_warn_percent, 30.0f);
    EXPECT_FLOAT_EQ(cfg.battery_rtl_percent, 20.0f);
    EXPECT_FLOAT_EQ(cfg.battery_crit_percent, 10.0f);
    EXPECT_EQ(cfg.fc_link_lost_timeout_ns, 3'000'000'000ULL);
    EXPECT_EQ(cfg.fc_link_rtl_timeout_ns, 15'000'000'000ULL);
    EXPECT_EQ(cfg.loiter_escalation_timeout_ns, 30'000'000'000ULL);
}

// ═══════════════════════════════════════════════════════════
// Phase 3: Three-tier battery escalation
// ═══════════════════════════════════════════════════════════

TEST(FaultManagerTest, BatteryWarnTierTriggersWarnOnly) {
    // Battery between warn (30%) and rtl (20%) → WARN, not RTL
    FaultManager mgr(default_cfg());
    auto         health  = make_healthy();
    auto         fc      = make_fc_ok();
    fc.battery_remaining = 25.0f;  // between 30% warn and 20% rtl
    uint64_t now         = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::WARN);
    EXPECT_TRUE(result.active_faults & FAULT_BATTERY_LOW);
    EXPECT_FALSE(result.active_faults & FAULT_BATTERY_CRITICAL);
}

TEST(FaultManagerTest, BatteryAboveWarnNoFault) {
    // Battery above warn (30%) → no battery fault
    FaultManager mgr(default_cfg());
    auto         health  = make_healthy();
    auto         fc      = make_fc_ok();
    fc.battery_remaining = 50.0f;
    uint64_t now         = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::NONE);
    EXPECT_FALSE(result.active_faults & FAULT_BATTERY_LOW);
    EXPECT_FALSE(result.active_faults & FAULT_BATTERY_CRITICAL);
}

// ═══════════════════════════════════════════════════════════
// Phase 3: FC link-loss RTL contingency
// ═══════════════════════════════════════════════════════════

TEST(FaultManagerTest, FCLinkLostLoiterThenRTL) {
    // FC disconnected for 4s → LOITER (above 3s)
    // FC disconnected for 16s → RTL (above 15s contingency)
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    auto         fc     = make_fc_ok();
    fc.connected        = false;
    fc.timestamp_ns     = 1'000 * S;                // last seen at t=1000s
    uint64_t now_loiter = fc.timestamp_ns + 4 * S;  // 4s after disconnect

    auto r1 = mgr.evaluate(health, fc, now_loiter - 10 * MS, now_loiter);
    EXPECT_EQ(r1.recommended_action, FaultAction::LOITER);
    EXPECT_TRUE(r1.active_faults & FAULT_FC_LINK_LOST);

    // Now 16s after disconnect → should escalate to RTL
    uint64_t now_rtl = fc.timestamp_ns + 16 * S;
    auto     r2      = mgr.evaluate(health, fc, now_rtl - 10 * MS, now_rtl);
    EXPECT_EQ(r2.recommended_action, FaultAction::RTL);
    EXPECT_TRUE(r2.active_faults & FAULT_FC_LINK_LOST);
}

TEST(FaultManagerTest, FCLinkLostBelowRTLTimeoutLoiters) {
    // FC disconnected for 10s → should still LOITER (below 15s contingency)
    FaultManager mgr(default_cfg());
    auto         health = make_healthy();
    auto         fc     = make_fc_ok();
    fc.connected        = false;
    fc.timestamp_ns     = 1'000 * S;
    uint64_t now        = fc.timestamp_ns + 10 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);
    EXPECT_EQ(result.recommended_action, FaultAction::LOITER);
}

// ═══════════════════════════════════════════════════════════
// Phase 3: Geofence breach
// ═══════════════════════════════════════════════════════════

TEST(FaultManagerTest, GeofenceBreachTriggersRTL) {
    FaultManager mgr(default_cfg());
    mgr.set_geofence_violation(true);

    auto     health = make_healthy();
    auto     fc     = make_fc_ok();
    uint64_t now    = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::RTL);
    EXPECT_TRUE(result.active_faults & FAULT_GEOFENCE_BREACH);
}

TEST(FaultManagerTest, GeofenceNoViolationNoFault) {
    FaultManager mgr(default_cfg());
    mgr.set_geofence_violation(false);

    auto     health = make_healthy();
    auto     fc     = make_fc_ok();
    uint64_t now    = 1'000 * S;

    auto result = mgr.evaluate(health, fc, now - 10 * MS, now);

    EXPECT_EQ(result.recommended_action, FaultAction::NONE);
    EXPECT_FALSE(result.active_faults & FAULT_GEOFENCE_BREACH);
}

TEST(FaultManagerTest, GeofenceViolationClearedAfterReset) {
    FaultManager mgr(default_cfg());
    mgr.set_geofence_violation(true);

    auto     health = make_healthy();
    auto     fc     = make_fc_ok();
    uint64_t now    = 1'000 * S;

    auto r1 = mgr.evaluate(health, fc, now - 10 * MS, now);
    EXPECT_TRUE(r1.active_faults & FAULT_GEOFENCE_BREACH);

    // reset() alone should clear the geofence flag — no explicit
    // set_geofence_violation(false) needed.
    mgr.reset();

    auto r2 = mgr.evaluate(health, fc, now - 10 * MS, now + 1 * S);
    EXPECT_FALSE(r2.active_faults & FAULT_GEOFENCE_BREACH);
}
