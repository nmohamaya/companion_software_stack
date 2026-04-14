# ADR-009: Tier 1 / Tier 2 Simulation Architecture

| Field | Value |
|-------|-------|
| **Status** | Accepted — implemented |
| **Date** | 2026-03-14 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Supersedes** | — |
| **Extended by** | ADR-011 (Cosys-AirSim Tier 3 photorealistic simulation) |
| **Related** | Issue #167, ADR-006 (HAL strategy), Improvement #46, scenarios 01–09 |

---

## 1. Context

The drone companion software stack requires simulation testing at two levels
with fundamentally different purposes:

1. **Software integration** — Validate that the 7 processes, 21 threads,
   Zenoh IPC channels, FSM transitions, fault handling, and config system
   work correctly together. This must run on any Linux dev machine with no
   external dependencies (no Gazebo, no PX4, no GPU).

2. **Physics-in-the-loop** — Validate that the planning, obstacle avoidance,
   waypoint acceptance, and sensor processing work correctly with realistic
   physics, rendered camera images, and a real PX4 autopilot (SITL).

Before this decision was formalised, the codebase had evolved organically
toward a two-tier model but without explicit design documentation. Several
bugs arose from misunderstanding which tier validated what:

- **Simulated VIO was non-responsive:** `SimulatedVIOBackend` traced a fixed
  circular trajectory ignoring trajectory commands, so no Tier 1 scenario
  could validate waypoint navigation or mission completion logic.
- **Thermal thresholds caused false failures:** Dev machine CPU temperatures
  (99°C) exceeded default thresholds (95°C), triggering `FAULT_THERMAL_CRITICAL`
  → RTL before missions could complete.
- **Collection window was too short:** The scenario runner used a hardcoded
  5-second post-injection window, insufficient for mission completion after
  the VIO was made responsive.

---

## 2. Decision

### Two-Tier Simulation Model

| | Tier 1 — Software Integration | Tier 2 — Physics SITL |
|---|---|---|
| **Runner** | `tests/run_scenario.sh` | `tests/run_scenario_gazebo.sh` |
| **Base config** | `config/default.json` | `config/gazebo_sitl.json` |
| **Requires** | Linux, no extra deps | Gazebo Harmonic, PX4, GPU |
| **CI** | Every PR | Nightly / manual |
| **HAL backends** | Simulated (all 7 processes) | Gazebo cameras, PX4 (MAVSDK), simulated gimbal |
| **VIO source** | `SimulatedVIOBackend` (target-following) | `GazeboVIOBackend` (ground-truth odometry) |
| **FC link** | `SimulatedFCLink` | `MAVSDKFCLink` (real PX4 SITL) |
| **Physics** | First-order dynamics (no inertia/drag) | Full Gazebo physics engine |
| **Purpose** | IPC, FSM, fault handling, config, process lifecycle | Waypoint accuracy, obstacle avoidance, sensor realism |

### Tier 1: Responsive Simulated VIO

The `SimulatedVIOBackend` steers toward the latest trajectory target set via
`set_trajectory_target()` using first-order dynamics:

```
direction = target - current_position
step = min(sim_speed_mps * dt, distance)
current_position += normalize(direction) * step
```

P4 (mission_planner) publishes `TrajectoryCmd` with target_x/y/z (the current
waypoint). P3 forwards these target positions to the VIO backend. The simulated
drone flies in a straight line toward each waypoint at configurable speed.

**Design choice — target-following vs velocity integration:** We evaluated
velocity integration (using the planner's velocity_x/y/z commands) but rejected
it because the simulated detector produces random phantom detections that the
obstacle avoider responds to with repulsive forces. Without a real flight
controller's position hold loop, these forces cause cumulative position drift
and geofence breaches. Target-following provides inherent stability: even if
the trajectory is momentarily perturbed, the VIO steers back toward the
waypoint on the next frame. Obstacle avoidance with camera detections is
validated in Tier 2 (Gazebo) where rendered frames produce meaningful
detections.

Key properties:
- **No target set:** Hovers at origin (0, 0, 0). Does not fly a phantom path.
- **Target following:** Moves in a straight line toward the waypoint at
  configurable speed (`slam.vio.sim_speed_mps`, default 3.0 m/s).
- **No overshoot:** Step is clamped to remaining distance.
- **Thread safety:** Targets are set via `std::atomic<float>` with
  `memory_order_release/acquire` (P3 main loop sets, VIO thread reads).
- **Noise:** ±0.01m Gaussian noise on position output (simulates VIO drift).

This means Tier 1 scenarios can validate:
- Waypoint acceptance (`waypoint_reached()` checks Euclidean distance)
- Full FSM transitions (IDLE → PREFLIGHT → TAKEOFF → EXECUTING → COMPLETE)
- Mission completion signalling ("Mission complete" in logs)
- RTL logic (fault injection → state transitions → return-to-launch)
- Geofence breach detection (VIO position crosses geofence boundary)

What Tier 1 does **not** validate:
- Realistic trajectory dynamics (acceleration, drag, wind)
- Obstacle avoidance with rendered camera detections
- Actual PX4 autopilot behaviour
- Real camera/sensor processing pipelines

### Tier 2: Full Gazebo SITL

Tier 2 scenarios use `config/gazebo_sitl.json` which configures:
- Gazebo camera backends (rendered RGB frames at 30 Hz)
- PX4 autopilot via MAVSDK (real MAVLink commands)
- Gazebo ground-truth odometry for VIO
- Full planning stack (D* Lite + 3D obstacle avoidance + HD-map)

Tier 2 validates everything Tier 1 does not: physics realism, sensor
processing, PX4 interaction, and navigation accuracy.

### Thermal Threshold Strategy

| Config file | `temp_warn_c` | `temp_crit_c` | Rationale |
|-------------|---------------|---------------|-----------|
| `default.json` | 105°C | 120°C | Dev-safe — above any desktop CPU temperature |
| `gazebo_sitl.json` | 105°C | 120°C | Same — Gazebo + PX4 loads CPU heavily |
| `hardware.json` | 75°C | 90°C | **Must be calibrated per-platform** |

**Important:** The `hardware.json` thresholds are placeholder values. For real
deployment, these must be calculated based on the host SBC's thermal runaway
characteristics:

- **Jetson Orin:** Thermal throttle starts at 97°C. Set `temp_crit_c` with
  sufficient margin (e.g., 85°C) to allow RTL to complete before throttling.
- **Thermal trend prediction (planned):** Replace threshold-only detection with
  sliding-window linear regression (dT/dt) to predict time-to-thermal-runaway.
  This enables early warnings well before the critical temperature is reached,
  giving the drone time to RTL safely. See ROADMAP.md Phase 12.
- **Platform-specific heatsink/fan curves** affect how quickly temperature
  rises under load. These should be characterised during hardware integration
  testing and used to set appropriate warning margins.

### Scenario Runner Collection Window

The scenario runner uses a dynamic collection window for Phase 4 (post-fault-
injection monitoring):

```
remaining = scenario_timeout - elapsed - 5s (cleanup buffer)
collection_time = max(remaining, 5s)
```

This ensures missions have the full timeout budget to complete, rather than
being killed after a hardcoded 5-second window.

---

## 3. Alternatives Considered

### A. Single-tier simulation (Gazebo only)

**Rejected.** Gazebo + PX4 requires GPU, significant dependencies, and ~30s
startup time. This makes CI prohibitively slow and prevents testing on
resource-constrained environments. Many software integration bugs (IPC wiring,
FSM logic, config parsing, fault handling) don't need physics simulation.

### B. Non-responsive simulated VIO (fixed trajectory)

**Rejected.** A fixed circular trajectory meant no Tier 1 scenario could
validate waypoint acceptance, mission completion, or RTL logic. This created
a blind spot where Tier 1 tests passed trivially without exercising the core
mission execution path. Bugs in FSM transitions, waypoint acceptance, and
fault response would only be caught in Tier 2 (Gazebo) runs.

### C. Threshold-only thermal monitoring (current)

**Accepted as interim.** Pure threshold comparison is simple and predictable.
However, it can't distinguish between a stable 80°C and a rapidly rising
75°C → 85°C → 95°C trend. The planned predictive thermal trend monitoring
(ROADMAP Phase 12) will address this limitation.

---

## 4. Consequences

### Positive

- **Fast CI:** Tier 1 runs in ~90s per scenario on any Linux machine,
  enabling per-PR validation of all software integration paths.
- **Full mission validation:** Tier 1 now validates the complete
  IDLE→PREFLIGHT→TAKEOFF→EXECUTING→COMPLETE FSM path.
- **No false thermal failures:** Raised dev thresholds eliminate
  `FAULT_THERMAL_CRITICAL` on hot development machines.
- **Clear separation:** Each tier has a documented purpose, making it
  obvious which scenarios to add/modify for a given change.

### Negative

- **Tier 1 fidelity gap:** First-order dynamics don't model acceleration,
  drag, or wind. A waypoint that's reachable in Tier 1 might not be
  reachable with real physics (tight turns, high winds).
- **Dual maintenance:** Some scenarios (e.g., obstacle avoidance) may need
  both a Tier 1 and Tier 2 variant with different pass criteria.
- **Dev thermal thresholds mask real issues:** A process that runs hot
  (approaching 100°C on a dev machine) won't trigger a warning in Tier 1,
  potentially hiding a resource leak that would cause thermal issues on
  real hardware.

### Risks

- **VIO drift accumulation:** The simulated VIO adds ±0.01m noise per frame.
  Over very long missions, this could drift significantly. Current scenarios
  are short enough (4 waypoints, <60s flight time) that this is not an issue.
- **Thermal threshold divergence:** `default.json`, `gazebo_sitl.json`, and
  `hardware.json` have different thermal thresholds. These must be kept in
  sync with their intended use cases.

---

## 5. References

- `process3_slam_vio_nav/include/slam/ivio_backend.h` — `SimulatedVIOBackend`
- `process7_system_monitor/src/main.cpp` — thermal zone computation
- `tests/run_scenario.sh` — Tier 1 scenario runner
- `tests/run_scenario_gazebo.sh` — Tier 2 scenario runner
- `config/default.json` — Tier 1 base config
- `config/gazebo_sitl.json` — Tier 2 base config
- `config/hardware.json` — Real hardware deployment config
- `docs/SIMULATION_ARCHITECTURE.md` — Full simulation design document
- Bug #32 (thermal override nesting), Bug #33 (collection window)
