# Aerial Autonomy Stack — Safety-Engineering Plan (v1)

How this companion-computer stack instills functional-safety discipline **by construction**, and exactly what a future certification campaign would formalise rather than re-architect. This is the named safety argument over decisions the repo already made — the ADR catalogue ([docs/adr/](adr/)), the CI gates, the watchdog architecture, the command-authority hardening — lifted into explicit hazards, claims, and evidence pointers. Tracked by #807; the wire-contract governance companion is #806.

**Scope.** This stack is the *companion computer*: perception, navigation, mission autonomy, and links. The flight controller (PX4 or equivalent) owns inner-loop control and its own arming checks — an independent, external safety layer. This plan covers the stack's contribution to system safety: **what it commands, what it trusts, and how it degrades.** It is a reference/development stack, not a certified system; the posture is *cert-readiness by construction* (see §4).

## 1. Governing rules

> **Move fast on evidence timing, never on safety properties.** The deferral test is binary: if deferring an item could let an unsafe design ship, it is NOW. Only things producible later *without changing the design* defer (quantified failure rates, tool-qualification dossiers, independent assessment, the full V-model document set).
>
> **Fix the root cause, not the symptom.** A patch that silences a failing test/check or hides a symptom can mask an unsafe design — a removed guard, a silenced race, an unhandled fault. Every failure follows symptom → root cause → fix → prevention; [tracking/BUG_FIXES.md](tracking/BUG_FIXES.md) (every bug with root cause and how-found) and [tracking/CI_ISSUES.md](tracking/CI_ISSUES.md) institutionalise the discipline, and every fixed bug gets a regression test before the fix.

## 2. What never defers (the floor)

Safety **properties**, built in now because retrofitting any of them is a re-architecture:

1. **Hazard awareness** — the hazard log (§5) exists and grows with every incident class found in test or simulation.
2. **Safe-state design** — responses are graded by reversibility: warn → replan → LOITER → RTL → LAND. On uncertainty, escalate to the *least-irreversible* mode; a LOITER that turns out unnecessary is cheap, a premature ARM is not.
3. **Command-authority discipline at the FC boundary** — any FSM transition that emits a physical FC command (ARM, TAKEOFF, LAND, RTL) requires **multi-second continuously-true gating state**, reset on any flicker; single-tick triggers on `fc_state.*` fields are forbidden. Windows are config-driven, never hardcoded.
4. **Cold-start data hygiene** — first observations from external systems (FC state, VIO pose, any IPC subscriber) are suspect: require multi-observation stability, minimum age, or a publisher-side timestamp proving the data postdates this process's birth.
5. **Isolation and containment** — one process per function, typed IPC as the only coupling (no shared memory outside the bus), and a three-layer watchdog: lock-free thread heartbeats → supervising process manager with dependency-ordered restart → systemd (`Type=notify`, watchdog, binds-to). See [design/hardening-design.md](design/hardening-design.md) and [ADR-004](adr/ADR-004-process-thread-watchdog-architecture.md).
6. **Coding-standard + sanitizer discipline on flight-critical paths** — C++17, `-Werror -Wall -Wextra`, clang-tidy/clang-format gates, the AVOID/PREFER construct rules ([reference/CPP_PATTERNS_GUIDE.md](reference/CPP_PATTERNS_GUIDE.md)), `Result<T,E>` instead of exceptions ([ADR-007](adr/ADR-007-error-handling.md)), and ASan/TSan/UBSan runs as merge-blocking CI legs ([deploy/run_ci_local.sh](../deploy/run_ci_local.sh) mirrors them locally). A race on a fusion or command path is a safety defect, not hygiene.
7. **Testable safety timing** — every debounce, staleness window, and timeout goes through the injectable clock ([iclock.h](../common/util/include/util/iclock.h) + `ScopedMockClock`), so safety timing logic is exercised deterministically in microseconds. Untestable timing is untested safety logic.
8. **Wire-contract integrity** — every IPC struct is `static_assert`-ed trivially-copyable and standard-layout with collocated ABI guards; every message carries a wire-version byte, receivers reject unknown versions, and recorded flights are replayable across format evolution ([wire_format.h](../common/ipc/include/ipc/wire_format.h), [ipc_types.h](../common/ipc/include/ipc/ipc_types.h)). Governance is being tightened in #806.

## 3. NOW items, each mapped to its existing anchor

| # | Item | What it is | Existing anchor |
|---|---|---|---|
| 3.1 | **Control-structure hazard analysis (STPA-lite)** | Unsafe-control-action thinking over the 7-process control structure, focused on the FC-command path: *provided-when-unsafe* (premature ARM), *not-provided* (missed failsafe), *wrong-timing* (stale state), *wrong-duration* (stuck mode). The cold-start epic is this analysis paying out; a formal pass is an open item (§7). | Process map + IPC graph (README), hazard log §5 |
| 3.2 | **Interference containment** | Spatial: typed IPC only, no shared mutable state across processes. Temporal: watchdog layers plus the observability rule — no mutex-protected loggers/profilers on flight-critical threads; buffer lock-free, drain on an IO thread (priority-inversion hazard documented per primitive). | [ADR-001](adr/ADR-001-ipc-framework-selection.md)/[ADR-002](adr/ADR-002-modular-ipc-backend-architecture.md), [DESIGN_RATIONALE.md](tracking/DESIGN_RATIONALE.md) concurrency/observability entries |
| 3.3 | **Degraded-mode ladder** | Reversibility-graded responses; asymmetric preconditions for asymmetric-cost actions — irreversible commands carry strictly stronger gates than recoverable ones. | Command debounce + fault escalation in P4 ([mission_state_tick.h](../process4_mission_planner/include/planner/mission_state_tick.h), [fault_manager.h](../process4_mission_planner/include/planner/fault_manager.h)) |
| 3.4 | **Requirements-based scenario evidence** | Each simulation scenario declares machine-checked pass criteria — acceptance criteria in executable form, run against the full stack (same code paths as the vehicle; simulation never swaps in a simpler planner than the product flies). | [config/scenarios/](../config/scenarios/), [tests/run_scenario.sh](../tests/run_scenario.sh), inventory in [tests/TESTS.md](../tests/TESTS.md) |
| 3.5 | **Command-authority floor at the FC boundary** | The debounce doctrine (§2.3) plus bounded waiting: if a gating condition never confirms, escalate to a named fault instead of waiting silently (preflight timeout → fault escalation, #718/#765). PX4's own arming checks remain an independent downstream layer — defense in depth, never a substitute. | #740 / #776 / #779 epic; regression tests in [tests/](../tests/) |
| 3.6 | **Post-incident evidence** | Flight recorder stamps every record with the wire version; the replay tool validates per record, so old recordings stay analysable across format evolution. | [common/recorder/](../common/recorder/), [design/API.md](design/API.md) |
| 3.7 | **Evidence-by-CI** | The local CI runner mirrors the hosted pipeline gate-for-gate, so "green" means the same thing everywhere; test counts and suites are inventoried in one SSOT. | [deploy/run_ci_local.sh](../deploy/run_ci_local.sh), [tests/TESTS.md](../tests/TESTS.md) |

## 4. Deferred — and why deferring is safe

The *design properties* above are built; what defers is **evidence production**, none of which changes a line of design: quantified failure-rate targets, tool qualification, independent assessment, a full traceability dossier, and conformance mapping to an operational regime (for a fielded product this stack's frame would be SORA / EASA specific category for operations, with IEC 61508-style technique classes and DO-178C-style requirements-based-testing structure for the software argument — the sanitizer/fuzz/scenario gates *implement* recognised technique classes; the standards do not name tools by brand and no conformance is claimed here). Trigger for all of it: productization toward a specific operational approval.

## 5. Starter hazard log

Seeded from **incident classes actually found and fixed** in this repo — each row names the hazard, the mitigation now in place, and where the evidence lives.

| Hazard | Mitigation (built) | Found by / guarded by |
|---|---|---|
| **Premature ARM/TAKEOFF on transient FC state** — estimator settling flickers `armable`/health for single ticks during cold start | Multi-second continuously-true debounce on every FC-command transition, reset on any drop, config-driven windows, mock-clock unit tests | #740 cold-start evidence; fixed in #776; tests in [tests/TESTS.md](../tests/TESTS.md); [BUG_FIXES.md](tracking/BUG_FIXES.md) |
| **Acting on historic IPC samples** — last-value caching replays stale messages to a re-connecting subscriber | Publisher-side timestamps; age guards; data must postdate the consumer's birth | #720/#722; hardened further in #776 |
| **Trusting pre-convergence VIO** — fresh-stamped zero/garbage poses published while the backend is still initialising | Initialisation-state gating + first-observation quarantine before poses feed navigation or mission decisions | #727 layered-defense epic |
| **Unbounded preflight wait** — `armable` never confirms and the FSM waits forever, masking a real fault | Preflight timeout with named fault escalation instead of silent waiting | #718/#765 |
| **Ghost obstacles promoted from ground clutter** — detector + clamped depth promoted non-existent obstacles into the map, corrupting avoidance | Fusion redesign (bearing from camera, range from radar, covariance-driven trust) + promotion hygiene; fixed in fusion logic, not tuned away in config | Epic #237; [ADR-013](adr/ADR-013-stereo-radar-redundancy-vs-fusion.md); [BUG_FIXES.md](tracking/BUG_FIXES.md) |
| **Stuck flight-critical thread silently stalls a control path** | Lock-free heartbeats → thread watchdog → process manager restart along the dependency graph → systemd watchdog | [ADR-004](adr/ADR-004-process-thread-watchdog-architecture.md); [design/hardening-design.md](design/hardening-design.md) |
| **Observability contention on a control path** — a mutex-protected logger/profiler blocks a higher-priority thread (priority inversion) or contaminates the measurement | Lock-free telemetry primitives drained by IO threads; per-primitive documented constraints; exceptions require a recorded rationale | [DESIGN_RATIONALE.md](tracking/DESIGN_RATIONALE.md) |
| **Silent wire-format drift** — a struct change ships without a version bump and peers misinterpret bytes | Layout/copyability `static_assert`s + collocated ABI guards + version byte with reject-unknown; per-topic manifest and CI drift gate arriving via #806 | [wire_format.h](../common/ipc/include/ipc/wire_format.h); [BUG_FIXES.md](tracking/BUG_FIXES.md) |

## 6. The assurance case (sketch)

Root claim: **"The stack shall not command an irreversible physical action on unconfirmed, stale, or first-observation external state, and on uncertainty it degrades to the least-irreversible safe mode."**

- **G1 — Command authority is debounced and bounded.** Evidence: FC-command debounce + preflight fault escalation and their deterministic-clock regression tests ([mission_state_tick.h](../process4_mission_planner/include/planner/mission_state_tick.h), [fault_manager.h](../process4_mission_planner/include/planner/fault_manager.h), [tests/TESTS.md](../tests/TESTS.md)).
- **G2 — Stale or foreign data cannot masquerade as fresh.** Evidence: age/birth-time guards, initialisation gating, wire-version rejection (#720/#722/#727 fixes + tests).
- **G3 — Stuck or failed components are detected and contained.** Evidence: three watchdog layers, restart dependency graph, fault-injection scenarios ([ADR-004](adr/ADR-004-process-thread-watchdog-architecture.md), [config/scenarios/](../config/scenarios/)).
- **G4 — Safety timing is deterministic and tested.** Evidence: injectable clock throughout safety windows; mock-clock unit tests ([iclock.h](../common/util/include/util/iclock.h)).
- **G5 — The wire contract cannot silently drift.** Evidence: static layout guards + version discipline today; manifest + CI gate via #806.

Each goal holds CI-generated evidence *now*; a certification campaign would populate the same leaves with independently assessed evidence — formalising, not re-analysing.

## 7. Open items

- [ ] #806 — contracts-lite: versioning policy, per-topic manifest, CI drift gate (Phase 1), cross-version deserialisation (Phase 3, deferred).
- [ ] SBOM + dependency-policy gate in CI (identify and anomaly-review every runtime dependency on control paths).
- [ ] Formal STPA pass over the 7-process control structure (current hazard log is incident-seeded; a systematic pass finds the interaction accidents not yet met).
- [ ] Written interference-containment argument (spatial/temporal, using the ISO 26262-6 Annex D categories as the temporal checklist).
- [ ] Expand §6 into a maintained GSN artifact once the open items above generate their evidence.

*Doc conventions: quantitative facts (test counts, scenario counts, interface lists) live in their single sources of truth — [tests/TESTS.md](../tests/TESTS.md), [config/scenarios/](../config/scenarios/), [common/hal/include/hal/](../common/hal/include/hal/) — and are deliberately not restated here.*
