# Integration Branch — Changes Since `main` (`389089a`)

**Range:** `389089a..ab885fe` (86 commits)
**Period:** 2026-04-20 → 2026-05-11
**Generated:** 2026-05-12

This document summarises every PR merged onto `feature/perception-v2-integration` since it diverged from `main`, grouped by theme. Use this when investigating regressions like "scenario 18 works on main but not on integration" — find the theme that touches the affected subsystem and look at its PRs first.

---

## Summary by theme

| Theme | PRs | Net impact area |
|-------|-----|-----------------|
| [1. Benchmark infrastructure](#1-benchmark-infrastructure-prs-590598) | 9 | Off-runtime: GT capture, baseline, dashboard |
| [2. HAL Perception v2 (E1-E5)](#2-hal-perception-v2-e1e5-prs-602604) | 4 | Headers + SAM/Projector backends |
| [3. PATH A end-to-end](#3-path-a-end-to-end-prs-609614) | 3 | New /semantic_voxels channel, P2 publisher, P4 consumer |
| [4. Avoider hardening (close-regime)](#4-avoider-hardening-prs-617620) | 2 | `brake_in_close_regime`, DA V2 calibration |
| [5. Scenario 33 stack (#645 family)](#5-scenario-33-stack-645-family-prs-636674) | ~35 | Most touch perception/avoider/grid/comms |
| [6. PR review-fix waves](#6-pr-review-fix-waves-prs-679695) | 17 | Plumbing + safety + tests across the stack |
| [7. Cosys ground-truth perception](#7-cosys-ground-truth-perception-prs-697704) | 4 | New Cosys backends (P3 + perception) |
| [8. Scenario-runner + safety nets](#8-scenario-runner--safety-nets-prs-707711) | 4 | `tests/run_scenario_gazebo.sh` + gating |
| [9. ARM-gate (today's work)](#9-arm-gate-todays-work-pr-717) | 1 | `tick_preflight`, `armable` field |

---

## Master chronological timeline (all 86 commits)

Use this to bisect a regression — find the day/window where the symptom started, then look at the commits in that range. Each row is a single squash-merge commit landing on `feature/perception-v2-integration`.

Legend: 🆕 new feature · 🛠 bug fix · 📝 docs · 🧹 chore/cleanup · ⚠ removed in PR #712

| # | Date | Commit | Type | PR | Subject | Subsystem |
|---|------|--------|------|----|---------|-----------|
| 1 | 04-20 | `c5f1efd` | 🆕 | [#590](https://github.com/nmohamaya/companion_software_stack/pull/590) | Perception metrics framework — TP/FP/FN, AP, MOTA/MOTP | benchmarks |
| 2 | 04-20 | `ca1bcf9` | 🆕 | [#591](https://github.com/nmohamaya/companion_software_stack/pull/591) | Latency profiler — per-stage percentiles + correlation-tagged traces | profiling |
| 3 | 04-20 | `07872f3` | 🆕 | [#593](https://github.com/nmohamaya/companion_software_stack/pull/593) | Wire LatencyProfiler into P2 + P4 with DR-022 opt-in config gate | P2 + P4 |
| 4 | 04-20 | `92e5f6f` | 🆕 | [#595](https://github.com/nmohamaya/companion_software_stack/pull/595) | Cosys GT emitter — per-frame ground truth for baseline capture | benchmarks |
| 5 | 04-21 | `d461e7e` | 🧹 | [#596](https://github.com/nmohamaya/companion_software_stack/pull/596) | Baseline capture infrastructure + seed `baseline.json` | benchmarks |
| 6 | 04-21 | `fb77e76` | 🆕 | [#597](https://github.com/nmohamaya/companion_software_stack/pull/597) | CI gating — baseline comparison tool + regression detection | CI |
| 7 | 04-21 | `b845224` | 🆕 | [#598](https://github.com/nmohamaya/companion_software_stack/pull/598) | Dashboard renderer — HTML/MD benchmark report | benchmarks |
| 8 | 04-21 | `c33f012` | 🆕 | [#599](https://github.com/nmohamaya/companion_software_stack/pull/599) | Per-class config schema + avoider/tracker integration | avoider, tracker |
| 9 | 04-22 | `95a3667` | 🧹 | — | Merge `origin/main` into integration | merge |
| 10 | 04-22 | `f7da728` | 🧹 | — | Resolve PROGRESS.md conflict | merge |
| 11 | 04-22 | `358a695` | 🆕 | [#602](https://github.com/nmohamaya/companion_software_stack/pull/602) | **E1 HAL Interface Layer — Perception v2 Abstractions** | HAL (new headers) |
| 12 | 04-22 | `8645db4` | 🆕 | [#603](https://github.com/nmohamaya/companion_software_stack/pull/603) | E5.1-E5.5 — SAM backend + MaskClassAssigner + YoloSeg + DetectorSwitcher | perception |
| 13 | 04-22 | `d2d2422` | 📝 | — | Fix #54 — model path validation cwd regression | docs |
| 14 | 04-22 | `c7d4291` | 🆕 | [#604](https://github.com/nmohamaya/companion_software_stack/pull/604) | E5.4 MaskDepthProjector + Scenario 33 non-COCO obstacles | perception |
| 15 | 04-22 | `86bdf1c` | 🛠 | — | Add missing `test_helpers.h` — untracked since #603 | tests |
| 16 | 04-22 | `aa79f40` | 🆕 | [#609](https://github.com/nmohamaya/companion_software_stack/pull/609) | **PATH A IPC wire type — `SemanticVoxelBatch` + `/semantic_voxels` channel** | IPC |
| 17 | 04-22 | `992b888` | 🆕 | [#611](https://github.com/nmohamaya/companion_software_stack/pull/611) | **PATH A end-to-end integration — P2 publisher + P4 consumer + SAM** | P2, P4 |
| 18 | 04-23 | `76fb35e` | 🆕 | [#614](https://github.com/nmohamaya/companion_software_stack/pull/614) | PATH A diagnostic infra + 8 perception fixes + Cosys runner/deploy UX | perception, deploy |
| 19 | 04-23 | `bbe1307` | 🆕 | [#617](https://github.com/nmohamaya/companion_software_stack/pull/617) | **`brake_in_close_regime` — avoider cancels forward motion into obstacles** | avoider |
| 20 | 04-24 | `8ea74f5` | 🆕 | [#620](https://github.com/nmohamaya/companion_software_stack/pull/620) | DA V2 anchored-scale calibration + texture-gated sampling | perception |
| 21 | 04-24 | `8f09ddb` | 🛠 | [#623](https://github.com/nmohamaya/companion_software_stack/pull/623) | Unblock scenario 33 PATH A diagnostic instrumentation | P4 |
| 22 | 04-24 | `a27d557` | 🛠 | [#628](https://github.com/nmohamaya/companion_software_stack/pull/628) | Scenario runners preflight every `model_path` before launch | scenario runner |
| 23 | 04-24 | `e87e4bd` | 🆕 | [#630](https://github.com/nmohamaya/companion_software_stack/pull/630) | Configurable PATH A voxel sample density | perception |
| 24 | 04-24 | `440fa31` | 🛠 | [#631](https://github.com/nmohamaya/companion_software_stack/pull/631) | Wire `use_cuda` through DA V2 with canary-inference fallback | perception |
| 25 | 04-24 | `153a334` | 🆕 | [#634](https://github.com/nmohamaya/companion_software_stack/pull/634) | Consolidate stacked #624 + #626 (FastSAM CUDA) onto integration | perception |
| 26 | 04-27 | `c7d64fb` | 🛠 | [#637](https://github.com/nmohamaya/companion_software_stack/pull/637) | Batch review-comment fixes for PRs #623, #628, #630, #632 | mixed |
| 27 | 04-27 | `dd8ac65` | 🛠 | [#636](https://github.com/nmohamaya/companion_software_stack/pull/636) | PATH A voxels flow through dynamic-TTL bucket before promotion | grid |
| 28 | 04-27 | `e86d81a` | 🆕 | [#639](https://github.com/nmohamaya/companion_software_stack/pull/639) | **Phase 1 — voxel clustering (per-frame instance IDs)** | perception |
| 29 | 04-27 | `7589d04` | 🆕 | [#640](https://github.com/nmohamaya/companion_software_stack/pull/640) | **Phase 2 — cross-frame voxel instance tracker** | perception |
| 30 | 04-30 | `bbcfea3` | 🆕 | [#641](https://github.com/nmohamaya/companion_software_stack/pull/641) | **⚡ Phase 3 — OccupancyGrid3D instance-aware voxel promotion** | grid (P4 hot path) |
| 31 | 04-30 | `b404f26` | 🆕 | [#642](https://github.com/nmohamaya/companion_software_stack/pull/642) | Phase 4 — enable voxel clustering + tracking + instance promotion in scenario 33 | scenario 33 config |
| 32 | 04-30 | `19f3332` | 🛠 ⚠ | [#646](https://github.com/nmohamaya/companion_software_stack/pull/646) | ObstacleAvoider3D post-correction toward-obstacle hard clamp | avoider |
| 33 | 04-30 | `c0b8a45` | 🆕 ⚠ | [#647](https://github.com/nmohamaya/companion_software_stack/pull/647) | Surface PATH A voxel-tracked obstacles to ObstacleAvoider3D | avoider |
| 34 | 04-30 | `38e1ef8` | 🛠 | [#649](https://github.com/nmohamaya/companion_software_stack/pull/649) | UKF radar-only S-matrix recovery via eigenvalue clamp + R-only fallback | UKF |
| 35 | 04-30 | `ff6947e` | 🛠 | [#650](https://github.com/nmohamaya/companion_software_stack/pull/650) | Correct misleading planner-fallback message + add hover-fallback counter | P4 |
| 36 | 04-30 | `2c242aa` | 🛠 | [#651](https://github.com/nmohamaya/companion_software_stack/pull/651) | **⚡ Comms heartbeat-resend trajectory cmd to beat SimpleFlight 60 ms timeout** | P5 comms (40 Hz) |
| 37 | 04-30 | `9dfea6c` | 🧹 | [#653](https://github.com/nmohamaya/companion_software_stack/pull/653) | Remove `max_static_cells` cap in scenario 33 | scenario 33 config |
| 38 | 04-30 | `1228284` | 🧹 | [#654](https://github.com/nmohamaya/companion_software_stack/pull/654) | Tighten scenario-33 voxel `position_clamp` 200m → 30m | scenario 33 config |
| 39 | 04-30 | `a5b39a9` | 🧹 | [#655](https://github.com/nmohamaya/companion_software_stack/pull/655) | Loosen scenario-33 `inflation_radius` 2.5m → 1.5m | scenario 33 config |
| 40 | 04-30 | `2493e41` | 🧹 | [#656](https://github.com/nmohamaya/companion_software_stack/pull/656) | Enable `static_cell_ttl_s=30s` in scenario 33 | scenario 33 config |
| 41 | 04-30 | `760bcc1` | 🛠 ⚠ | [#657](https://github.com/nmohamaya/companion_software_stack/pull/657) | ObstacleAvoider3D AABB-aware distance + repulsion direction | avoider |
| 42 | 04-30 | `ed611a9` | 🛠 | [#658](https://github.com/nmohamaya/companion_software_stack/pull/658) | MaskDepthProjector altitude filter (#1) + voxel viz tool | perception |
| 43 | 04-30 | `023d2a2` | 🛠 | [#659](https://github.com/nmohamaya/companion_software_stack/pull/659) | MaskDepthProjector SAM mask size filter (#2) | perception |
| 44 | 04-30 | `7c9779d` | 🧹 | [#660](https://github.com/nmohamaya/companion_software_stack/pull/660) | Tighten scenario-33 voxel confidence + instance promotion threshold | scenario 33 config |
| 45 | 04-30 | `d796186` | 🛠 | [#661](https://github.com/nmohamaya/companion_software_stack/pull/661) | Allow radar-confirmed promotion to bypass voxel-input pause | grid |
| 46 | 04-30 | `6f69cd6` | 🛠 | [#664](https://github.com/nmohamaya/companion_software_stack/pull/664) | Batch 1 of review fixes for #645 PR stack | mixed |
| 47 | 04-30 | `c069e23` | 🛠 | [#665](https://github.com/nmohamaya/companion_software_stack/pull/665) | Batch 2 — UKF construction guard + Pass 3 transition | UKF |
| 48 | 04-30 | `d532b2e` | 🛠 | [#666](https://github.com/nmohamaya/companion_software_stack/pull/666) | Batch 3 — reset trajectory sentinel on ARM | P5 comms |
| 49 | 04-30 | `7960623` | 🛠 | [#667](https://github.com/nmohamaya/companion_software_stack/pull/667) | **🚨 RTL landing-pause overrides radar promotion bypass (P1 SAFETY-CRITICAL)** | P4 |
| 50 | 04-30 | `a3e8853` | 🛠 | [#668](https://github.com/nmohamaya/companion_software_stack/pull/668) | MaskDepthProjector P2 review fixes — bbox/altitude robustness | perception |
| 51 | 04-30 | `539e4dd` | 🛠 | [#669](https://github.com/nmohamaya/companion_software_stack/pull/669) | Clamp `voxel_input.min_confidence` to [0, 1] | grid |
| 52 | 04-30 | `2cc1a16` | 🛠 | [#670](https://github.com/nmohamaya/companion_software_stack/pull/670) | Tracker — double-precision age conversion + NaN guard + clock-domain doc | tracker |
| 53 | 04-30 | `5809faf` | 🛠 | [#671](https://github.com/nmohamaya/companion_software_stack/pull/671) | Atomic pause flags in OccupancyGrid3D for cross-thread safety | grid |
| 54 | 04-30 | `94c7e54` | 🧹 | [#672](https://github.com/nmohamaya/companion_software_stack/pull/672) | Cleanup review follow-ups — script path + scenario gate comment | mixed |
| 55 | 04-30 | `1bf0c18` | 📝 | [#673](https://github.com/nmohamaya/companion_software_stack/pull/673) | Log #645-stack deferred review items in IMPROVEMENTS.md | docs |
| 56 | 04-30 | `d198cd5` | 🛠 | [#674](https://github.com/nmohamaya/companion_software_stack/pull/674) | ZenohSubscriber latency tracker — no accumulate on quiet topics | IPC |
| 57 | 04-30 | `3cfaff0` | 📝 | [#675](https://github.com/nmohamaya/companion_software_stack/pull/675) | Log #643 superseded items in IMPROVEMENTS.md | docs |
| 58 | 04-30 | `c860c65` | 📝 | [#676](https://github.com/nmohamaya/companion_software_stack/pull/676) | Refine #643 supersede entry — drop max_match_distance bump | docs |
| 59 | 04-30 | `e1c073f` | 📝 | [#677](https://github.com/nmohamaya/companion_software_stack/pull/677) | Update pending-reviews tracker | docs |
| 60 | 04-30 | `49812b6` | 📝 | [#678](https://github.com/nmohamaya/companion_software_stack/pull/678) | Forward-port #652 keystone-fix follow-ups | docs |
| 61 | 05-01 | `7037ebd` | 🛠 | [#679](https://github.com/nmohamaya/companion_software_stack/pull/679) | #636 Pass 1+2 review findings (P2 batch) | grid |
| 62 | 05-01 | `bc46ed6` | 🛠 | [#680](https://github.com/nmohamaya/companion_software_stack/pull/680) | #631 Pass 1+2 review findings (P2 batch) | perception |
| 63 | 05-01 | `b671b39` | 🛠 | [#681](https://github.com/nmohamaya/companion_software_stack/pull/681) | #633 P1 + P2 review findings (FastSAM + scenario 33) | perception |
| 64 | 05-02 | `36907fc` | 🛠 | [#682](https://github.com/nmohamaya/companion_software_stack/pull/682) | #646 rename misleading test, add lateral-preservation lock | avoider tests |
| 65 | 05-02 | `7c4ded5` | 🛠 | [#683](https://github.com/nmohamaya/companion_software_stack/pull/683) | #649 review findings (UKF S-matrix recovery) | UKF |
| 66 | 05-02 | `6b18f97` | 🛠 | [#684](https://github.com/nmohamaya/companion_software_stack/pull/684) | #651 P1 + P2 review findings (comms heartbeat) | P5 comms |
| 67 | 05-02 | `8c3a2d7` | 🛠 ⚠ | [#685](https://github.com/nmohamaya/companion_software_stack/pull/685) | #657 drone-inside-AABB safety + atomic close-regime + tests | avoider |
| 68 | 05-02 | `7332d1d` | 📝 ⚠ | [#686](https://github.com/nmohamaya/companion_software_stack/pull/686) | DR-041 — drone-inside-AABB defense-in-depth rationale | docs |
| 69 | 05-02 | `e75b485` | 🛠 | [#687](https://github.com/nmohamaya/companion_software_stack/pull/687) | #602 P1 + P2 review findings (HAL Perception v2) | HAL |
| 70 | 05-02 | `5ab7caf` | 🛠 | [#688](https://github.com/nmohamaya/companion_software_stack/pull/688) | #603 document DetectorSwitcher integration gap + sam_thread hardening | perception |
| 71 | 05-02 | `d44ab77` | 🛠 | [#689](https://github.com/nmohamaya/companion_software_stack/pull/689) | #611 P1 + P2 review findings (PATH A end-to-end) | perception, P4 |
| 72 | 05-02 | `4f66be1` | 🛠 | [#690](https://github.com/nmohamaya/companion_software_stack/pull/690) | #634 cross-backend consistency — DA V2 net_ rebuild + FastSAM abs-path | perception |
| 73 | 05-02 | `8fc01c7` | 📝 | [#691](https://github.com/nmohamaya/companion_software_stack/pull/691) | #674 document relaxed-ordering rationale | docs |
| 74 | 05-02 | `58799b2` | 🛠 | [#692](https://github.com/nmohamaya/companion_software_stack/pull/692) | #685 Copilot — AABB-inside direction magnitude + dead-zone floor | avoider |
| 75 | 05-02 | `f21228e` | 🛠 | [#693](https://github.com/nmohamaya/companion_software_stack/pull/693) | **⚡ #684 Copilot — heartbeat staleness split into two timestamps** | P5 comms |
| 76 | 05-02 | `36ea7f7` | 🛠 | [#694](https://github.com/nmohamaya/companion_software_stack/pull/694) | Copilot sweep — cross-PR review hardening | mixed |
| 77 | 05-02 | `575a7bd` | 🧹 | [#695](https://github.com/nmohamaya/companion_software_stack/pull/695) | scenario-33 `position_clamp_m` 30 → 50 to avoid blind grid past X=30 | scenario 33 config |
| 78 | 05-03 | `f0029f0` | 🛠 | [#697](https://github.com/nmohamaya/companion_software_stack/pull/697) | **SIM-only Cosys ground-truth pose passthrough for P3** | P3 SLAM/VIO |
| 79 | 05-03 | `4ce15e8` | 🛠 | [#699](https://github.com/nmohamaya/companion_software_stack/pull/699) | Reject DA V2 max-clamp saturation samples in PATH A projector | perception |
| 80 | 05-03 | `78ce45d` | 📝 | [#700](https://github.com/nmohamaya/companion_software_stack/pull/700) | ADR-013 — stereo + radar defence-in-depth | docs |
| 81 | 05-05 | `54693e6` | 📝 | [#662](https://github.com/nmohamaya/companion_software_stack/pull/662) | Roadmap Phase 14 — stereo-first autonomy | docs |
| 82 | 05-05 | `eff1718` | 🆕 | [#704](https://github.com/nmohamaya/companion_software_stack/pull/704) | **⚡⚡ Scenario 33 PASS — ground-truth perception backends + HD-map + diagnostics** | perception, P4 |
| 83 | 05-07 | `f42aee6` | 🛠 | [#709](https://github.com/nmohamaya/companion_software_stack/pull/709) | Close two cleanup gaps in Gazebo scenario runner | scenario runner |
| 84 | 05-08 | `1e9872d` | 🆕 | [#707](https://github.com/nmohamaya/companion_software_stack/pull/707) | Gate scenario-33 avoider safety nets behind config flags (Phase 1) | avoider config |
| 85 | 05-11 | `ee7478f` | 🛠 | [#711](https://github.com/nmohamaya/companion_software_stack/pull/711) | **⚡ Empirical cleanup — flip `aabb_aware_distance` default to false** | avoider |
| 86 | 05-11 | `ab885fe` | 🛠 | [#717](https://github.com/nmohamaya/companion_software_stack/pull/717) | **🚨 Gate ARM on FC preflight readiness (`health_all_ok`)** | P4 PREFLIGHT |

**Legend tags inline:**
- ⚡ = touches a hot path (P4 planning_loop, P5 comms tx) — **first place to look for runtime regression**
- 🚨 = safety-critical change
- ⚠ = removed/tombstoned by PR #712 (today's empirical cleanup)
- 🆕 / 🛠 / 📝 / 🧹 = feat / fix / docs / chore

---

## 1. Benchmark infrastructure (PRs #590–#598)

Off-runtime tooling. Should not affect scenario behaviour unless `latency_profiler.enabled=true` in scenario config (scenario 18 has it off).

| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `c5f1efd` | 04-20 | [#590](https://github.com/nmohamaya/companion_software_stack/pull/590) | Perception metrics framework — TP/FP/FN, AP, MOTA/MOTP |
| `ca1bcf9` | 04-20 | [#591](https://github.com/nmohamaya/companion_software_stack/pull/591) | Latency profiler — per-stage percentiles + correlation-tagged traces |
| `07872f3` | 04-20 | [#593](https://github.com/nmohamaya/companion_software_stack/pull/593) | Wire LatencyProfiler into P2 + P4 with DR-022 opt-in config gate |
| `92e5f6f` | 04-20 | [#595](https://github.com/nmohamaya/companion_software_stack/pull/595) | Cosys GT emitter — per-frame ground truth for baseline capture |
| `d461e7e` | 04-21 | [#596](https://github.com/nmohamaya/companion_software_stack/pull/596) | Baseline capture infrastructure + seed `baseline.json` |
| `fb77e76` | 04-21 | [#597](https://github.com/nmohamaya/companion_software_stack/pull/597) | CI gating — baseline comparison tool + regression detection |
| `b845224` | 04-21 | [#598](https://github.com/nmohamaya/companion_software_stack/pull/598) | Dashboard renderer — HTML/MD benchmark report |
| `c33f012` | 04-21 | [#599](https://github.com/nmohamaya/companion_software_stack/pull/599) | Per-class config schema + avoider/tracker integration |

**Regression-watch tags:**
- `07872f3` wires the profiler into P2+P4 hot paths. Even gated, the hooks add a config-load + atomic-check per tick. Worth checking if `latency_profiler.enabled` accidentally defaults `true`.

---

## 2. HAL Perception v2 (E1-E5) (PRs #602–#604)

New abstraction layer for perception backends (SAM, mask classification, depth projection). Purely additive — existing code paths untouched.

| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `358a695` | 04-22 | [#602](https://github.com/nmohamaya/companion_software_stack/pull/602) | E1 HAL Interface Layer — Perception v2 Abstractions (`ievent_camera.h`, `iinference_backend.h`, `isemantic_projector.h`, `ivolumetric_map.h`) |
| `8645db4` | 04-22 | [#603](https://github.com/nmohamaya/companion_software_stack/pull/603) | E5.1-E5.5 — SAM backend + MaskClassAssigner + YoloSeg + DetectorSwitcher |
| `c7d4291` | 04-22 | [#604](https://github.com/nmohamaya/companion_software_stack/pull/604) | E5.4 MaskDepthProjector + Scenario 33 non-COCO obstacles |
| `86bdf1c` | 04-22 | (hotfix) | Add missing `test_helpers.h` — untracked since PR #603 |
| `d2d2422` | 04-22 | (docs) | Fix #54 — model path validation cwd regression |

---

## 3. PATH A end-to-end (PRs #609–#614)

New parallel perception path: SAM masks → CpuSemanticProjector → SemanticVoxelBatch IPC → P4 grid promotion. **Config-gated by `path_a.enabled` (false for scenario 18).**

| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `aa79f40` | 04-22 | [#609](https://github.com/nmohamaya/companion_software_stack/pull/609) | PATH A IPC wire type — `SemanticVoxelBatch` + `/semantic_voxels` channel [PR 1/3] |
| `992b888` | 04-22 | [#611](https://github.com/nmohamaya/companion_software_stack/pull/611) | PATH A end-to-end integration — P2 publisher + P4 consumer + SAM backends |
| `76fb35e` | 04-23 | [#614](https://github.com/nmohamaya/companion_software_stack/pull/614) | PATH A diagnostic infra + 8 perception fixes + Cosys runner/deploy UX |

**Regression-watch tags:**
- `992b888` adds the P4 subscriber for `/semantic_voxels`. Even with `path_a.enabled=false`, the Zenoh subscription is declared. If the subscription has weird liveness behaviour it could affect P4 startup or tick rates.

---

## 4. Avoider hardening (PRs #617–#620)

| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `bbe1307` | 04-23 | [#617](https://github.com/nmohamaya/companion_software_stack/pull/617) | `brake_in_close_regime` — avoider cancels forward motion into obstacles (#513) |
| `8ea74f5` | 04-24 | [#620](https://github.com/nmohamaya/companion_software_stack/pull/620) | DA V2 anchored-scale calibration + texture-gated sampling |

**Regression-watch tags:**
- `bbe1307` runs on every NAVIGATE tick when `close_regime_active_=true`. Adds atomic-load + per-obstacle dot-product computation. Cheap, but on the hot path.

---

## 5. Scenario 33 stack (#645 family) (PRs #636–#674)

The biggest theme — **35+ PRs** chasing scenario 33's TemplateCube collisions. Touches perception, avoider, occupancy grid, comms, scenario config. Most are config tweaks specific to scenario 33; some are infrastructure changes that affect all scenarios.

### Voxel pipeline (Phase 1-4)
| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `dd8ac65` | 04-27 | [#636](https://github.com/nmohamaya/companion_software_stack/pull/636) | PATH A voxels flow through dynamic-TTL bucket before promotion |
| `e86d81a` | 04-27 | [#639](https://github.com/nmohamaya/companion_software_stack/pull/639) | **Phase 1** — voxel clustering (per-frame instance IDs) |
| `7589d04` | 04-27 | [#640](https://github.com/nmohamaya/companion_software_stack/pull/640) | **Phase 2** — cross-frame voxel instance tracker |
| `bbcfea3` | 04-30 | [#641](https://github.com/nmohamaya/companion_software_stack/pull/641) | **Phase 3** — OccupancyGrid3D instance-aware voxel promotion |
| `b404f26` | 04-30 | [#642](https://github.com/nmohamaya/companion_software_stack/pull/642) | **Phase 4** — enable voxel clustering + tracking + instance promotion **in scenario 33** |

**Regression-watch tags:**
- `bbcfea3` changes the grid promotion path. Even when PATH A is off, the grid's `update_obstacles()` may hit new code branches. **Top suspect for any `update_obstacles` slowdown.**

### Avoider & grid behaviour
| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `19f3332` | 04-30 | [#646](https://github.com/nmohamaya/companion_software_stack/pull/646) | ObstacleAvoider3D post-correction toward-obstacle hard clamp ⚠ *removed by #712* |
| `c0b8a45` | 04-30 | [#647](https://github.com/nmohamaya/companion_software_stack/pull/647) | Surface PATH A voxel-tracked obstacles to ObstacleAvoider3D ⚠ *removed by #712* |
| `760bcc1` | 04-30 | [#657](https://github.com/nmohamaya/companion_software_stack/pull/657) | ObstacleAvoider3D AABB-aware distance + repulsion direction ⚠ *removed by #712* |
| `8c3a2d7` | 05-02 | [#685](https://github.com/nmohamaya/companion_software_stack/pull/685) | Drone-inside-AABB safety + atomic close-regime + tests ⚠ *partially removed by #712* |

### Fault / safety paths
| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `38e1ef8` | 04-30 | [#649](https://github.com/nmohamaya/companion_software_stack/pull/649) | UKF radar-only S-matrix recovery via eigenvalue clamp + R-only fallback |
| `ff6947e` | 04-30 | [#650](https://github.com/nmohamaya/companion_software_stack/pull/650) | Correct misleading planner-fallback message + add hover-fallback counter |
| `2c242aa` | 04-30 | [#651](https://github.com/nmohamaya/companion_software_stack/pull/651) | **Comms heartbeat-resend trajectory cmd to beat SimpleFlight 60 ms timeout** |
| `7960623` | 04-30 | [#667](https://github.com/nmohamaya/companion_software_stack/pull/667) | RTL landing-pause overrides radar promotion bypass (P1 SAFETY-CRITICAL) |
| `d796186` | 04-30 | [#661](https://github.com/nmohamaya/companion_software_stack/pull/661) | Allow radar-confirmed promotion to bypass voxel-input pause |

**Regression-watch tags:**
- `2c242aa` (#651) — adds **40 Hz trajectory resend** on the comms thread. Designed for Cosys SimpleFlight but runs against any `IFCLink`. On Gazebo+MAVLink, this means 2.5× more outbound MAVLink traffic than before. If PX4 EKF2 is sensitive to MAVLink load, this could affect preflight convergence.

### Scenario 33 config tuning (mostly config-file changes, scenario 33-specific)
| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `9dfea6c` | 04-30 | [#653](https://github.com/nmohamaya/companion_software_stack/pull/653) | Remove `max_static_cells` cap in scenario 33 (was 500, now unlimited) |
| `1228284` | 04-30 | [#654](https://github.com/nmohamaya/companion_software_stack/pull/654) | Tighten scenario-33 voxel `position_clamp` 200m → 30m |
| `a5b39a9` | 04-30 | [#655](https://github.com/nmohamaya/companion_software_stack/pull/655) | Loosen scenario-33 `inflation_radius` 2.5m → 1.5m to reopen WP2→WP3 corridor |
| `2493e41` | 04-30 | [#656](https://github.com/nmohamaya/companion_software_stack/pull/656) | Enable `static_cell_ttl_s=30s` in scenario 33 to decay outbound wake |
| `7c9779d` | 04-30 | [#660](https://github.com/nmohamaya/companion_software_stack/pull/660) | Tighten scenario-33 voxel confidence + instance promotion threshold |
| `575a7bd` | 05-02 | [#695](https://github.com/nmohamaya/companion_software_stack/pull/695) | scenario-33 `position_clamp_m` 30 → 50 to avoid blind grid past X=30 |

### MaskDepthProjector hardening
| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `ed611a9` | 04-30 | [#658](https://github.com/nmohamaya/companion_software_stack/pull/658) | MaskDepthProjector altitude filter (#1) + voxel viz tool |
| `023d2a2` | 04-30 | [#659](https://github.com/nmohamaya/companion_software_stack/pull/659) | MaskDepthProjector SAM mask size filter (#2) |
| `a3e8853` | 04-30 | [#668](https://github.com/nmohamaya/companion_software_stack/pull/668) | MaskDepthProjector P2 review fixes — bbox/altitude robustness |

### Review-fix batches & docs
| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `6f69cd6` | 04-30 | [#664](https://github.com/nmohamaya/companion_software_stack/pull/664) | Batch 1 of review fixes for #645 PR stack |
| `c069e23` | 04-30 | [#665](https://github.com/nmohamaya/companion_software_stack/pull/665) | Batch 2 — UKF construction guard + Pass 3 transition |
| `d532b2e` | 04-30 | [#666](https://github.com/nmohamaya/companion_software_stack/pull/666) | Batch 3 — reset trajectory sentinel on ARM (#651 P2) |
| `539e4dd` | 04-30 | [#669](https://github.com/nmohamaya/companion_software_stack/pull/669) | Clamp `voxel_input.min_confidence` to [0, 1] |
| `2cc1a16` | 04-30 | [#670](https://github.com/nmohamaya/companion_software_stack/pull/670) | Tracker — double-precision age conversion + NaN guard + clock-domain doc |
| `5809faf` | 04-30 | [#671](https://github.com/nmohamaya/companion_software_stack/pull/671) | Atomic pause flags in OccupancyGrid3D for cross-thread safety |
| `94c7e54` | 04-30 | [#672](https://github.com/nmohamaya/companion_software_stack/pull/672) | Cleanup review follow-ups — script path + scenario gate comment |
| `1bf0c18` | 04-30 | [#673](https://github.com/nmohamaya/companion_software_stack/pull/673) | Log #645-stack deferred review items in IMPROVEMENTS.md |
| `d198cd5` | 04-30 | [#674](https://github.com/nmohamaya/companion_software_stack/pull/674) | ZenohSubscriber latency tracker no longer accumulates samples on quiet topics |
| `3cfaff0` / `c860c65` / `e1c073f` / `49812b6` | 04-30 | (docs) | Tracker / docs / forward-port follow-ups |

### Scenario-runner / preflight tooling
| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `a27d557` | 04-24 | [#628](https://github.com/nmohamaya/companion_software_stack/pull/628) | Scenario runners preflight every `model_path` before launch |
| `e87e4bd` | 04-24 | [#630](https://github.com/nmohamaya/companion_software_stack/pull/630) | Configurable PATH A voxel sample density |
| `440fa31` | 04-24 | [#631](https://github.com/nmohamaya/companion_software_stack/pull/631) | Wire `use_cuda` through DA V2 with canary-inference fallback |
| `153a334` | 04-24 | [#634](https://github.com/nmohamaya/companion_software_stack/pull/634) | Consolidate stacked #624 + #626 (FastSAM CUDA) onto integration |
| `c7d64fb` | 04-27 | [#637](https://github.com/nmohamaya/companion_software_stack/pull/637) | Batch review-comment fixes for PRs #623, #628, #630, #632 |
| `8f09ddb` | 04-24 | [#623](https://github.com/nmohamaya/companion_software_stack/pull/623) | Unblock scenario 33 PATH A diagnostic instrumentation |

---

## 6. PR review-fix waves (PRs #679–#695)

Mostly defensive hardening from the 10-agent two-pass review pipeline. Spread across the whole stack.

| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `7037ebd` | 05-01 | [#679](https://github.com/nmohamaya/companion_software_stack/pull/679) | #636 Pass 1+2 review findings (P2 batch) |
| `bc46ed6` | 05-01 | [#680](https://github.com/nmohamaya/companion_software_stack/pull/680) | #631 Pass 1+2 review findings (P2 batch) |
| `b671b39` | 05-01 | [#681](https://github.com/nmohamaya/companion_software_stack/pull/681) | #633 P1 + P2 review findings (FastSAM + scenario 33) |
| `36907fc` | 05-02 | [#682](https://github.com/nmohamaya/companion_software_stack/pull/682) | #646 rename misleading test, add lateral-preservation lock |
| `7c4ded5` | 05-02 | [#683](https://github.com/nmohamaya/companion_software_stack/pull/683) | #649 review findings (UKF S-matrix recovery) |
| `6b18f97` | 05-02 | [#684](https://github.com/nmohamaya/companion_software_stack/pull/684) | #651 P1 + P2 review findings (comms heartbeat) |
| `7332d1d` | 05-02 | [#686](https://github.com/nmohamaya/companion_software_stack/pull/686) | DR-041 — drone-inside-AABB defense-in-depth rationale ⚠ *tombstoned in #712* |
| `e75b485` | 05-02 | [#687](https://github.com/nmohamaya/companion_software_stack/pull/687) | #602 P1 + P2 review findings (HAL Perception v2) |
| `5ab7caf` | 05-02 | [#688](https://github.com/nmohamaya/companion_software_stack/pull/688) | #603 document DetectorSwitcher integration gap + sam_thread hardening |
| `d44ab77` | 05-02 | [#689](https://github.com/nmohamaya/companion_software_stack/pull/689) | #611 P1 + P2 review findings (PATH A end-to-end) |
| `4f66be1` | 05-02 | [#690](https://github.com/nmohamaya/companion_software_stack/pull/690) | #634 cross-backend consistency — DA V2 net_ rebuild + FastSAM abs-path + zero-divisor guards |
| `8fc01c7` | 05-02 | [#691](https://github.com/nmohamaya/companion_software_stack/pull/691) | #674 document relaxed-ordering rationale + correct IMPROVEMENTS.md |
| `58799b2` | 05-02 | [#692](https://github.com/nmohamaya/companion_software_stack/pull/692) | #685 Copilot — AABB-inside direction magnitude + dead-zone floor + boundary contact |
| `f21228e` | 05-02 | [#693](https://github.com/nmohamaya/companion_software_stack/pull/693) | **#684 Copilot — heartbeat staleness measured wrong thing — split into two timestamps** |
| `36ea7f7` | 05-02 | [#694](https://github.com/nmohamaya/companion_software_stack/pull/694) | Copilot sweep — cross-PR review hardening |

**Regression-watch tags:**
- `f21228e` (#693) changes how comms thread measures trajectory staleness. Splits one timestamp into two (`last_send` for heartbeat-period gate, `last_new_trajectory` for stale-bound). If either is mis-initialised, comms could falsely suppress heartbeats.

---

## 7. Cosys ground-truth perception (PRs #697–#704)

Adds Cosys-AirSim ground-truth perception backends (camera, depth, segmentation, name resolver) for scenario 33 testing. **Config-gated** — scenario 18 doesn't activate any of this.

| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `f0029f0` | 05-03 | [#697](https://github.com/nmohamaya/companion_software_stack/pull/697) | SIM-only Cosys ground-truth pose passthrough for P3 |
| `4ce15e8` | 05-03 | [#699](https://github.com/nmohamaya/companion_software_stack/pull/699) | Reject DA V2 max-clamp saturation samples in PATH A projector |
| `78ce45d` | 05-03 | [#700](https://github.com/nmohamaya/companion_software_stack/pull/700) | ADR-013 — stereo + radar defence-in-depth (not unified UKF fusion) |
| `54693e6` | 05-05 | [#662](https://github.com/nmohamaya/companion_software_stack/pull/662) | Roadmap Phase 14 — stereo-first autonomy + multi-sensor fusion |
| `eff1718` | 05-05 | [#704](https://github.com/nmohamaya/companion_software_stack/pull/704) | **Scenario 33 PASS — ground-truth perception backends + HD-map + diagnostics** |

**Regression-watch tags:**
- `eff1718` (#704) is the biggest single PR in the range. Large diff. Includes a behavioural change in the planner: **cached path validation against current grid before reusing** (the change behind the #714 stale test). Could affect SURVEY → NAVIGATE handoff timing.
- `f0029f0` (#697) — touches P3 SLAM/VIO startup. Even though Cosys-gated, init-order changes could leak.

---

## 8. Scenario-runner + safety nets (PRs #707–#711)

| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `f42aee6` | 05-07 | [#709](https://github.com/nmohamaya/companion_software_stack/pull/709) | Close two cleanup gaps in Gazebo scenario runner (#708) |
| `1e9872d` | 05-08 | [#707](https://github.com/nmohamaya/companion_software_stack/pull/707) | Gate scenario-33 avoider safety nets behind config flags (Phase 1) (#706) |
| `ee7478f` | 05-11 | [#711](https://github.com/nmohamaya/companion_software_stack/pull/711) | **Empirical cleanup — flip `aabb_aware_distance` default to false (Gazebo regression resolution) (#710)** |

**Regression-watch tags:**
- `ee7478f` (#711) removes three avoider safety-net code paths after empirical sweep validated they were dead weight on scenario 33. **The removed code paths might be load-bearing on scenario 18.** This is a strong candidate for the "works on main, fails on integration" pattern.

---

## 9. ARM-gate (today's work) (PR #717)

| Commit | Date | PR | What it did |
|--------|------|----|-------------|
| `ab885fe` | 05-11 | [#717](https://github.com/nmohamaya/companion_software_stack/pull/717) | **Gate ARM on FC preflight readiness (`health_all_ok`) (#716)** |

Empirically validated: 5/5 cold-start scenario 18 runs yesterday with 0 PX4 "Arming denied". Today's first run after reboot had a 17-second PX4-init wait (visible as drone idle) followed by clean ARM. Today's 2nd run PASSed 19/19 cleanly with full survey yaw.

**Regression-watch tags:**
- Doesn't change SURVEY or NAVIGATE — only PREFLIGHT.
- Adds 1 MAVSDK subscription (`subscribe_health_all_ok`) on MavlinkFCLink boot. Health subscription is read-only; shouldn't add MAVLink load to PX4.

---

## Suspect ranking for "works on main, fails on integration"

If the issue is **SURVEY-phase planning_loop hang**, the strongest suspects in order are:

1. **`bbcfea3` (#641) Phase 3 OccupancyGrid3D instance-aware voxel promotion** — touches grid promotion path. Even with PATH A off, `update_obstacles()` may hit new code branches.
2. **`2c242aa` (#651) Comms heartbeat-resend** — 40 Hz outbound MAVLink traffic. If PX4's MAVLink processing falls behind, EKF2 sensor fusion timing could suffer.
3. **`eff1718` (#704) Scenario 33 PASS** — large planner-touching PR. The cached-path-validation logic could change NAVIGATE timing.
4. **`ee7478f` (#711) Empirical cleanup** — removed code that might be load-bearing on scenario 18.
5. **`f21228e` (#693) Heartbeat staleness split** — comms-side bookkeeping; could mis-fire under edge cases.
6. **`b404f26` (#642) Phase 4 voxel clustering enable for scenario 33** — config-gated to scenario 33 only, but worth ruling out.

If the issue is **17-second rotor-off wait** specifically, that's the ARM-gate (`ab885fe` / #717) doing its job — PX4 needs that time on cold-cache runs regardless of which branch is checked out.

## Suggested next investigation step

Cheapest sanity check: rebuild this worktree on **`ee7478f`** (the commit immediately before today's PR #717 merge) and run scenario 18. If it has the same SURVEY hang on this machine → the regression is **before** today's work, somewhere in the 80 pre-#717 commits. If clean → the regression is in #717 itself (unlikely given yesterday's 5/5 PASS at the same code content).

After that, follow the suspect ranking above and bisect more narrowly.

---

## Appendix A — Which PRs **actually run** in scenario 18?

Scenario 18 runtime config (verified from merged JSON):

- `detector.backend = color_contour`
- `tracker.backend = bytetrack`
- `fusion.backend = ukf` (with `radar_enabled = true`)
- `path_a.enabled = false` (default — scenario 18 does not override)
- `depth_estimator.enabled = false`
- `voxel_input.enabled = false` (default — scenario 18 does not override)
- `path_planner.backend = dstar_lite`
- `obstacle_avoider.backend = potential_field_3d`
- `fc_link.backend = mavlink` (MAVSDK to PX4 SITL)
- All `*.backend = gazebo` for camera / IMU / depth / radar / SLAM

Given these flags, here is the subset of the 86 commits whose runtime code paths execute in scenario 18.

### Critical hot-path (touches comms / planner tick / avoider unconditionally) ⚡

| PR | Commit | What it does | Why it runs in scenario 18 |
|----|--------|--------------|-----------------------------|
| **#651** | `2c242aa` | **Comms heartbeat-resend trajectory cmd at 40 Hz** | `fc_tx_thread` runs against any `IFCLink`; no config flag. Once first ARM happens, this resends the cached trajectory every 25 ms. **2.5× more MAVLink to PX4 than pre-#651.** |
| **#666** | `d532b2e` | Reset trajectory sentinel on ARM | Same `fc_tx_thread`. Fires on every ARM. |
| **#684** | `6b18f97` | #651 review fixes — comms heartbeat | Same thread. |
| **#693** | `f21228e` | Heartbeat staleness split into two timestamps | Same `fc_tx_thread`. Replaces single `last_traj_send` with `last_send` + `last_new_traj`. |
| **#717** | `ab885fe` | **ARM-gate on `health_all_ok`** | New code in `tick_preflight` runs at every PREFLIGHT tick. |
| **#674** | `d198cd5` | ZenohSubscriber latency tracker no-accumulate on quiet topics | Affects every Zenoh subscriber. Scenario 18 has 6+ subs in P4 alone. |

### Avoider (`potential_field_3d` runs in scenario 18) 🛡

| PR | Commit | What it does | Why it runs |
|----|--------|--------------|-------------|
| **#599** | `c33f012` | Per-class config schema + avoider integration | Loads `per_class.*` keys at avoider init. Always loaded; takes effect on every per-class lookup at NAVIGATE tick. |
| **#617** | `bbe1307` | **`brake_in_close_regime`** | Default `true`. Cancels forward velocity component when in close regime. |
| **#685** ⚠ | `8c3a2d7` | Drone-inside-AABB safety + atomic close-regime + tests | Partially removed by PR #712. Atomic `close_regime_active_` may still be in the code path |
| **#692** | `58799b2` | #685 Copilot — AABB-inside direction + dead-zone floor | Builds on #685; partly removed by #712 |
| **#707** | `1e9872d` | Gate scenario-33 avoider safety nets behind config flags | New flags default off → scenario 18 behaviour unchanged from pre-PR |
| **#711** | `ee7478f` | **Empirical cleanup — flip `aabb_aware_distance` default to false** | Removed three avoider safety nets that may have been load-bearing on scenario 18 |

### OccupancyGrid3D / D*Lite planner (P4 hot path) 📦

| PR | Commit | What it does | Why it runs |
|----|--------|--------------|-------------|
| **#641** | `bbcfea3` | Phase 3 OccupancyGrid3D instance-aware promotion | Added new methods; `update_from_objects()` (scenario 18's path) NOT modified. Class layout / vtable changed → may affect icache |
| **#650** | `ff6947e` | Correct misleading planner-fallback message + hover-fallback counter | Adds atomic counter incremented on every hover-fallback event |
| **#671** | `5809faf` | Atomic pause flags in OccupancyGrid3D | Adds two atomic bools to OccupancyGrid3D. Loaded with `memory_order_acquire` on every `update_from_objects` call |
| **#704** | `eff1718` | Scenario 33 PASS — **includes Issue #698 cached-path validation** | New code in `grid_planner_base.h`: on D*Lite search failure, walks cached path checking each cell. **Runs on every search failure in scenario 18.** |
| **#661** | `d796186` | Radar-confirmed promotion bypass voxel-input pause | Scenario 18 has radar enabled. Runs when radar confirms an obstacle. |
| **#667** | `7960623` | RTL landing-pause overrides radar promotion bypass (SAFETY-CRITICAL) | Builds on #661. Runs at RTL state transition. |

### UKF fusion (`fusion.backend=ukf` in scenario 18) 📡

| PR | Commit | What it does | Why it runs |
|----|--------|--------------|-------------|
| **#649** | `38e1ef8` | **UKF radar-only S-matrix recovery via eigenvalue clamp + R-only fallback** | Modifies the UKF measurement-update path. Scenario 18 has `radar_enabled=true` → fires on every radar-only update |
| **#665** | `c069e23` | UKF construction guard + Pass 3 transition | Validation in UKF constructor. Runs once at perception startup |
| **#683** | `7c4ded5` | #649 review findings | Same UKF path |

### Tracker (`bytetrack` in scenario 18) 🎯

| PR | Commit | What it does | Why it runs |
|----|--------|--------------|-------------|
| **#670** | `2cc1a16` | Tracker double-precision age conversion + NaN guard + clock-domain doc | Modifies tracker age math. Runs in every fusion cycle |

### Mission planner main loop (always-on) 🧠

| PR | Commit | What it does | Why it runs |
|----|--------|--------------|-------------|
| **#593** | `07872f3` | Wire LatencyProfiler into P2 + P4 with DR-022 opt-in config gate | **Opt-in defaulted off**, but the hook code still executes (atomic-bool check) on every profiled call |

### Off-runtime / not in scenario 18's path 📋

These 60+ PRs are **fully gated off** in scenario 18:

- **#590-#598** Benchmark infrastructure (off-runtime CI / dashboard)
- **#602-#604** HAL Perception v2 + SAM/MaskClassAssigner/MaskDepthProjector — `path_a.enabled` gated
- **#609, #611, #614** PATH A IPC + end-to-end — `path_a.enabled` gated
- **#620, #631** DA V2 — `depth_estimator.enabled` gated
- **#628, #637, #709** Scenario-runner / preflight (build / launcher only)
- **#630, #634, #636, #639, #640, #642, #658, #659** PATH A / voxel pipeline — gated
- **#653-#656, #660, #695** scenario-33 config-only changes
- **#646, #647, #657, #685(partial)** ⚠ Removed by PR #712 yesterday
- **#662, #700** ADR / roadmap docs

---

## Appendix B — Top 5 suspects for "works on main, fails on integration"

Only these unconditionally execute in scenario 18:

1. **#651 / #693 / #666 / #684 — Comms heartbeat-resend stack.** 40 Hz unconditional MAVLink resend. Could affect PX4 EKF2 if MAVLink RX queue is sensitive to outbound load.
2. **#717 — ARM-gate.** Adds 17 s visible wait when PX4 is slow. By design.
3. **#704 — Scenario 33 PASS.** Includes cached-path validation that runs on every D*Lite search failure on scenario 18 too.
4. **#599 — Per-class avoider config.** Always loaded; could change avoider behaviour if config keys default differently from main expected.
5. **#711 — Empirical cleanup.** Removed avoider code paths that *might* have been load-bearing on scenario 18 (only validated against scenario 33).

**Bisect order suggestion:** start with #651 (heartbeat-resend). Test scenario 18 at `2c242aa^` — if it PASSes cleanly, the heartbeat-resend is the cause of the integration-vs-main divergence.

---

*This file is generated, not maintained. Run `git log 389089a..HEAD --oneline` for the canonical list.*
