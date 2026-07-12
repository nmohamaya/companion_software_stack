# contracts/ — the governed wire contract

Every cross-process boundary in this stack is a typed message on a named topic. This directory is the **governance layer** over that contract: the message set, the compatibility policy, and a machine-readable manifest that CI enforces against the compiled code.

**Division of authority** (see [ADR-017](../docs/adr/ADR-017-staged-wire-contract-governance.md)):

- [`common/ipc/include/ipc/ipc_types.h`](../common/ipc/include/ipc/ipc_types.h) **defines** the structs and topic names (the code SSOT).
- [`topics.json`](topics.json) **mirrors** topic ↔ struct ↔ expected `sizeof` ↔ wire version — and [`tests/test_contracts_manifest.cpp`](../tests/test_contracts_manifest.cpp) fails the build if code and manifest drift.
- [`VERSIONING.md`](VERSIONING.md) states the compatibility policy every wire change must follow.

A wire-format change is therefore a `contracts/` change **in the same commit** — the drift gate makes forgetting mechanical to catch, not archaeological.

## Message set

| Topic | Struct | Purpose |
|---|---|---|
| `/drone_mission_cam` | `VideoFrame` | Mission-camera frames (P1 → P2) |
| `/drone_stereo_cam` | `StereoFrame` | Stereo pairs (P1 → P3) |
| `/detected_objects` | `DetectedObjectList` | Tracked detections (P2 → P4) |
| `/semantic_voxels` | `SemanticVoxelBatch` | Semantic voxel batches (PATH A perception) |
| `/slam_pose` | `Pose` | Fused vehicle pose (P3 → P4/P5/P6) |
| `/mission_status` | `MissionStatus` | Mission FSM status telemetry |
| `/trajectory_cmd` | `TrajectoryCmd` | Trajectory setpoints (P4 → P5) |
| `/payload_commands` | `PayloadCommand` | Gimbal/camera commands (P4 → P6) |
| `/fc_commands` | `FCCommand` | Flight-controller command requests (P4 → P5) — **safety-relevant** |
| `/fc_state` | `FCState` | Flight-controller state (P5 → P4/P7) — **safety-relevant** |
| `/gcs_commands` | `GCSCommand` | GCS operator commands (P5 → P4) — **safety-relevant** |
| `/mission_upload` | `MissionUpload` | Waypoint mission upload (→ P4) |
| `/payload_status` | `PayloadStatus` | Payload status (P6 → P4/P7) |
| `/system_health` | `SystemHealth` | Aggregated process health (P7 → P4) |
| `/fault_overrides` | `FaultOverrides` | Fault-injection / override control — **safety-relevant** |
| `/radar_detections` | `RadarDetectionList` | Radar detection lists (perception). **Frame contract (#816):** `azimuth_rad`/`elevation_rad` are BODY-frame FLU — the producing HAL compensates the sensor mount extrinsics before publishing; consumers apply only body→world attitude. Executable spec: `tests/test_frame_contracts.cpp`. |
| `/drone_thread_health_*` (×7, one per process) | `ThreadHealth` | Per-process thread heartbeat health (→ P7) |

Struct sizes and the current wire version are **deliberately not restated here** — [`topics.json`](topics.json) is the machine-checked source; `ctest -R ContractsManifest` verifies it.

## What this is not (yet)

Not a schema-first system: nothing here generates code, and there is no polyglot consumer. Those are staged behind explicit triggers — first non-C++ consumer (Phase 2), first forced incompatible bump (Phase 3) — recorded in [ADR-017](../docs/adr/ADR-017-staged-wire-contract-governance.md) and tracked by [#806](https://github.com/nmohamaya/companion_software_stack/issues/806).
