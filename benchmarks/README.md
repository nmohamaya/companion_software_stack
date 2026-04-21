# Perception Benchmark Baseline

Reference metrics for the pre-rewrite perception pipeline (Epic #523, Issue #573).
Every perception v2 PR is compared against this baseline to quantify improvement
or detect regression.

## Files

| File | Purpose |
|------|---------|
| `baseline.json` | Current-stack metrics across 5 scenarios |

## Covered Scenarios

| Scenario | Config | Tier | Stack |
|----------|--------|------|-------|
| `obstacle_avoidance` | `02_obstacle_avoidance.json` | 2 (Gazebo) | HD-map + color_contour + ByteTrack + UKF |
| `perception_avoidance` | `18_perception_avoidance.json` | 2 (Gazebo) | No HD-map, runtime perception only |
| `yolov8_detection` | `21_yolov8_detection.json` | 2 (Gazebo) | YOLOv8 + ByteTrack + UKF |
| `cosys_perception` | `29_cosys_perception.json` | 3 (Cosys-AirSim) | YOLO + DepthAnythingV2 + radar |
| `cosys_static` | `30_cosys_static.json` | 3 (Cosys-AirSim) | Static proving ground |

## Metrics per Scenario

Each scenario entry contains:

- **Detection**: TP, FP, FN, micro-precision, micro-recall, mAP (PASCAL VOC 11-point)
- **Per-class**: per-class TP/FP/FN, precision, recall, F1, AP, detection count
- **Tracking**: MOTA, MOTP, ID switches, fragmentations
- **Latency** (optional): per-stage p50/p90/p95/p99 from `LatencyProfiler`

## How to Regenerate

Baselines are captured from live scenario runs. The `BaselineCapture` class
(`tests/benchmark/baseline_capture.h`) accumulates per-frame metrics during a
scenario and writes to `baseline.json`.

### From Cosys-AirSim (Tier 3)

```bash
# 1. Start Cosys-AirSim with the proving-ground scene
# 2. Run scenarios 29 and 30 with GT emitter + baseline capture enabled
bash tests/run_scenario.sh 29 --baseline-capture
bash tests/run_scenario.sh 30 --baseline-capture
```

### From Gazebo (Tier 2)

```bash
# 1. Start Gazebo SITL
# 2. Run scenarios 02, 18, and 21 with baseline capture
bash tests/run_scenario.sh 02 --baseline-capture
bash tests/run_scenario.sh 18 --baseline-capture
bash tests/run_scenario.sh 21 --baseline-capture
```

### Merge Results

After capturing individual scenarios, merge them into the single baseline.
The merge script is planned but not yet implemented — for now, merge manually
by copying per-scenario entries into `baseline.json`:

```bash
# (Planned) Automated merge:
# python3 benchmarks/merge_baselines.py benchmarks/baseline.json
```

## When to Update

Update `baseline.json` when:

1. **Scenario configs change** (detection parameters, waypoints, objects) — the old
   numbers no longer represent a fair comparison
2. **HAL backends change** (new camera resolution, different radar config) — sensor
   characteristics shifted
3. **Ground-truth class map changes** — different classes or thresholds

Do **not** update the baseline when:

- The perception pipeline improves — the whole point is to measure the delta
- A v2 PR introduces a new algorithm — compare against the old baseline
- Tests are flaky — investigate the root cause first

## How to Interpret a Metric Delta

| Metric | Direction | Interpretation |
|--------|-----------|----------------|
| TP | higher is better | More correct detections |
| FP | lower is better | Fewer hallucinations |
| FN | lower is better | Fewer missed objects |
| Precision | higher is better | Detection quality |
| Recall | higher is better | Detection coverage |
| mAP | higher is better | Overall detection accuracy |
| MOTA | higher is better | Tracking accuracy (1.0 = perfect) |
| MOTP | higher is better | Localisation quality (IoU) |
| ID switches | lower is better | Track identity stability |
| Latency p95 | lower is better | Responsiveness |

A PR that improves mAP but increases FP needs scrutiny — false positives cause
the planner to avoid phantom obstacles. Both precision and recall matter for
safe flight.
