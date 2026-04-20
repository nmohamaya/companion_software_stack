# ADR-012: Object Detector Licensing Posture — YOLO vs RT-DETR

| Field | Value |
|-------|-------|
| **Status** | Accepted |
| **Date** | 2026-04-20 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Supersedes** | — |
| **Related** | Perception Architecture (docs/design/perception_architecture.md), Issue #443 (custom class training), Issue #448 (altitude-based YOLO dataset), Issue #455 (OpenCvYoloDetector) |

---

## 1. Context

### The licensing problem

The current object detection stack uses **YOLOv8** via the Ultralytics library and its pretrained weights, integrated through `OpenCvYoloDetector` (Issue #455). Since 2024, Ultralytics has re-licensed YOLOv5/v8/v11 under **AGPL-3.0**.

AGPL-3.0 is a strong copyleft license. In the context of a shipped product, it imposes two obligations that matter to us:

1. **Source disclosure.** Any derivative work that is "conveyed" (distributed, or made network-accessible) must be published under AGPL-3.0 with complete corresponding source, including modifications.
2. **Network use trigger.** Unlike GPL, AGPL treats "users interacting with the software over a network" as recipients. A drone that exposes a telemetry server, GCS link, or any remote API could be interpreted as triggering this.

For a stack destined for commercial drone products — sold to customers, deployed on fleets, or licensed to OEMs — AGPL is incompatible with most commercial models. Customers who receive the drone or its software would have the right to demand the full source of everything linked to YOLO. Ultralytics sells a commercial license to escape AGPL, but it's per-seat and expensive.

### Alternatives considered

**RT-DETR** (Baidu, from PaddleDetection) is a real-time transformer-based detector, released under **Apache-2.0**. It's the closest competitor to YOLO in accuracy and speed, with unambiguous commercial licensing.

Other candidates — YOLOv3 (BSD-like, outdated), YOLOv7 (GPL, same problem), Faster R-CNN (too slow), DETR (not real-time), EfficientDet (Apache but slower than both) — either have the same licensing issue or unacceptable performance.

### Why this decision matters now (but not urgently)

The perception architecture rewrite (see `docs/design/perception_architecture.md`) introduces an `IInferenceBackend` HAL that abstracts the model. **If we design that interface correctly, swapping detectors is additive work, not a rewrite.** That's the key insight that allows us to defer the actual migration.

But the interface design is happening now, and we need a default — which means committing to a preferred detector for the perception v1 milestone and a clear path for the commercial-release migration.

---

## 2. Technical comparison (licensing aside)

**Honest assessment: if licensing didn't matter, YOLO wins for our specific stack.** Reasons:

| Factor | YOLO | RT-DETR | Winner for us |
|---|---|---|---|
| Hardware — Pascal (dev desktop) | ~45 FPS (R18 equiv) | ~35 FPS | YOLO |
| Hardware — Jetson Orin (edge) | Mature TRT INT8 recipes | Works but less battle-tested | YOLO |
| Hardware — A10G (cloud) | Strong | Slightly stronger | RT-DETR |
| Fine-tuning ease | `ultralytics` CLI — half-day | PaddleDetection/Transformers — 1-2 weeks | YOLO (large margin) |
| Ecosystem depth | Massive — thousands of tutorials | Smaller, growing | YOLO |
| Pretrained weights variety | n/s/m/l/x | R18/R50/R101 | YOLO (more granular) |
| Accuracy COCO mAP50 | 50.2 (YOLOv8m) | 49.2 (R18) / 53.1 (R50) | Variant-dependent tie |
| VisDrone pretrained | Many variants | Fewer | YOLO (more choice) |
| Segmentation variant | YOLOv8-seg built-in | None (need SAM) | YOLO (but SAM covers us architecturally) |
| Small-object detection | Good | Better — self-attention helps | RT-DETR |
| Crowded scenes | Needs NMS tuning | No NMS, set prediction | RT-DETR |
| Hiring new engineers | Trivial | Harder | YOLO |

### Nuance: the drone-spotting-drone use case

If detecting other drones (counter-UAS, swarm awareness, air-traffic) becomes a **dominant** scenario rather than a secondary one, RT-DETR's small-object advantage becomes meaningfully valuable. Drones in the frame are typically 20-40 pixels at realistic engagement ranges, against sky or cluttered backgrounds, and transformer self-attention handles this better than YOLO's grid-based head.

**However**, even in a drone-heavy use case, the detector is only ~#5 on the priority list:

1. Radar + micro-Doppler (PATH C in our architecture) — drones are great radar targets, terrible camera targets
2. Input resolution (640 → 1280+) — bigger win than detector choice
3. Fine-tuning on Anti-UAV / DUT-Anti-UAV datasets — both detectors benefit equally
4. Tile-based inference for tiny distant objects
5. Detector choice (YOLO vs RT-DETR) — the smallest of the levers

In counter-UAS literature, radar + RF do ~90% of the detection; camera is confirmation. Our architecture already reflects this with PATH C as a primary drone-detection path. **The detector decision matters less than we might intuit**, even for drone-vs-drone.

### Net technical conclusion

- **For obstacle avoidance (people, vehicles, static structures) — YOLO wins clearly** on operational ease.
- **For drone-vs-drone specifically — RT-DETR has a real but small technical advantage.** Radar-Doppler matters more.
- **Licensing is the dominant factor** in the decision, not technical merit.

---

## 3. Decision

**Stay on YOLO for perception v1 development. Plan the migration to RT-DETR as a gate before commercial distribution, not before.**

Specifically:

1. **v1 perception development (now → first internal flight trials):** YOLO continues to be the primary detector. Current `OpenCvYoloDetector` stays. Custom class training (#443) uses YOLO. VisDrone pretrained weights are YOLOv8 variants.

2. **`IInferenceBackend` HAL is detector-agnostic from day one.** No YOLO-specific assumptions in the interface. Preprocessing, postprocessing, and output tensor shapes are abstracted. Swapping to RT-DETR later is purely additive.

3. **RT-DETR backend implemented in parallel, not as a replacement.** `OpenCvRtDetrDetector` class is built alongside YOLO — both selectable via `perception.detector.backend` config. RT-DETR is validated but not the default.

4. **Migration trigger — commercial release.** Switch default to RT-DETR at one of these gates, whichever comes first:
   - First commercial customer pilot
   - First external partner integration or OEM deal
   - First public binary release
   - 12 months from this ADR (forces a review even if no commercial path materialises)

5. **Pre-migration checklist** (what needs to be true before we flip the default):
   - `OpenCvRtDetrDetector` implemented and unit-tested
   - RT-DETR-VisDrone fine-tuned variant validated in our Gazebo + Cosys scenarios
   - Anti-UAV fine-tune recipe documented (if drone-detection becomes a priority)
   - Scenarios #21, #29 retuned with RT-DETR
   - CI validates both paths

6. **Custom class training (#443) short-term:** Continues on YOLO. When we migrate to RT-DETR, we retrain on RT-DETR. This is a known sunk cost and accepted.

7. **Mitigation — CI isolation.** Mark the YOLO-dependent code paths clearly so that:
   - A future commercial release build can be produced without YOLO entirely
   - No proprietary logic creeps into YOLO-adjacent code (keeping it "stub + thin wrapper")

---

## 4. Consequences

### Positive

- **Development velocity preserved.** Small team, stretched across architecture rewrite, infrastructure, and flight prep — dropping YOLO's ecosystem advantage now would cost 2-3 weeks we don't have.
- **`ultralytics` CLI stays usable** for the custom class work (#443, #439, #448).
- **Battle-tested Jetson Orin deployment path** stays active. When we flip to RT-DETR, we do it with evidence of its Orin performance, not blind faith.
- **`IInferenceBackend` discipline is preserved** — the interface stays clean because we validate it with two implementations rather than ad-hoc growth around YOLO's peculiarities.
- **Clear commercial trigger.** No ambiguity about "when do we switch." The gate is visible and planned.

### Negative

- **Sunk cost on custom training.** When we migrate to RT-DETR, any custom fine-tuned YOLO weights (#443) get retrained, not reused. Estimate: ~2-4 weeks of retraining effort at migration time.
- **Compliance debt accumulates.** Every week on YOLO is a week where AGPL obligations are technically active. For an R&D stack that isn't distributed, this is fine — but if an accidental distribution happens (e.g., publishing a demo binary, giving an early partner a preview), we're obligated.
- **Two codepaths to maintain briefly.** During the pre-migration window, both `OpenCvYoloDetector` and `OpenCvRtDetrDetector` exist. Tests, CI, docs cover both. Estimate: 10% maintenance overhead.
- **Risk of "just one more feature" delay.** It's easy to let the migration slip if it's always the lower priority. The 12-month forcing review is the mitigation.

### Neutral

- **No change to IPC wire format** — `DetectedObjects` is model-agnostic either way.
- **No change to downstream (tracker, fusion, planner)** — they consume the same message shape regardless of detector.

---

## 5. Implementation plan

Tied to the perception rewrite phasing (see `perception_architecture.md`).

### Now → perception v1

- **`IInferenceBackend` interface design** — done detector-agnostic. No YOLO leakage into the HAL.
- **`OpenCvYoloDetector` refactored** behind the new HAL but kept as-is functionally. COCO + VisDrone weights both supported via config.
- **Custom class training (#443)** — proceeds on YOLO.

### Perception v1 ships → commercial gate approaches

- **Implement `OpenCvRtDetrDetector`** — mirror the YOLO class structure.
- **`models/download_rtdetr.sh`** — ONNX export pipeline (Hugging Face `transformers` → ONNX with onnxsim).
- **Add RT-DETR to scenarios #21 and #29** as a secondary variant — validate functionality, benchmark accuracy and latency.
- **Document RT-DETR fine-tuning recipe** in `docs/guides/PERCEPTION_TRAINING.md`.

### At the commercial gate

- **Flip default:** `MODEL_PRESET=orin/cloud/dev` → `backend = "rtdetr"`.
- **Retrain custom classes** on RT-DETR.
- **Gate YOLO** behind compile flag: `option(ACKNOWLEDGE_AGPL_YOLO "Enable AGPL-licensed YOLO detector" OFF)`.
- **CI split:** default job has `ACKNOWLEDGE_AGPL_YOLO=OFF`, a separate dev-mode job has it ON to keep the YOLO path validated.
- **Retune scenarios** with RT-DETR as primary.

---

## 6. Alternatives rejected

### (a) Switch to RT-DETR now

Costs 2-3 weeks of migration + retraining + ecosystem-learning during a period where we're already committing to a major architecture rewrite. The AGPL exposure is theoretical until distribution; we can afford to wait.

### (b) Pay for Ultralytics commercial license

~$500-$1,500 per engineer per year. Doesn't solve the customer-side problem — downstream licensees still need their own license. Also doesn't future-proof: Ultralytics could raise prices or change terms.

### (c) Stay on YOLOv3 (BSD-licensed)

Accuracy regression (~8-10 mAP points below v8). Anchor-based, requires tuning. Community has moved on; maintenance burden grows. Rejected.

### (d) Custom in-house detector

6-12 months of specialised ML engineering to match current state-of-the-art. Rejected as unjustifiable cost for a capability that's available free.

### (e) Defer the entire decision

Every week of YOLO-specific code in the HAL raises the switching cost. The way this decision is structured — YOLO-agnostic interface, validated RT-DETR alongside — bounds the deferral risk. We defer the *migration*, not the *architectural preparation*.

---

## 7. Open questions

1. **Segmentation backbone.** If we adopt SAM for class-agnostic masks (PATH A), is the SAM license (Apache-2.0 for SAM-2) compatible with our intended distribution model? Separate ADR.
2. **Pre-trained weights provenance.** Both YOLO and RT-DETR released weights are trained on COCO/VisDrone/etc. Are those training data terms compatible with commercial use downstream? Needs legal review — applies equally to both detectors.
3. **Drone-detection priority.** Is drone-vs-drone detection a dominant scenario for our target customers? If yes, RT-DETR's small-object advantage becomes more valuable and the migration gate moves earlier. Revisit after first customer conversations.
4. **TensorRT parity on Orin.** Is RT-DETR INT8 calibration as stable as YOLO's on Jetson? Benchmark during first Orin hardware trials.

---

## 8. Revisit triggers

- **Ultralytics relicenses** YOLO under a permissive license → stay on YOLO, drop the migration plan.
- **Commercial customer conversation** starts → accelerate RT-DETR migration to align with contract timing.
- **Drone-vs-drone becomes a dominant use case** → re-evaluate; RT-DETR technical case strengthens enough to migrate earlier.
- **Custom YOLO fine-tuning effort exceeds 4 weeks** on #443 without results → reconsider whether to burn that effort on a detector we'll replace.
- **12 months from ADR date (2027-04-20)** — forced review even if no other trigger fires.
