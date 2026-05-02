// process2_perception/src/main.cpp
// Process 2 — Perception: inference, tracking, fusion pipeline.
// Reads video frames from SHM, runs detection → tracking → fusion,
// publishes fused objects to SHM.

#include "hal/cpu_semantic_projector.h"
#include "hal/hal_factory.h"
#include "hal/idepth_estimator.h"
#include "hal/iinference_backend.h"
#include "hal/iradar.h"
#include "hal/simulated_sam_backend.h"
#include "ipc/ipc_types.h"
#include "perception/detector_interface.h"
#include "perception/fusion_engine.h"
#include "perception/ifusion_engine.h"
#include "perception/itracker.h"
#include "perception/kalman_tracker.h"
#include "perception/mask_depth_projector.h"
#include "perception/types.h"
#include "perception/ukf_fusion_engine.h"
#include "perception/voxel_clusterer.h"
#include "perception/voxel_instance_tracker.h"
#include "perception/voxel_obstacle_snapshot.h"
#include "util/config_keys.h"
#include "util/diagnostic.h"
#include "util/latency_profiler.h"
#include "util/path_a_trace.h"
#include "util/process_context.h"
#include "util/scoped_timer.h"
#include "util/sd_notify.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"
#include "util/triple_buffer.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <optional>
#include <thread>

#include <Eigen/Geometry>  // Quaternionf for full-quaternion transform (#421)

using namespace drone::perception;

// Only gates the main-thread health-check loop.  Worker threads use
// g_shutdown_phase exclusively — do not read g_running from worker threads.
static std::atomic<bool> g_running{true};

// ── Phased shutdown (Issue #446) ────────────────────────────
// Phase 0 = running normally
// Phase 1 = inference stopped  (detector no longer consuming frames)
// Phase 2 = tracker stopped    (tracker drained inference buffer)
// Phase 3 = fusion/depth/radar stopped (fusion drained tracker buffer)
// Phase 4 = all stopped
//
// Each downstream stage drains its input TripleBuffer before exiting,
// ensuring in-flight data is processed rather than dropped.
static std::atomic<int> g_shutdown_phase{0};

// ── Camera → body extrinsic rotation (PR #614 / Issue #612) ─────────
// OpenCV camera optical frame (X=right, Y=down, Z=forward) →
// aerospace-ENU body frame (X=forward, Y=left, Z=up).  Hoisted to file
// scope so the constant is built once at program start and reviewers
// reading `mask_projection_thread` see the frame convention at the top
// of the file, next to the phased-shutdown globals.  Eigen types aren't
// constexpr-constructible, so a meyers singleton is the idiomatic fit.
static const Eigen::Matrix3f& R_body_from_cam() {
    static const Eigen::Matrix3f kMatrix = (Eigen::Matrix3f() << 0, 0, 1,  //
                                            -1, 0, 0,                      //
                                            0, -1, 0                       //
                                            )
                                               .finished();
    return kMatrix;
}

// ── Inference thread ────────────────────────────────────────
// Stops when shutdown_phase >= 1.  No drain needed — inference is the
// pipeline source; stopping it is what triggers downstream drains.
// @param projection_output_queue nullable — when PATH A is enabled
//                 (Epic #520, Issue #608), the mask_projection_thread reads
//                 from a second TripleBuffer.  inference_thread writes to
//                 it in addition to the primary tracker queue.  Each
//                 TripleBuffer has a single consumer — fanout is explicit.
// @param profiler nullable — pass nullptr to disable per-stage latency
//                 profiling. See DR-022 for the safety analysis of
//                 mutex-protected recording from this flight-critical
//                 thread.
static void inference_thread(drone::ipc::ISubscriber<drone::ipc::VideoFrame>& video_sub,
                             drone::TripleBuffer<Detection2DList>&            output_queue,
                             std::atomic<int>& shutdown_phase, IDetector& detector,
                             drone::util::LatencyProfiler*         profiler,
                             drone::TripleBuffer<Detection2DList>* projection_output_queue) {
    DRONE_LOG_INFO("[Inference] Thread started — using detector: {}{}", detector.name(),
                   projection_output_queue ? " (PATH A fanout active)" : "");

    auto hb = drone::util::ScopedHeartbeat("inference", true);

    uint64_t frame_count = 0;
    while (shutdown_phase.load(std::memory_order_acquire) < 1) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        drone::ipc::VideoFrame frame;
        bool                   got_frame = video_sub.receive(frame);

        if (got_frame) {
            drone::util::FrameDiagnostics diag(frame.sequence_number);

            auto dets = [&]() {
                drone::util::ScopedDiagTimer              timer(diag, "Detect");
                std::optional<drone::util::ScopedLatency> bench_detect;
                // See DR-022 (docs/tracking/DESIGN_RATIONALE.md): mutex-protected
                // profiler on a flight-critical thread is accepted because
                // recorders share priority, mutex hold is <100ns dominated by
                // the detector's ms-scale work, and gated by benchmark.profiler.enabled.
                if (profiler) bench_detect.emplace(*profiler, "Detect");
                return detector.detect(frame.pixel_data, frame.width, frame.height, frame.channels);
            }();

            Detection2DList det_list;
            det_list.detections     = std::move(dets);
            det_list.timestamp_ns   = frame.timestamp_ns;
            det_list.frame_sequence = frame.sequence_number;

            diag.add_metric("Detect", "n_detections",
                            static_cast<double>(det_list.detections.size()));

            // PATH A fanout — mask_projection_thread needs the raw detector
            // bboxes alongside SAM masks.  Write a copy first; move into the
            // primary queue second.
            if (projection_output_queue) {
                projection_output_queue->write(det_list);  // copy
            }
            output_queue.write(std::move(det_list));
            ++frame_count;

            if (diag.has_warnings() || diag.has_errors()) {
                diag.log_summary("Inference");
            } else if (frame_count % 100 == 0) {
                DRONE_LOG_INFO("[Inference] Processed {} frames (writes={})", frame_count,
                               output_queue.write_count());
            }
            // Log IPC latency periodically
            video_sub.log_latency_if_due();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    DRONE_LOG_INFO("[Inference] Thread stopped — {} frames, {} writes", frame_count,
                   output_queue.write_count());
}

// ── SAM thread ──────────────────────────────────────────────
// Stops when shutdown_phase >= 1.  Consumes video frames via its own
// Zenoh subscriber (same pattern as depth_thread) and produces
// class-agnostic masks for the mask_projection_thread.  Only started
// when perception.path_a.enabled is true.
// @param sam   IInferenceBackend that returns per-mask
//              InferenceDetection entries (e.g. SimulatedSAMBackend).
static void sam_thread(drone::ipc::ISubscriber<drone::ipc::VideoFrame>& video_sub,
                       drone::TripleBuffer<Masks2DList>&                output_queue,
                       std::atomic<int>& shutdown_phase, drone::hal::IInferenceBackend& sam) {
    DRONE_LOG_INFO("[SAM] Thread started — backend: {}", sam.name());

    auto hb = drone::util::ScopedHeartbeat("sam", true);

    uint64_t frame_count = 0;
    uint64_t fail_count  = 0;
    while (shutdown_phase.load(std::memory_order_acquire) < 1) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        drone::ipc::VideoFrame frame;
        bool                   got_frame = video_sub.receive(frame);

        if (!got_frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // PR #603 P2 review: validate the wire-format frame at the
        // boundary.  PR #688 Copilot review: VideoFrame's `pixel_data`
        // is a fixed-size inline array (not a pointer) and the struct
        // has no `valid` field — `VideoFrame::validate()` checks that
        // `width × height × channels` fits within the inline buffer
        // and that none of those dims are zero.  Stride is NOT
        // currently validated; if the producer ever sets a stride
        // that points past the buffer end, that would slip through.
        // Tracked as a follow-up to extend `VideoFrame::validate()`.
        if (!frame.validate()) {
            ++fail_count;
            if (fail_count % 100 == 1) {
                DRONE_LOG_WARN("[SAM] frame.validate() failed ({} consecutive)", fail_count);
            }
            // PR #603 P2 review: backpressure on persistent failure so
            // we don't busy-loop pulling+rejecting bad frames at the
            // subscriber's max rate.  100ms gives the upstream camera /
            // P1 process time to recover; brief enough that operators
            // see real frames quickly when the stream restores.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        auto result = sam.infer(frame.pixel_data, frame.width, frame.height, frame.channels);
        if (!result.is_ok()) {
            ++fail_count;
            if (fail_count % 100 == 1) {
                DRONE_LOG_WARN("[SAM] infer() failed ({} consecutive): {}", fail_count,
                               result.error());
            }
            // PR #603 P2 review: backpressure on persistent inference
            // failure (e.g. CUDA driver crash, model file disappeared)
            // — same 100ms backoff as the validation path.  Without
            // this the SAM thread spins at frame-arrival rate, burning
            // CPU on a backend that's known-bad.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        fail_count = 0;

        Masks2DList list;
        list.masks          = std::move(result).value().detections;
        list.timestamp_ns   = frame.timestamp_ns;
        list.frame_sequence = frame.sequence_number;
        output_queue.write(std::move(list));
        ++frame_count;

        if (frame_count % 100 == 0) {
            DRONE_LOG_INFO("[SAM] Processed {} frames (writes={})", frame_count,
                           output_queue.write_count());
        }
    }
    DRONE_LOG_INFO("[SAM] Thread stopped — {} frames, {} writes", frame_count,
                   output_queue.write_count());
}

// ── Mask projection thread (PATH A consumer) ─────────────────
// Stops when shutdown_phase >= 2 (same as fusion_thread — runs alongside
// fusion, not downstream of it).  Correlates SAM masks, detector bboxes,
// depth, and camera pose, runs MaskDepthProjector, converts the
// resulting VoxelUpdate[] to a SemanticVoxelBatch, and publishes on
// `/semantic_voxels` for P4's voxel subscriber.
//
// Synchronization: primary trigger is SAM output (the most expensive
// stage).  On each arriving mask list, pull latest-value from the other
// three inputs.  If detections or depth are missing for this tick,
// skip — the MaskDepthProjector needs both (masks alone can't produce
// voxels without depth).
static void mask_projection_thread(
    drone::TripleBuffer<Masks2DList>&                       masks_queue,
    drone::TripleBuffer<Detection2DList>&                   detections_queue,
    drone::TripleBuffer<drone::hal::DepthMap>&              depth_queue,
    drone::ipc::ISubscriber<drone::ipc::Pose>&              pose_sub,
    drone::ipc::IPublisher<drone::ipc::SemanticVoxelBatch>& voxel_pub,
    std::atomic<int>& shutdown_phase, drone::perception::MaskDepthProjector& projector,
    drone::util::PathATrace* trace,
    // Issue #638 Phase 1 — voxel clustering params.
    // eps_m == 0 disables clustering (every voxel
    // gets instance_id=0, current default).
    float cluster_eps_m, int cluster_min_pts,
    // Issue #638 Phase 2 — cross-frame instance tracker.
    // Disabled when clustering is disabled (eps==0).
    drone::perception::VoxelInstanceTracker* tracker,
    // Issue #645 — snapshot of voxel-derived obstacles
    // surfaced to the fusion thread for inclusion in the
    // DetectedObjectList consumed by ObstacleAvoider3D.
    // nullptr disables the surfacing path.
    drone::TripleBuffer<drone::perception::VoxelObstacleSnapshot>* voxel_obstacle_buf) {
    DRONE_LOG_INFO("[MaskProj] Thread started — publishing to {}",
                   drone::ipc::topics::SEMANTIC_VOXELS);

    auto hb = drone::util::ScopedHeartbeat("mask_projection", true);

    uint64_t                            tick_count      = 0;
    uint64_t                            published_count = 0;
    uint64_t                            skipped_count   = 0;
    std::optional<drone::ipc::Pose>     latest_pose;
    std::optional<drone::hal::DepthMap> latest_depth;
    std::optional<Detection2DList>      latest_dets;
    // Issue #638 Phase 1 — reusable scratch for the per-frame cluster pass.
    // Keeping the maps + parent vector across frames avoids per-frame
    // hashmap reallocation; the clusterer clears them at entry.
    // Pre-reserve to typical worst-case (5000 voxels/frame in scenario 33);
    // avoids reallocations on N-growth ticks (review P1-E from PR #639).
    drone::perception::VoxelClusterScratch cluster_scratch;
    cluster_scratch.parent.reserve(5000);
    cluster_scratch.cell_to_voxel.reserve(5000);
    cluster_scratch.root_size.reserve(512);
    cluster_scratch.root_to_id.reserve(512);

    // Issue #638 P1-D — reusable SemanticVoxelBatch (was make_unique per
    // tick).  ~40 KB struct allocated once at thread start; cleared via
    // {0, 0, 0} reassignment at batch boundary instead of heap alloc.
    drone::ipc::SemanticVoxelBatch batch_buf{};

    while (shutdown_phase.load(std::memory_order_acquire) < 2) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());

        auto masks_opt = masks_queue.read();
        if (!masks_opt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        ++tick_count;

        // Refresh latest detections + depth + pose (best-effort; keep prior
        // snapshot if the triple buffer hasn't produced new data this tick).
        if (auto d = detections_queue.read()) latest_dets = std::move(*d);
        if (auto d = depth_queue.read()) latest_depth = std::move(*d);
        drone::ipc::Pose pose_msg;
        if (pose_sub.receive(pose_msg)) latest_pose = pose_msg;

        // Need all four inputs for projection to produce meaningful voxels.
        // Skip the tick if any are missing — next tick will retry with the
        // cached snapshot.
        if (!latest_dets || !latest_depth || !latest_pose) {
            ++skipped_count;
            if (skipped_count % 100 == 1) {
                DRONE_LOG_DEBUG("[MaskProj] skip — dets={} depth={} pose={}",
                                latest_dets.has_value(), latest_depth.has_value(),
                                latest_pose.has_value());
            }
            continue;
        }

        // Convert perception::Detection2D -> hal::InferenceDetection for the
        // MaskDepthProjector API.  Masks are empty on these (detector doesn't
        // produce masks — SAM does).
        std::vector<drone::hal::InferenceDetection> det_hal;
        det_hal.reserve(latest_dets->detections.size());
        for (const auto& d : latest_dets->detections) {
            drone::hal::InferenceDetection h;
            h.bbox.x      = d.x;
            h.bbox.y      = d.y;
            h.bbox.w      = d.w;
            h.bbox.h      = d.h;
            h.class_id    = static_cast<int>(d.class_id);
            h.confidence  = d.confidence;
            h.mask_width  = 0;
            h.mask_height = 0;
            det_hal.push_back(std::move(h));
        }

        // Build camera pose from the SLAM body pose.  Two stages:
        //   1. T_world_body — SLAM quaternion + translation (drone body frame
        //      in world coordinates).
        //   2. R_body_from_cam — static camera→body extrinsic rotation.  The
        //      projector back-projects in the OpenCV camera convention
        //      (X=right, Y=down, Z=forward).  Without this rotation, the
        //      camera's forward axis (depth) was landing on the body's up
        //      axis (altitude) — every voxel ended up 5-30 m above the drone
        //      instead of in front of it.  Diagnosed via Issue #612 voxel
        //      trace: 10 321 voxels over a full scenario-33 run with zero
        //      within 3 m of the actual cube-collision point.
        Eigen::Quaternionf q(static_cast<float>(latest_pose->quaternion[0]),
                             static_cast<float>(latest_pose->quaternion[1]),
                             static_cast<float>(latest_pose->quaternion[2]),
                             static_cast<float>(latest_pose->quaternion[3]));
        if (q.squaredNorm() < 1e-6f) {
            ++skipped_count;
            continue;
        }
        q.normalize();

        // See file-scope R_body_from_cam() — definition lives at the top of
        // main.cpp next to the phased-shutdown globals.
        Eigen::Affine3f cam_pose = Eigen::Affine3f::Identity();
        cam_pose.linear()        = q.toRotationMatrix() * R_body_from_cam();
        cam_pose.translation()   = Eigen::Vector3f(static_cast<float>(latest_pose->translation[0]),
                                                   static_cast<float>(latest_pose->translation[1]),
                                                   static_cast<float>(latest_pose->translation[2]));

        auto proj = projector.project(masks_opt->masks, det_hal, *latest_depth, cam_pose);
        if (!proj.is_ok()) {
            if (tick_count % 100 == 1) {
                DRONE_LOG_WARN("[MaskProj] project() failed: {}", proj.error());
            }
            continue;
        }
        auto voxels = std::move(proj).value();
        if (voxels.empty()) {
            continue;
        }

        // Issue #638 Phase 1 — assign per-frame cluster IDs in-place.
        // Voxels in clusters of <cluster_min_pts get instance_id=0
        // (downstream Phase 3 will skip promotion for those).  Disabled
        // when cluster_eps_m == 0; today P4 ignores instance_id so the
        // field is published-only until Phase 3 lands.
        drone::perception::assign_instance_ids(voxels, cluster_eps_m, cluster_min_pts,
                                               &cluster_scratch);
        const uint32_t n_clusters = drone::perception::count_clusters(voxels);

        // Issue #638 Phase 2 — convert frame-local cluster IDs to stable
        // cross-frame instance IDs.  Tracker is null when clustering is
        // disabled (eps==0) so this is a no-op in legacy scenarios.
        if (tracker != nullptr && cluster_eps_m > 0.0f) {
            tracker->update(voxels, masks_opt->timestamp_ns);

            // Issue #645 — surface stable voxel-derived tracks to the fusion
            // thread so ObstacleAvoider3D can repel from non-COCO obstacles
            // (cubes, pillars, walls) that are visible to PATH A but not to
            // YOLO.  Lock-free hand-off via TripleBuffer: producer always
            // writes, consumer always reads the latest.
            if (voxel_obstacle_buf != nullptr) {
                drone::perception::VoxelObstacleSnapshot snap{};
                snap.timestamp_ns      = masks_opt->timestamp_ns;
                const auto& tracks_map = tracker->tracks();
                uint32_t    n          = 0;
                for (const auto& [stable_id, tr] : tracks_map) {
                    if (n >= drone::perception::kMaxVoxelObstacleEntries) break;
                    auto& e             = snap.entries[n];
                    e.stable_id         = stable_id;
                    e.centroid_x        = tr.centroid_m.x();
                    e.centroid_y        = tr.centroid_m.y();
                    e.centroid_z        = tr.centroid_m.z();
                    e.aabb_min_x        = tr.aabb_min_m.x();
                    e.aabb_min_y        = tr.aabb_min_m.y();
                    e.aabb_min_z        = tr.aabb_min_m.z();
                    e.aabb_max_x        = tr.aabb_max_m.x();
                    e.aabb_max_y        = tr.aabb_max_m.y();
                    e.aabb_max_z        = tr.aabb_max_m.z();
                    e.last_seen_ns      = tr.last_seen_ns;
                    e.observation_count = tr.observation_count;
                    ++n;
                }
                snap.num_entries = n;
                voxel_obstacle_buf->write(std::move(snap));
            }
        }

        if (cluster_eps_m > 0.0f && tick_count % 100 == 1) {
            DRONE_LOG_INFO("[VoxelCluster] frame_seq={}: {} voxels → {} clusters → {} tracks "
                           "(eps={:.2f}m, min_pts={})",
                           masks_opt->frame_sequence, voxels.size(), n_clusters,
                           tracker != nullptr ? tracker->track_count() : 0u, cluster_eps_m,
                           cluster_min_pts);
        }

        // Pack VoxelUpdate[] -> SemanticVoxelBatch.  Truncate by descending
        // confidence if we ever overshoot MAX_VOXELS_PER_BATCH (see #608).
        // Use a pre-allocated stack-resident batch_buf (review P1-D from
        // PR #639) — `make_unique` per tick was a multi-KB heap allocation
        // on a 10 Hz flight-critical thread.  The struct is trivially-
        // copyable so reassigning fields in place is safe.
        auto& batch           = batch_buf;
        batch.timestamp_ns    = masks_opt->timestamp_ns;
        batch.frame_sequence  = static_cast<uint64_t>(masks_opt->frame_sequence);
        const size_t n_voxels = std::min<size_t>(voxels.size(), drone::ipc::MAX_VOXELS_PER_BATCH);
        batch.num_voxels      = static_cast<uint32_t>(n_voxels);
        for (size_t i = 0; i < n_voxels; ++i) {
            const auto& v   = voxels[i];
            auto&       out = batch.voxels[i];
            out.position_x  = v.position_m.x();
            out.position_y  = v.position_m.y();
            out.position_z  = v.position_m.z();
            out.occupancy   = std::clamp(v.occupancy, 0.0f, 1.0f);
            out.confidence  = std::clamp(v.confidence, 0.0f, 1.0f);
            // hal::VoxelUpdate::semantic_label is a uint8_t that the
            // CpuSemanticProjector sets from the detection's assigned class.
            // Clamp to the ObjectClass range so validate() accepts it.
            const uint8_t label_byte = std::min<uint8_t>(
                v.semantic_label,
                static_cast<uint8_t>(drone::ipc::ObjectClass::GEOMETRIC_OBSTACLE));
            out.semantic_label = static_cast<drone::ipc::ObjectClass>(label_byte);
            out.timestamp_ns   = v.timestamp_ns;
            out.instance_id    = v.instance_id;
        }
        voxel_pub.publish(batch);
        ++published_count;

        // First-publish marker — short runs that fail before 50 batches still
        // emit "[MaskProj] Published …" once, so scenario pass_criteria log
        // greps (see config/scenarios/33_non_coco_obstacles.json) still fire.
        if (published_count == 1) {
            DRONE_LOG_INFO("[MaskProj] Published first batch — {} voxels", batch.num_voxels);
        }

        // Diagnostic trace (Issue #612).  Inert when disabled.  Writes the
        // full pose / bboxes / voxel positions for off-line analysis of the
        // "voxels aren't landing on the cube" class of bugs.
        if (trace && trace->enabled()) {
            trace->record_batch(masks_opt->timestamp_ns,
                                static_cast<uint64_t>(masks_opt->frame_sequence), *latest_pose,
                                det_hal, masks_opt->masks, voxels);
        }

        if (published_count % 50 == 0) {
            DRONE_LOG_INFO("[MaskProj] Published {} batches — last batch: {} voxels, {} skipped",
                           published_count, batch.num_voxels, skipped_count);
        }
    }
    DRONE_LOG_INFO("[MaskProj] Thread stopped — {} ticks, {} published, {} skipped", tick_count,
                   published_count, skipped_count);
}

// ── Tracker thread ──────────────────────────────────────────
// Stops when shutdown_phase >= 2.  When phase hits 1 (inference stopped),
// the tracker drains any remaining data in the inference->tracker buffer
// before exiting.
// @param profiler nullable — see inference_thread docstring and DR-022.
static void tracker_thread(drone::TripleBuffer<Detection2DList>&   input_queue,
                           drone::TripleBuffer<TrackedObjectList>& output_queue,
                           std::atomic<int>& shutdown_phase, ITracker& tracker,
                           int drain_timeout_ms, drone::util::LatencyProfiler* profiler) {
    DRONE_LOG_INFO("[Tracker] Thread started — backend: {}", tracker.name());

    auto hb = drone::util::ScopedHeartbeat("tracker", true);

    constexpr uint64_t kStatusInterval = 50;  // Log tracker status every N cycles
    uint64_t           cycle_count     = 0;

    // Drain state: track when we enter drain mode and enforce a timeout.
    bool tracker_draining    = false;
    auto tracker_drain_start = std::chrono::steady_clock::now();

    while (shutdown_phase.load(std::memory_order_acquire) < 2) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());

        // Enter drain mode when inference has stopped (phase >= 1)
        if (!tracker_draining && shutdown_phase.load(std::memory_order_acquire) >= 1) {
            tracker_draining    = true;
            tracker_drain_start = std::chrono::steady_clock::now();
            DRONE_LOG_INFO("[Tracker] Entering drain mode — consuming remaining detections");
        }

        auto det_opt = input_queue.read();
        if (det_opt) {
            drone::util::FrameDiagnostics diag(det_opt->frame_sequence);

            auto tracked = [&]() {
                drone::util::ScopedDiagTimer              timer(diag, "Track");
                std::optional<drone::util::ScopedLatency> bench_track;  // See DR-022
                if (profiler) bench_track.emplace(*profiler, "Track");
                return tracker.update(*det_opt);
            }();

            diag.add_metric("Track", "n_tracks", static_cast<double>(tracked.objects.size()));

            if (!tracked.objects.empty()) {
                output_queue.write(std::move(tracked));
            }
            ++cycle_count;

            if (cycle_count % kStatusInterval == 0) {
                DRONE_LOG_INFO("[Tracker] Status: backend={}, cycles={}, writes={}", tracker.name(),
                               cycle_count, output_queue.write_count());
            }

            if (diag.has_warnings() || diag.has_errors()) {
                diag.log_summary("Tracker");
            }
        } else if (tracker_draining) {
            // No new data and upstream is done — drain complete.
            DRONE_LOG_INFO("[Tracker] Drain complete — no remaining data from inference");
            break;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        // Drain timeout: prevent hanging if TripleBuffer keeps producing
        // stale reads during shutdown.
        if (tracker_draining) {
            const auto elapsed = std::chrono::steady_clock::now() - tracker_drain_start;
            if (elapsed >= std::chrono::milliseconds(drain_timeout_ms)) {
                DRONE_LOG_WARN("[Tracker] Drain timeout ({}ms) — forcing exit", drain_timeout_ms);
                break;
            }
        }
    }
    DRONE_LOG_INFO("[Tracker] Thread stopped — {} cycles, {} writes", cycle_count,
                   output_queue.write_count());
}

// ── Fusion thread ───────────────────────────────────────────
// pose_sub  — IPC subscriber for drone/slam/pose (Pose).
//             Used to rotate camera-frame detections into world frame.
// radar_sub — IPC subscriber for radar detections (RadarDetectionList).
//             Fed into fusion engine for camera+radar multi-sensor fusion.
// @param profiler nullable — see inference_thread docstring and DR-022.
static void fusion_thread(
    drone::TripleBuffer<TrackedObjectList>&                  tracked_queue,
    drone::ipc::IPublisher<drone::ipc::DetectedObjectList>&  det_pub,
    drone::ipc::ISubscriber<drone::ipc::Pose>&               pose_sub,
    drone::ipc::ISubscriber<drone::ipc::RadarDetectionList>& radar_sub,
    std::atomic<int>& shutdown_phase, IFusionEngine& engine, int fusion_rate_hz,
    drone::TripleBuffer<drone::hal::DepthMap>* depth_buf, int drain_timeout_ms,
    drone::util::LatencyProfiler* profiler,
    // Issue #645 — voxel-derived obstacles (PATH A) appended
    // to the published DetectedObjectList so ObstacleAvoider3D
    // sees non-COCO obstacles.  nullptr disables surfacing.
    drone::TripleBuffer<drone::perception::VoxelObstacleSnapshot>* voxel_obstacle_buf,
    uint32_t voxel_obstacle_min_observations, uint64_t voxel_obstacle_max_age_ns) {
    DRONE_LOG_INFO("[Fusion] Thread started — backend: {}, rate: {} Hz", engine.name(),
                   fusion_rate_hz);

    auto hb = drone::util::ScopedHeartbeat("fusion", true);

    uint64_t fusion_count = 0;

    // Cached latest drone pose (world frame: North, East, Up)
    drone::ipc::Pose latest_pose{};
    bool             has_pose = false;

    // Rate limiting — sleep_until for consistent cadence
    const auto period    = std::chrono::milliseconds(1000 / fusion_rate_hz);
    auto       next_tick = std::chrono::steady_clock::now();

    // Drain tracking: once phase >= 2 (tracker stopped), consume remaining data.
    bool fusion_draining    = false;
    auto fusion_drain_start = std::chrono::steady_clock::now();

    while (shutdown_phase.load(std::memory_order_acquire) < 3) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());

        // Enter drain mode when tracker has stopped (phase >= 2)
        if (!fusion_draining && shutdown_phase.load(std::memory_order_acquire) >= 2) {
            fusion_draining    = true;
            fusion_drain_start = std::chrono::steady_clock::now();
            DRONE_LOG_INFO("[Fusion] Entering drain mode — consuming remaining tracked data");
        }

        // Read latest pose (ZenohSubscriber is latest-value, not queue — single read)
        {
            drone::ipc::Pose p{};
            if (pose_sub.receive(p)) {
                if (!has_pose) {
                    DRONE_LOG_INFO("[Fusion] First SLAM pose received: ({:.2f}, {:.2f}, {:.2f})",
                                   p.translation[0], p.translation[1], p.translation[2]);
                }
                latest_pose = p;
                has_pose    = true;
            } else if (fusion_count > 0 && fusion_count % 300 == 0 && !has_pose) {
                DRONE_LOG_WARN("[Fusion] Still no SLAM pose after {} cycles (sub connected={})",
                               fusion_count, pose_sub.is_connected());
            }
        }

        // Pass drone altitude for radar ground-plane filtering.
        // translation[2] is world-frame z (AGL when world origin is on the ground).
        if (has_pose) {
            engine.set_drone_altitude(static_cast<float>(latest_pose.translation[2]));

            // Pass full pose for dormant obstacle re-identification.
            const double qw  = latest_pose.quaternion[0];
            const double qx  = latest_pose.quaternion[1];
            const double qy  = latest_pose.quaternion[2];
            const double qz  = latest_pose.quaternion[3];
            const float  yaw = static_cast<float>(
                std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));
            engine.set_drone_pose(static_cast<float>(latest_pose.translation[0]),
                                  static_cast<float>(latest_pose.translation[1]),
                                  static_cast<float>(latest_pose.translation[2]), yaw);
        }

        // Read latest radar detections (latest-value — single read)
        {
            drone::ipc::RadarDetectionList radar_list{};
            if (radar_sub.receive(radar_list)) {
                engine.set_radar_detections(radar_list);
            }
        }

        // Read latest ML depth map for depth estimation enhancement (Issue #430)
        if (depth_buf) {
            if (auto dopt = depth_buf->read()) {
                engine.set_depth_map(std::move(*dopt));
            }
        }

        // Fuse when tracking data arrives (latest-value, never stale)
        if (auto topt = tracked_queue.read()) {
            drone::util::FrameDiagnostics diag(fusion_count);

            auto fused = [&]() {
                drone::util::ScopedDiagTimer              timer(diag, "Fuse");
                std::optional<drone::util::ScopedLatency> bench_fuse;  // See DR-022
                if (profiler) bench_fuse.emplace(*profiler, "Fuse");
                return engine.fuse(*topt);
            }();

            diag.add_metric("Fuse", "n_fused_objects", static_cast<double>(fused.objects.size()));

            // ── Full quaternion camera→world transform (Issue #421) ──
            // fusion_engine returns position_3d in camera body frame (FRD):
            //   cam.x = forward (along boresight)
            //   cam.y = right
            //   cam.z = down
            // World frame (NEU): translation[0]=North, [1]=East, [2]=Up.
            //
            // Previous: yaw-only rotation ignored pitch/roll, causing
            // systematic depth errors during forward flight (5-15deg pitch).
            // Now: use full VIO quaternion for body FRD → world NEU.
            //
            // Transform chain:
            //   1. Negate Z to convert FRD → FRU (body forward-right-up)
            //   2. Apply full rotation matrix from VIO quaternion (FRU → NEU)
            //   3. Translate by drone world position
            if (has_pose) {
                // Full rotation from VIO quaternion (w, x, y, z order).
                // Stay in float — UKF state is float, ~3mm precision at 100m is sufficient
                // for obstacle avoidance (review fix #9).
                const Eigen::Quaternionf q(static_cast<float>(latest_pose.quaternion[0]),
                                           static_cast<float>(latest_pose.quaternion[1]),
                                           static_cast<float>(latest_pose.quaternion[2]),
                                           static_cast<float>(latest_pose.quaternion[3]));
                // Guard degenerate quaternion — NaN would propagate to all positions (review fix #11)
                if (q.norm() < 1e-6f) {
                    DRONE_LOG_DEBUG("[Fusion] Skipping transform — degenerate quaternion");
                    continue;
                }
                const Eigen::Matrix3f R = q.normalized().toRotationMatrix();
                // Note: yaw for set_drone_pose() is extracted separately at ~line 182.

                const float dn = static_cast<float>(latest_pose.translation[0]);
                const float de = static_cast<float>(latest_pose.translation[1]);
                const float du = static_cast<float>(latest_pose.translation[2]);

                for (auto& obj : fused.objects) {
                    // Velocity: body FRD → FRU (negate z) → rotate to world NEU
                    const Eigen::Vector3f v_fru(obj.velocity_3d.x(), obj.velocity_3d.y(),
                                                -obj.velocity_3d.z());
                    obj.velocity_3d = R * v_fru;

                    // Re-identified objects already have world-frame positions
                    // from the dormant obstacle pool — skip position transform.
                    if (obj.in_world_frame) continue;

                    // Position: body FRD → FRU (negate z) → rotate → translate
                    const Eigen::Vector3f p_fru(obj.position_3d.x(), obj.position_3d.y(),
                                                -obj.position_3d.z());
                    obj.position_3d = R * p_fru + Eigen::Vector3f(dn, de, du);
                }
            }

            // Only publish once we have a valid pose — positions are meaningless
            // in camera frame and would produce incorrect avoidance reactions.
            if (!has_pose) {
                DRONE_LOG_DEBUG("[Fusion] Skipping publish — no pose available yet");
                continue;
            }

            // Publish to SHM (world-frame positions)
            drone::ipc::DetectedObjectList shm_list{};
            shm_list.timestamp_ns   = fused.timestamp_ns;
            shm_list.frame_sequence = fused.frame_sequence;
            shm_list.num_objects =
                std::min(static_cast<uint32_t>(fused.objects.size()),
                         static_cast<uint32_t>(drone::ipc::MAX_DETECTED_OBJECTS));

            for (uint32_t i = 0; i < shm_list.num_objects; ++i) {
                auto& src              = fused.objects[i];
                auto& dst              = shm_list.objects[i];
                dst.track_id           = src.track_id;
                dst.class_id           = static_cast<drone::ipc::ObjectClass>(src.class_id);
                dst.confidence         = src.confidence;
                dst.position_x         = src.position_3d.x();
                dst.position_y         = src.position_3d.y();
                dst.position_z         = src.position_3d.z();
                dst.velocity_x         = src.velocity_3d.x();
                dst.velocity_y         = src.velocity_3d.y();
                dst.velocity_z         = src.velocity_3d.z();
                dst.heading            = src.heading;
                dst.has_camera         = src.has_camera;
                dst.has_radar          = src.has_radar;
                dst.estimated_radius_m = src.estimated_radius_m;
                dst.estimated_height_m = src.estimated_height_m;
                dst.radar_update_count = src.radar_update_count;
                // Some fusion backends (notably camera_only) do not populate
                // depth_confidence.  Fall back to object confidence so
                // downstream promotion logic receives a meaningful signal.
                float dc = src.depth_confidence;
                if (dc <= 0.0f && src.has_camera) {
                    dc = src.confidence;
                }
                dst.depth_confidence = std::clamp(dc, 0.0f, 1.0f);
            }

            // Issue #645 — append voxel-derived obstacles from PATH A so
            // ObstacleAvoider3D can repel from non-COCO geometry the
            // detector path never produces (cubes, pillars, walls).  The
            // mask_projection_thread writes a snapshot of stable Phase 2
            // tracks; we read the latest snapshot and synthesise
            // GEOMETRIC_OBSTACLE entries via a pure helper (testable
            // without spinning up the full thread).
            if (voxel_obstacle_buf != nullptr) {
                if (auto snap_opt = voxel_obstacle_buf->read()) {
                    const uint64_t now_ns = static_cast<uint64_t>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count());
                    auto stats = drone::perception::append_voxel_obstacles_to_list(
                        *snap_opt, shm_list, now_ns, voxel_obstacle_min_observations,
                        voxel_obstacle_max_age_ns);
                    if (stats.appended > 0 && fusion_count % 100 == 0) {
                        DRONE_LOG_INFO("[Fusion] +{} voxel obstacles "
                                       "(snap={} skipped_obs={} skipped_age={} skipped_full={})",
                                       stats.appended, snap_opt->num_entries, stats.skipped_obs,
                                       stats.skipped_age, stats.skipped_full);
                    }
                }
            }

            det_pub.publish(shm_list);
            ++fusion_count;

            if (diag.has_warnings() || diag.has_errors()) {
                diag.log_summary("Fusion");
            } else if (fusion_count % 100 == 0) {
                DRONE_LOG_INFO("[Fusion] {} cycles, {} fused objects this frame", fusion_count,
                               fused.objects.size());
            }

            // Log IPC latency from the thread that owns receive()
            pose_sub.log_latency_if_due();
            radar_sub.log_latency_if_due();
        } else if (fusion_draining) {
            // No new tracked data and tracker is done — drain complete.
            DRONE_LOG_INFO("[Fusion] Drain complete — no remaining tracked data");
            break;
        }

        // Drain timeout: prevent hanging if TripleBuffer keeps producing
        // stale reads during shutdown.
        if (fusion_draining) {
            const auto elapsed = std::chrono::steady_clock::now() - fusion_drain_start;
            if (elapsed >= std::chrono::milliseconds(drain_timeout_ms)) {
                DRONE_LOG_WARN("[Fusion] Drain timeout ({}ms) — forcing exit", drain_timeout_ms);
                break;
            }
        }

        // Rate-limited sleep — reset next_tick if we fell behind to avoid
        // catch-up spinning (sleep_until returning immediately for many iterations).
        next_tick += period;
        auto now = std::chrono::steady_clock::now();
        if (now > next_tick) {
            next_tick = now;
        }
        std::this_thread::sleep_until(next_tick);
    }
    DRONE_LOG_INFO("[Fusion] Thread stopped after {} cycles (reads={})", fusion_count,
                   tracked_queue.read_count());
}

// ── Depth estimation thread (Issue #430) ───────────────────
// Reads video frames, runs ML depth estimation, publishes results
// to a triple buffer for consumption by the fusion thread.
// Stops when shutdown_phase >= 3.  Depth is a leaf node (writes to fusion's
// TripleBuffer) — no downstream drain needed, just stop consuming frames.
// @param projection_output_queue nullable — when PATH A is enabled
//                 (Epic #520, Issue #608), mask_projection_thread needs its
//                 own depth stream.  TripleBuffer is single-consumer, so we
//                 fanout by writing the same depth map twice.  Same pattern
//                 as inference_thread's projection_output_queue.
static void depth_thread(drone::ipc::ISubscriber<drone::ipc::VideoFrame>& video_sub,
                         drone::TripleBuffer<drone::hal::DepthMap>&       output_queue,
                         std::atomic<int>& shutdown_phase, drone::hal::IDepthEstimator& estimator,
                         int                                        max_fps,
                         drone::TripleBuffer<drone::hal::DepthMap>* projection_output_queue) {
    DRONE_LOG_INFO("[Depth] Thread started — backend: {}, max_fps: {}{}", estimator.name(), max_fps,
                   projection_output_queue ? " (PATH A fanout active)" : "");

    auto hb = drone::util::ScopedHeartbeat("depth", true);

    // Rate limiting — avoid running faster than the model can process
    const auto min_period = (max_fps > 0) ? std::chrono::milliseconds(1000 / max_fps)
                                          : std::chrono::milliseconds(0);
    auto       last_run   = std::chrono::steady_clock::now();

    uint64_t frame_count = 0;
    uint64_t fail_count  = 0;
    while (shutdown_phase.load(std::memory_order_acquire) < 3) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());

        // Rate limit BEFORE receive() to avoid copying a ~6MB VideoFrame
        // that would be immediately dropped. Sleep until the next allowed tick.
        auto now = std::chrono::steady_clock::now();
        if (now < last_run + min_period) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        drone::ipc::VideoFrame frame;
        bool                   got_frame = video_sub.receive(frame);

        if (got_frame) {
            last_run = std::chrono::steady_clock::now();

            auto result = estimator.estimate(frame.pixel_data, frame.width, frame.height,
                                             frame.channels);
            if (result.is_ok()) {
                auto depth_map         = std::move(result).value();
                depth_map.timestamp_ns = frame.timestamp_ns;
                // PATH A fanout — mask_projection_thread needs its own depth
                // stream.  Write the copy first so output_queue can move the
                // original into its scratch slot without reallocation.
                if (projection_output_queue) {
                    projection_output_queue->write(depth_map);  // copy
                }
                output_queue.write(std::move(depth_map));
                ++frame_count;
                fail_count = 0;  // reset on success

                if (frame_count % 100 == 0) {
                    DRONE_LOG_INFO("[Depth] Processed {} frames (writes={})", frame_count,
                                   output_queue.write_count());
                }
            } else {
                ++fail_count;
                // Throttled warning: log first failure and every 100th after
                if (fail_count == 1 || fail_count % 100 == 0) {
                    DRONE_LOG_WARN("[Depth] estimate() failed ({} consecutive): {}", fail_count,
                                   result.error());
                }
                // Backpressure: sleep to avoid hot-looping on persistent HAL failure
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    DRONE_LOG_INFO("[Depth] Thread stopped — {} frames, {} writes", frame_count,
                   output_queue.write_count());
}

// ── Radar HAL read thread ──────────────────────────────────
// Polls the radar HAL backend at its configured update rate and publishes
// RadarDetectionList to IPC for consumption by the fusion thread.
// Stops when shutdown_phase >= 3.  Radar is a leaf node (publishes to IPC
// consumed by fusion) — no downstream drain needed, just stop polling.
static void radar_read_thread(drone::hal::IRadar&                                     radar,
                              drone::ipc::IPublisher<drone::ipc::RadarDetectionList>& radar_pub,
                              std::atomic<int>& shutdown_phase, int update_rate_hz) {
    // Clamp update rate to a sane range to prevent tight loops or division issues
    constexpr int kMinRateHz        = 1;
    constexpr int kMaxRateHz        = 1000;
    int           effective_rate_hz = update_rate_hz;
    if (effective_rate_hz < kMinRateHz || effective_rate_hz > kMaxRateHz) {
        DRONE_LOG_WARN("[Radar] Invalid update rate {} Hz — clamping to [{}, {}]", update_rate_hz,
                       kMinRateHz, kMaxRateHz);
        effective_rate_hz = std::clamp(effective_rate_hz, kMinRateHz, kMaxRateHz);
    }

    const auto period =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(1)) /
        effective_rate_hz;

    DRONE_LOG_INFO("[Radar] Read thread started — backend: {}, rate: {} Hz, period: {} ms",
                   radar.name(), effective_rate_hz, period.count());

    auto hb = drone::util::ScopedHeartbeat("radar_read", true);

    uint64_t read_count = 0;

    while (shutdown_phase.load(std::memory_order_acquire) < 3) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());

        auto detections = radar.read();
        if (detections.num_detections > 0) {
            radar_pub.publish(detections);
            ++read_count;
        }

        std::this_thread::sleep_for(period);
    }

    DRONE_LOG_INFO("[Radar] Read thread stopped — {} publishes", read_count);
}

// ═══════════════════════════════════════════════════════════
// main()
// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    // ── Common boilerplate (args, signals, logging, config, bus) ──
    auto ctx_result = drone::util::init_process(argc, argv, "perception", g_running,
                                                drone::util::perception_schema());
    if (!ctx_result.is_ok()) return ctx_result.error();
    auto& ctx = ctx_result.value();

    // ── Subscribe to video frames from Process 1 ────────────
    auto video_sub =
        ctx.bus.subscribe<drone::ipc::VideoFrame>(drone::ipc::topics::VIDEO_MISSION_CAM);
    if (!video_sub->is_connected()) {
        DRONE_LOG_ERROR("Cannot connect to video channel — is video_capture running?");
        return 1;
    }

    // ── Create publisher for detected objects → Process 4 ───
    auto det_pub =
        ctx.bus.advertise<drone::ipc::DetectedObjectList>(drone::ipc::topics::DETECTED_OBJECTS);
    if (!det_pub->is_ready()) {
        DRONE_LOG_ERROR("Failed to create publisher: {}", drone::ipc::topics::DETECTED_OBJECTS);
        return 1;
    }

    // ── Create detector from config ────────────────────────────
    std::string detector_backend =
        ctx.cfg.get<std::string>(drone::cfg_key::perception::detector::BACKEND, "simulated");
    auto detector = create_detector(detector_backend, &ctx.cfg);
    DRONE_LOG_INFO("[Perception] Detector backend: {} ({})", detector_backend, detector->name());

    // ── Create tracker from config ────────────────────────────
    std::string tracker_backend =
        ctx.cfg.get<std::string>(drone::cfg_key::perception::tracker::BACKEND, "bytetrack");
    auto tracker_result = create_tracker(tracker_backend, &ctx.cfg);
    if (!tracker_result.is_ok()) {
        DRONE_LOG_ERROR("[Perception] Failed to create tracker: {}",
                        tracker_result.error().message());
        return 1;
    }
    auto tracker = std::move(tracker_result).value();
    DRONE_LOG_INFO("[Perception] Tracker  backend: {} ({})", tracker_backend, tracker->name());

    // ── Create fusion engine from config ────────────────────
    CalibrationData calib;
    calib.camera_intrinsics       = Eigen::Matrix3f::Identity();
    calib.camera_intrinsics(0, 0) = ctx.cfg.get<float>(drone::cfg_key::perception::fusion::FX,
                                                       500.0f);
    calib.camera_intrinsics(1, 1) = ctx.cfg.get<float>(drone::cfg_key::perception::fusion::FY,
                                                       500.0f);
    calib.camera_intrinsics(0, 2) = ctx.cfg.get<float>(drone::cfg_key::perception::fusion::CX,
                                                       960.0f);
    calib.camera_intrinsics(1, 2) = ctx.cfg.get<float>(drone::cfg_key::perception::fusion::CY,
                                                       540.0f);
    calib.camera_height_m = ctx.cfg.get<float>(drone::cfg_key::perception::fusion::CAMERA_HEIGHT_M,
                                               1.5f);
    calib.assumed_obstacle_height_m =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::ASSUMED_OBSTACLE_HEIGHT_M, 3.0f);
    calib.depth_scale = ctx.cfg.get<float>(drone::cfg_key::perception::fusion::DEPTH_SCALE, 0.7f);

    // Per-class height priors for apparent-size depth estimation (Issue #423)
    calib.height_priors[static_cast<uint8_t>(drone::perception::ObjectClass::UNKNOWN)] =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::HEIGHT_PRIORS_UNKNOWN, 3.0f);
    calib.height_priors[static_cast<uint8_t>(drone::perception::ObjectClass::PERSON)] =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::HEIGHT_PRIORS_PERSON, 1.7f);
    calib.height_priors[static_cast<uint8_t>(drone::perception::ObjectClass::VEHICLE_CAR)] =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::HEIGHT_PRIORS_VEHICLE_CAR, 1.5f);
    calib.height_priors[static_cast<uint8_t>(drone::perception::ObjectClass::VEHICLE_TRUCK)] =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::HEIGHT_PRIORS_VEHICLE_TRUCK, 3.5f);
    calib.height_priors[static_cast<uint8_t>(drone::perception::ObjectClass::DRONE)] =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::HEIGHT_PRIORS_DRONE, 0.3f);
    calib.height_priors[static_cast<uint8_t>(drone::perception::ObjectClass::ANIMAL)] =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::HEIGHT_PRIORS_ANIMAL, 0.8f);
    calib.height_priors[static_cast<uint8_t>(drone::perception::ObjectClass::BUILDING)] =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::HEIGHT_PRIORS_BUILDING, 10.0f);
    calib.height_priors[static_cast<uint8_t>(drone::perception::ObjectClass::TREE)] =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::HEIGHT_PRIORS_TREE, 6.0f);
    calib.height_priors[static_cast<uint8_t>(drone::perception::ObjectClass::GEOMETRIC_OBSTACLE)] =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::HEIGHT_PRIORS_GEOMETRIC_OBSTACLE,
                           2.0f);
    calib.bbox_height_noise_px =
        ctx.cfg.get<float>(drone::cfg_key::perception::fusion::BBOX_HEIGHT_NOISE_PX, 2.5f);

    // Validate height priors — zero/negative values produce nonsensical depth (review fix #5)
    for (auto& hp : calib.height_priors) {
        hp = std::max(0.1f, hp);
    }

    std::string fusion_backend =
        ctx.cfg.get<std::string>(drone::cfg_key::perception::fusion::BACKEND, "camera_only");
    auto fusion_engine = create_fusion_engine(fusion_backend, calib, &ctx.cfg);
    DRONE_LOG_INFO("[Perception] Fusion   backend: {} ({})", fusion_backend, fusion_engine->name());

    // ── Create radar HAL + publisher (optional) ────────────
    bool radar_enabled = ctx.cfg.get<bool>(drone::cfg_key::perception::radar::ENABLED, false);
    std::unique_ptr<drone::hal::IRadar>                                     radar;
    std::unique_ptr<drone::ipc::IPublisher<drone::ipc::RadarDetectionList>> radar_pub;
    int radar_update_rate_hz = ctx.cfg.get<int>(drone::cfg_key::perception::radar::UPDATE_RATE_HZ,
                                                20);

    if (radar_enabled) {
        try {
            radar = drone::hal::create_radar(ctx.cfg, drone::cfg_key::perception::radar::SECTION);
            if (!radar->init()) {
                DRONE_LOG_ERROR("[Radar] HAL init() failed — radar disabled");
                radar.reset();
            } else {
                radar_pub = ctx.bus.advertise<drone::ipc::RadarDetectionList>(
                    drone::ipc::topics::RADAR_DETECTIONS);
                DRONE_LOG_INFO("[Perception] Radar HAL: {} — publishing to {}", radar->name(),
                               drone::ipc::topics::RADAR_DETECTIONS);
            }
        } catch (const std::exception& e) {
            DRONE_LOG_ERROR("[Radar] Failed to create HAL backend: {} — radar disabled", e.what());
            radar.reset();
        }
    } else {
        DRONE_LOG_INFO("[Perception] Radar disabled (perception.radar.enabled=false)");
    }

    // ── Create depth estimator HAL (optional, Issue #430) ──
    bool depth_enabled = ctx.cfg.get<bool>(drone::cfg_key::perception::depth_estimator::ENABLED,
                                           false);
    std::unique_ptr<drone::hal::IDepthEstimator> depth_estimator;
    if (depth_enabled) {
        try {
            depth_estimator = drone::hal::create_depth_estimator(
                ctx.cfg, drone::cfg_key::perception::depth_estimator::SECTION);
            DRONE_LOG_INFO("[Perception] Depth estimator: {}", depth_estimator->name());
        } catch (const std::exception& e) {
            DRONE_LOG_ERROR("[Depth] Failed to create HAL backend: {} — depth disabled", e.what());
            depth_estimator.reset();
            depth_enabled = false;
        }
    } else {
        DRONE_LOG_INFO("[Perception] Depth estimator disabled "
                       "(perception.depth_estimator.enabled=false)");
    }

    // ── PATH A — SAM + mask projection (Epic #520, Issue #608) ──
    // Gated on perception.path_a.enabled (default false); scenario 33
    // turns it on to exercise the mask-aware voxelisation path that
    // classifies non-COCO obstacles (cubes, panels, pillars) as
    // GEOMETRIC_OBSTACLE voxels for the P4 occupancy grid.
    const bool path_a_enabled = ctx.cfg.get<bool>(drone::cfg_key::perception::path_a::ENABLED,
                                                  false);
    std::unique_ptr<drone::hal::IInferenceBackend>                          sam_backend;
    std::unique_ptr<drone::hal::ISemanticProjector>                         semantic_projector;
    std::unique_ptr<drone::perception::MaskDepthProjector>                  mask_projector;
    std::unique_ptr<drone::ipc::IPublisher<drone::ipc::SemanticVoxelBatch>> voxel_pub;
    if (path_a_enabled) {
        try {
            // Route through the HAL factory so `perception.path_a.sam.backend`
            // in config actually selects the backend.  The factory reads
            // `{section}.backend` under the SAM section and returns the matching
            // IInferenceBackend ("sam_simulated", "simulated", or plugin-loaded).
            // Previous code hardcoded SimulatedSAMBackend regardless of the
            // config value — that's why `sam.backend = "yolov8_seg"` had no
            // effect.  Factory still returns SimulatedSAMBackend when the value
            // is "sam_simulated" (scenario 33's current setting).
            const std::string sam_section =
                std::string(drone::cfg_key::perception::path_a::SECTION) + ".sam";
            sam_backend = drone::hal::create_inference_backend(ctx.cfg, sam_section);
            const std::string sam_model_path =
                ctx.cfg.get<std::string>(drone::cfg_key::perception::path_a::SAM_MODEL_PATH, "");
            const int sam_input_size =
                ctx.cfg.get<int>(drone::cfg_key::perception::path_a::SAM_INPUT_SIZE, 512);
            if (!sam_backend->init(sam_model_path, sam_input_size)) {
                DRONE_LOG_ERROR("[PathA] SAM init() failed (backend={}, model='{}') — PATH A "
                                "disabled",
                                sam_backend->name(), sam_model_path);
                sam_backend.reset();
            }
        } catch (const std::exception& e) {
            DRONE_LOG_ERROR("[PathA] SAM construction failed: {} — PATH A disabled", e.what());
            sam_backend.reset();
        }

        if (sam_backend) {
            auto                         sp = std::make_unique<drone::hal::CpuSemanticProjector>();
            drone::hal::CameraIntrinsics intr;
            intr.fx = calib.camera_intrinsics(0, 0);
            intr.fy = calib.camera_intrinsics(1, 1);
            intr.cx = calib.camera_intrinsics(0, 2);
            intr.cy = calib.camera_intrinsics(1, 2);
            intr.width =
                static_cast<uint32_t>(ctx.cfg.get<int>("video_capture.mission_cam.width", 1280));
            intr.height =
                static_cast<uint32_t>(ctx.cfg.get<int>("video_capture.mission_cam.height", 720));
            if (!sp->init(intr)) {
                DRONE_LOG_ERROR("[PathA] CpuSemanticProjector init failed (intrinsics={}x{}) — "
                                "PATH A disabled",
                                intr.width, intr.height);
                sam_backend.reset();
            } else {
                // Issue #616 — opt-in texture gate on the depth-map Sobel
                // magnitude.  Zero default is backward-compatible; scenarios
                // that want to reject samples on flat/textureless surfaces
                // set a non-zero threshold (tuned empirically against
                // scenario voxel-on-target output; 0.3–1.0 is a reasonable
                // range for DA V2 @ 518×518).
                const float tgate = ctx.cfg.get<float>(
                    drone::cfg_key::perception::semantic_projector::TEXTURE_GATE_THRESHOLD, 0.0f);
                sp->set_texture_gate_threshold(tgate);
                if (tgate > 0.0f) {
                    DRONE_LOG_INFO("[PathA] CpuSemanticProjector texture gate enabled: "
                                   "threshold={:.3f} (un-normalised Sobel magnitude)",
                                   tgate);
                }
                // Pipe the estimator's max depth through so the projector's
                // horizon-cutoff stays in sync with the configured depth range
                // (PR #620 code-quality review — previously hardcoded at 20 m).
                const float max_depth = ctx.cfg.get<float>("perception.depth_estimator.max_depth_m",
                                                           20.0f);
                sp->set_max_obstacle_depth_m(max_depth);

                // Issue #629 — mask sampling density.  Legacy default 4×4=16
                // probes per mask.  Dense-perception / no-HD-map scenarios
                // (scenario 33) override to 8 or 16 for higher per-frame
                // discovery rate.  Logged unconditionally so post-restart
                // operators can confirm what density is actually active —
                // a silent fallback to N=4 in a scenario that needs N=16
                // is a 16× drop in obstacle discovery (review: PR #630
                // fault-recovery finding 2).
                //
                // Uses `cfg.require<int>()` rather than `cfg.get<int>()` so a
                // wrong-type config value (e.g. `"sample_grid_size": "dense"`)
                // surfaces as a WARN instead of silently falling through to
                // the default 4 (PR #630 review-security P3 #2).
                constexpr int kDefaultGridSize = 4;
                int           grid_size_raw    = kDefaultGridSize;
                const auto    cfg_key_str =
                    std::string(drone::cfg_key::perception::semantic_projector::SAMPLE_GRID_SIZE);
                if (auto r = ctx.cfg.require<int>(cfg_key_str); r.is_ok()) {
                    grid_size_raw = r.value();
                } else {
                    if (r.error().code() == drone::util::ErrorCode::TypeMismatch) {
                        DRONE_LOG_WARN("[PathA] {} has wrong JSON type — using default {} "
                                       "(error: {})",
                                       cfg_key_str, kDefaultGridSize, r.error().message());
                    }
                    // MissingKey is the common case (no override) — silent.
                }
                sp->set_sample_grid_size(grid_size_raw);
                const int grid_size_eff = sp->sample_grid_size();
                if (grid_size_raw != grid_size_eff) {
                    DRONE_LOG_WARN("[PathA] sample_grid_size {} out of [2,64] — clamped to {}",
                                   grid_size_raw, grid_size_eff);
                }
                DRONE_LOG_INFO(
                    "[PathA] CpuSemanticProjector sample grid: {}×{} (= {} probes/mask){}",
                    grid_size_eff, grid_size_eff, grid_size_eff * grid_size_eff,
                    (grid_size_raw == grid_size_eff) ? "" : " [clamped from cfg]");
                semantic_projector = std::move(sp);
                const float iou    = ctx.cfg.get<float>(
                    drone::cfg_key::perception::path_a::MASK_CLASS_IOU_THRESHOLD, 0.5f);
                // Issue #645 — altitude filter on back-projected voxels.
                // Diagnostic on run 2026-04-30_174147 showed 22 % of voxels
                // above 6 m (sky misprojections) and 10 % below 0.3 m (ground
                // bias), totalling ~32 % of voxels per frame as guaranteed
                // ghosts.  Drop them at the projector boundary.  0 disables
                // the bound; default {0, 0} = legacy behaviour.
                drone::perception::MaskDepthProjector::AltitudeFilter alt{};
                alt.min_z_m = ctx.cfg.get<float>("perception.path_a.altitude_filter.min_z_m", 0.0f);
                alt.max_z_m = ctx.cfg.get<float>("perception.path_a.altitude_filter.max_z_m", 0.0f);
                // Issue #645 #659 — SAM mask size filter.  41-47 masks/frame
                // observed in scenario 33 for a scene with ~7 spawned
                // obstacles.  Most are tiny texture features (drop with
                // min_area_px) or huge sky/ground regions (drop with
                // max_area_px) that produce ghost voxels.
                drone::perception::MaskDepthProjector::MaskSizeFilter mask_size{};
                mask_size.min_area_px =
                    ctx.cfg.get<float>("perception.path_a.mask_size_filter.min_area_px", 0.0f);
                mask_size.max_area_px =
                    ctx.cfg.get<float>("perception.path_a.mask_size_filter.max_area_px", 0.0f);
                mask_projector = std::make_unique<drone::perception::MaskDepthProjector>(
                    *semantic_projector, iou, alt, mask_size);
                voxel_pub = ctx.bus.advertise<drone::ipc::SemanticVoxelBatch>(
                    drone::ipc::topics::SEMANTIC_VOXELS);
                if (!voxel_pub->is_ready()) {
                    DRONE_LOG_ERROR("[PathA] Failed to create publisher for {} — PATH A disabled",
                                    drone::ipc::topics::SEMANTIC_VOXELS);
                    sam_backend.reset();
                    mask_projector.reset();
                    semantic_projector.reset();
                    voxel_pub.reset();
                }
            }
        }

        if (sam_backend && mask_projector && voxel_pub) {
            DRONE_LOG_INFO("[PathA] Enabled — SAM: {}, projector: {}, publish: {}",
                           sam_backend->name(), semantic_projector->name(),
                           drone::ipc::topics::SEMANTIC_VOXELS);
        } else {
            DRONE_LOG_WARN(
                "[PathA] Requested via config but initialisation failed — running "
                "without PATH A (scenario behaviour degrades to PATH B / detector only)");
        }
    } else {
        DRONE_LOG_INFO("[Perception] PATH A disabled (perception.path_a.enabled=false)");
    }
    // PATH A requires the depth thread — MaskDepthProjector has nothing to
    // project without depth, so mask_projection_thread would skip forever.
    // Refuse to enable if depth is off rather than let the scenario run with
    // a silently-inert pipeline (same class of failure as #605).
    if (path_a_enabled && !depth_enabled) {
        DRONE_LOG_ERROR("[PathA] perception.path_a.enabled=true requires "
                        "perception.depth_estimator.enabled=true — PATH A disabled");
        sam_backend.reset();
        mask_projector.reset();
        semantic_projector.reset();
        voxel_pub.reset();
    }
    const bool path_a_active = path_a_enabled && depth_enabled && sam_backend && mask_projector &&
                               voxel_pub;

    // ── Drain timeout (Issue #446) — configurable, default 500ms ──
    const int drain_timeout_ms =
        std::clamp(ctx.cfg.get<int>(drone::cfg_key::perception::DRAIN_TIMEOUT_MS, 500), 50, 5000);

    // ── Internal triple buffers (lock-free latest-value handoff) ──
    drone::TripleBuffer<Detection2DList>   inference_to_tracker;
    drone::TripleBuffer<TrackedObjectList> tracker_to_fusion;
    // Depth map triple buffer — only used when depth estimator is enabled
    drone::TripleBuffer<drone::hal::DepthMap> depth_to_fusion;
    // PATH A fanout — inference + depth both fanout an extra copy so the
    // projection thread has its own single-consumer slot on each.
    drone::TripleBuffer<Detection2DList>      inference_to_projection;
    drone::TripleBuffer<Masks2DList>          sam_to_projection;
    drone::TripleBuffer<drone::hal::DepthMap> depth_to_projection;
    // Issue #645 — voxel-derived obstacle snapshot from mask_projection_thread
    // (producer) to fusion_thread (consumer).  Lock-free latest-value hand-off
    // so the fusion thread can append GEOMETRIC_OBSTACLE entries to its
    // DetectedObjectList publish (consumed by ObstacleAvoider3D).
    drone::TripleBuffer<drone::perception::VoxelObstacleSnapshot> voxel_obstacle_to_fusion;

    // ── Optional benchmark profiler (Epic #523, Issue #571) ──────────
    // Opt-in via config (benchmark.profiler.enabled). Disabled by default
    // so production builds pay zero overhead. When active, records per-stage
    // latency from the detector / tracker / fusion threads and dumps a JSON
    // summary to benchmark.profiler.output_dir on shutdown.
    //
    // DR-022 documents the priority-inversion / measurement-contamination
    // analysis that permits mutex-protected ScopedLatency calls on these
    // flight-critical threads (see docs/tracking/DESIGN_RATIONALE.md).
    const bool benchmark_profiler_enabled =
        ctx.cfg.get<bool>(drone::cfg_key::benchmark::PROFILER_ENABLED, false);
    std::optional<drone::util::LatencyProfiler> benchmark_profiler;
    if (benchmark_profiler_enabled) {
        benchmark_profiler.emplace();
        DRONE_LOG_INFO("[Benchmark] LatencyProfiler enabled — stages: Detect, Track, Fuse");
    }
    drone::util::LatencyProfiler* profiler_ptr = benchmark_profiler ? &*benchmark_profiler
                                                                    : nullptr;

    // ── Launch threads ──────────────────────────────────────
    std::thread t_inference(inference_thread, std::ref(*video_sub), std::ref(inference_to_tracker),
                            std::ref(g_shutdown_phase), std::ref(*detector), profiler_ptr,
                            path_a_active ? &inference_to_projection : nullptr);

    std::thread t_tracker(tracker_thread, std::ref(inference_to_tracker),
                          std::ref(tracker_to_fusion), std::ref(g_shutdown_phase),
                          std::ref(*tracker), drain_timeout_ms, profiler_ptr);

    // Subscribe to drone pose for the camera→world transform in the fusion thread
    auto pose_sub = ctx.bus.subscribe<drone::ipc::Pose>(drone::ipc::topics::SLAM_POSE);

    // Subscribe to radar detections for multi-sensor fusion
    auto radar_sub =
        ctx.bus.subscribe<drone::ipc::RadarDetectionList>(drone::ipc::topics::RADAR_DETECTIONS);

    const int fusion_rate_hz =
        std::clamp(ctx.cfg.get<int>(drone::cfg_key::perception::fusion::RATE_HZ, 30), 1, 100);

    // Issue #645 — gating for voxel-derived obstacles surfaced to the avoider.
    // min_observations matches the Phase 3 instance-promotion gate (#641) so the
    // avoider only sees obstacles that have already passed the noise filter for
    // grid promotion.  max_age_ms beyond track_max_age_s stops a flicker of a
    // stale snapshot — the tracker itself ages tracks out via wall-clock.
    const int voxel_obstacle_min_obs = std::clamp(
        ctx.cfg.get<int>("perception.path_a.avoider_surface.min_observations", 3), 1, 1000);
    const int voxel_obstacle_max_age_ms = std::clamp(
        ctx.cfg.get<int>("perception.path_a.avoider_surface.max_age_ms", 1000), 50, 60000);
    const uint64_t voxel_obstacle_max_age_ns = static_cast<uint64_t>(voxel_obstacle_max_age_ms) *
                                               1'000'000ULL;

    std::thread t_fusion(fusion_thread, std::ref(tracker_to_fusion), std::ref(*det_pub),
                         std::ref(*pose_sub), std::ref(*radar_sub), std::ref(g_shutdown_phase),
                         std::ref(*fusion_engine), fusion_rate_hz,
                         depth_enabled ? &depth_to_fusion : nullptr, drain_timeout_ms, profiler_ptr,
                         &voxel_obstacle_to_fusion, static_cast<uint32_t>(voxel_obstacle_min_obs),
                         voxel_obstacle_max_age_ns);

    // Launch depth estimation thread if HAL is active (Issue #430)
    // Subscriber must outlive the thread — declare in outer scope (same pattern as pose_sub, radar_sub)
    std::unique_ptr<drone::ipc::ISubscriber<drone::ipc::VideoFrame>> depth_video_sub;
    std::thread                                                      t_depth;
    if (depth_enabled && depth_estimator) {
        // Separate video subscriber for depth thread — don't share with inference
        depth_video_sub =
            ctx.bus.subscribe<drone::ipc::VideoFrame>(drone::ipc::topics::VIDEO_MISSION_CAM);
        // 0 = no rate limit (run as fast as model allows), consistent with detector max_fps
        const int depth_max_fps = std::clamp(
            ctx.cfg.get<int>(drone::cfg_key::perception::depth_estimator::MAX_FPS, 15), 0, 60);
        t_depth = std::thread(depth_thread, std::ref(*depth_video_sub), std::ref(depth_to_fusion),
                              std::ref(g_shutdown_phase), std::ref(*depth_estimator), depth_max_fps,
                              path_a_active ? &depth_to_projection : nullptr);
    }

    // Launch radar read thread if HAL is active and publisher is ready
    std::thread t_radar;
    if (radar && radar_pub && radar_pub->is_ready()) {
        t_radar = std::thread(radar_read_thread, std::ref(*radar), std::ref(*radar_pub),
                              std::ref(g_shutdown_phase), radar_update_rate_hz);
    } else if (radar) {
        DRONE_LOG_WARN("[Radar] HAL active but publisher not ready — radar read thread disabled");
    }

    // Launch PATH A threads (Epic #520, Issue #608) if enabled + initialised.
    // Each needs its own video subscriber (Zenoh SHM topology — see depth_video_sub pattern).
    //
    // IMPORTANT (PR #614 review): declaration order matters for exception-safety.
    // `path_a_trace` (the unique_ptr) MUST be declared BEFORE `t_sam` / `t_mask_proj`
    // so that at scope exit the threads destruct FIRST (std::thread dtor runs
    // std::terminate on a still-joinable thread, so they must already be joined
    // on the normal path — which we do at Phase 2 — but on an exception path
    // between construction and the join the dtor order still matters).
    // Declaring trace before the threads means: on scope exit, threads destruct
    // first, and *their* dtor is what enforces the join-or-terminate invariant.
    // The trace stays alive for the entire join window.
    std::unique_ptr<drone::ipc::ISubscriber<drone::ipc::VideoFrame>> sam_video_sub;
    std::unique_ptr<drone::ipc::ISubscriber<drone::ipc::Pose>>       pathA_pose_sub;
    std::unique_ptr<drone::util::PathATrace>                         path_a_trace;
    // Issue #638 Phase 2 — cross-frame voxel instance tracker.  Lifetime
    // matches the mask_projection thread's; declared here for the same
    // ordering reason as path_a_trace above.
    std::unique_ptr<drone::perception::VoxelInstanceTracker> path_a_tracker;
    std::thread                                              t_sam;
    std::thread                                              t_mask_proj;
    if (path_a_active) {
        sam_video_sub =
            ctx.bus.subscribe<drone::ipc::VideoFrame>(drone::ipc::topics::VIDEO_MISSION_CAM);
        pathA_pose_sub = ctx.bus.subscribe<drone::ipc::Pose>(drone::ipc::topics::SLAM_POSE);

        // Diagnostic voxel trace (Issue #612).  Opt-in via
        // perception.path_a.diag.trace_voxels; output path defaults under
        // drone_logs/ so scenario runs land trace next to the run's other logs.
        const bool trace_enabled =
            ctx.cfg.get<bool>(drone::cfg_key::perception::path_a::DIAG_TRACE_VOXELS, false);
        const std::string trace_path =
            ctx.cfg.get<std::string>(drone::cfg_key::perception::path_a::DIAG_TRACE_PATH,
                                     "drone_logs/path_a_voxel_trace.jsonl");
        path_a_trace = std::make_unique<drone::util::PathATrace>(trace_enabled, trace_path);

        // Issue #638 Phase 1 — voxel clustering knobs.  eps_m == 0 disables
        // (default), preserving today's behaviour.  Scenarios that opt in
        // override perception.path_a.cluster.eps_m / .min_pts.
        //
        // P2-D from PR #639 review: defensively reject NaN/Inf/negative
        // values from a tampered config — `cfg.get<float>` returns the raw
        // value (possibly NaN) and `assign_instance_ids` would otherwise
        // hit UB in the floor cast.  The clusterer's own guard catches it
        // too, but failing closed at config-read with a WARN gives the
        // operator visibility instead of silent disable.
        float cluster_eps_m_raw =
            ctx.cfg.get<float>(drone::cfg_key::perception::path_a::CLUSTER_EPS_M, 0.0f);
        int cluster_min_pts_raw =
            ctx.cfg.get<int>(drone::cfg_key::perception::path_a::CLUSTER_MIN_PTS, 3);
        float cluster_eps_m   = cluster_eps_m_raw;
        int   cluster_min_pts = cluster_min_pts_raw;
        if (!std::isfinite(cluster_eps_m) || cluster_eps_m < 0.0f) {
            DRONE_LOG_WARN("[VoxelCluster] Invalid cluster.eps_m {} — disabling clustering",
                           cluster_eps_m_raw);
            cluster_eps_m = 0.0f;
        }
        if (cluster_min_pts < 0) {
            DRONE_LOG_WARN("[VoxelCluster] Invalid cluster.min_pts {} — disabling clustering",
                           cluster_min_pts_raw);
            cluster_min_pts = 0;
        }
        if (cluster_eps_m > 0.0f) {
            DRONE_LOG_INFO("[VoxelCluster] enabled: eps={:.2f}m min_pts={}", cluster_eps_m,
                           cluster_min_pts);
        }

        // Issue #638 Phase 2 — cross-frame instance tracker.  Constructed
        // here so its track table persists across frames; only consulted
        // when clustering is enabled.
        //
        // P2-B/C/F from PR #640 review: NaN/Inf and out-of-range guards.
        // Without these, NaN config slips through `std::max(0.0f, NaN)`
        // (returns NaN), then track-distance comparisons return false for
        // every candidate → unbounded `tracks_` growth → DOS via memory
        // exhaustion.  An over-large `max_match_distance_m` would
        // conflate distinct obstacles across the visible world.
        constexpr float kMaxMatchDistanceCapM        = 50.0f;    // physical sensor sanity cap
        constexpr float kTrackMaxAgeCapS             = 3600.0f;  // 1 h (avoids uint64_t overflow)
        float           tracker_max_match_distance_m = ctx.cfg.get<float>(
            drone::cfg_key::perception::path_a::TRACKER_MAX_MATCH_DISTANCE_M, 3.0f);
        float tracker_track_max_age_s =
            ctx.cfg.get<float>(drone::cfg_key::perception::path_a::TRACKER_TRACK_MAX_AGE_S, 2.0f);
        if (!std::isfinite(tracker_max_match_distance_m) || tracker_max_match_distance_m < 0.0f ||
            tracker_max_match_distance_m > kMaxMatchDistanceCapM) {
            DRONE_LOG_WARN("[VoxelTracker] tracker.max_match_distance_m {} out of [0,{}] — "
                           "clamping to 3.0m",
                           tracker_max_match_distance_m, kMaxMatchDistanceCapM);
            tracker_max_match_distance_m = 3.0f;
        }
        if (!std::isfinite(tracker_track_max_age_s) || tracker_track_max_age_s < 0.0f ||
            tracker_track_max_age_s > kTrackMaxAgeCapS) {
            DRONE_LOG_WARN("[VoxelTracker] tracker.track_max_age_s {} out of [0,{}] — "
                           "clamping to 2.0s",
                           tracker_track_max_age_s, kTrackMaxAgeCapS);
            tracker_track_max_age_s = 2.0f;
        }
        path_a_tracker = std::make_unique<drone::perception::VoxelInstanceTracker>(
            tracker_max_match_distance_m, tracker_track_max_age_s);
        if (cluster_eps_m > 0.0f) {
            DRONE_LOG_INFO("[VoxelTracker] enabled: max_match={:.2f}m max_age={:.1f}s",
                           tracker_max_match_distance_m, tracker_track_max_age_s);
        }

        t_sam       = std::thread(sam_thread, std::ref(*sam_video_sub), std::ref(sam_to_projection),
                                  std::ref(g_shutdown_phase), std::ref(*sam_backend));
        t_mask_proj = std::thread(
            mask_projection_thread, std::ref(sam_to_projection), std::ref(inference_to_projection),
            std::ref(depth_to_projection), std::ref(*pathA_pose_sub), std::ref(*voxel_pub),
            std::ref(g_shutdown_phase), std::ref(*mask_projector), path_a_trace.get(),
            cluster_eps_m, cluster_min_pts, path_a_tracker.get(), &voxel_obstacle_to_fusion);
    }

    // ── Thread watchdog + health publisher ──────────────────
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub =
        ctx.bus.advertise<drone::ipc::ThreadHealth>(drone::ipc::topics::THREAD_HEALTH_PERCEPTION);
    drone::util::ThreadHealthPublisher health_publisher(*thread_health_pub, "perception", watchdog);

    DRONE_LOG_INFO("All perception threads started — READY");
    drone::systemd::notify_ready();

    // ── Main loop ───────────────────────────────────────────
    while (g_running.load(std::memory_order_acquire)) {
        drone::systemd::notify_watchdog();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        health_publisher.publish_snapshot();

        DRONE_LOG_INFO("[HealthCheck] perception alive");
    }

    // ── Phased shutdown (Issue #446) ───────────────────────
    // Signal handler sets g_running=false.  We translate that into a phased
    // drain sequence: stop inference first, let tracker drain, then fusion,
    // then auxiliary threads.  Each phase transition uses release ordering
    // so downstream threads see the updated phase with full memory visibility.
    //
    // notify_stopping() is called AFTER the profiler dump (if any) — that
    // way the dump happens while systemd still considers the service
    // "running" (fault-recovery review, PR #593), and any dump failure is
    // logged at full severity rather than during the narrower stop-timeout
    // window.
    DRONE_LOG_INFO("Shutting down — phased drain (timeout={}ms per stage)...", drain_timeout_ms);

    // Phase 1: stop inference + SAM (pipeline sources)
    g_shutdown_phase.store(1, std::memory_order_release);
    DRONE_LOG_INFO("[Shutdown] Phase 1 — stopping inference (+ SAM if active)");
    t_inference.join();
    if (t_sam.joinable()) t_sam.join();

    // Phase 2: stop tracker + mask projection (drain upstream buffers)
    g_shutdown_phase.store(2, std::memory_order_release);
    DRONE_LOG_INFO("[Shutdown] Phase 2 — stopping tracker (+ mask projection if active)");
    t_tracker.join();
    if (t_mask_proj.joinable()) t_mask_proj.join();

    // Phase 3: stop fusion, depth, radar (fusion drains tracker→fusion buffer)
    g_shutdown_phase.store(3, std::memory_order_release);
    DRONE_LOG_INFO("[Shutdown] Phase 3 — stopping fusion, depth, radar (draining)");
    t_fusion.join();
    if (t_depth.joinable()) t_depth.join();
    if (t_radar.joinable()) t_radar.join();

    // Phase 4: all stopped
    g_shutdown_phase.store(4, std::memory_order_release);

    // ── Benchmark profiler JSON dump (Issue #571 wiring) ──────
    // All worker threads have joined — the profiler's mutex is uncontended
    // and the captured window covers the full run. Dump before
    // notify_stopping() so we remain under the active-state watchdog and
    // the user (who explicitly enabled the feature) sees an ERROR if it
    // fails rather than a quiet WARN lost in the shutdown log noise.
    // DR-022 covers the broader flight-critical-thread analysis.
    if (benchmark_profiler) {
        const std::string output_dir = ctx.cfg.get<std::string>(
            drone::cfg_key::benchmark::PROFILER_OUTPUT_DIR, "drone_logs/benchmark");
        const std::filesystem::path path = std::filesystem::path(output_dir) /
                                           "latency_perception.json";
        const auto status = benchmark_profiler->dump_to_file(path);
        if (status == drone::util::LatencyProfiler::DumpStatus::Ok) {
            DRONE_LOG_INFO("[Benchmark] Wrote profiler snapshot → {}", path.string());
        } else {
            // Profiler was explicitly enabled — the user wanted this output;
            // losing it is a real failure, not a warning.
            DRONE_LOG_ERROR("[Benchmark] Profiler dump FAILED for {}: {}", path.string(),
                            drone::util::LatencyProfiler::describe(status));
        }
    }

    drone::systemd::notify_stopping();
    DRONE_LOG_INFO("=== Perception process stopped ===");
    return 0;
}
