// process2_perception/src/main.cpp
// Process 2 — Perception: inference, tracking, fusion pipeline.
// Reads video frames from SHM, runs detection → tracking → fusion,
// publishes fused objects to SHM.

#include "hal/hal_factory.h"
#include "hal/idepth_estimator.h"
#include "hal/iradar.h"
#include "ipc/ipc_types.h"
#include "perception/detector_interface.h"
#include "perception/fusion_engine.h"
#include "perception/ifusion_engine.h"
#include "perception/itracker.h"
#include "perception/kalman_tracker.h"
#include "perception/types.h"
#include "perception/ukf_fusion_engine.h"
#include "util/config_keys.h"
#include "util/diagnostic.h"
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
#include <thread>

#include <Eigen/Geometry>  // Quaternionf for full-quaternion transform (#421)

using namespace drone::perception;

static std::atomic<bool> g_running{true};

// ── Inference thread ────────────────────────────────────────
static void inference_thread(drone::ipc::ISubscriber<drone::ipc::VideoFrame>& video_sub,
                             drone::TripleBuffer<Detection2DList>&            output_queue,
                             std::atomic<bool>& running, IDetector& detector) {
    DRONE_LOG_INFO("[Inference] Thread started — using detector: {}", detector.name());

    auto hb = drone::util::ScopedHeartbeat("inference", true);

    uint64_t frame_count = 0;
    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        drone::ipc::VideoFrame frame;
        bool                   got_frame = video_sub.receive(frame);

        if (got_frame) {
            drone::util::FrameDiagnostics diag(frame.sequence_number);

            auto dets = [&]() {
                drone::util::ScopedDiagTimer timer(diag, "Detect");
                return detector.detect(frame.pixel_data, frame.width, frame.height, frame.channels);
            }();

            Detection2DList det_list;
            det_list.detections     = std::move(dets);
            det_list.timestamp_ns   = frame.timestamp_ns;
            det_list.frame_sequence = frame.sequence_number;

            diag.add_metric("Detect", "n_detections",
                            static_cast<double>(det_list.detections.size()));

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

// ── Tracker thread ──────────────────────────────────────────
static void tracker_thread(drone::TripleBuffer<Detection2DList>&   input_queue,
                           drone::TripleBuffer<TrackedObjectList>& output_queue,
                           std::atomic<bool>& running, ITracker& tracker) {
    DRONE_LOG_INFO("[Tracker] Thread started — backend: {}", tracker.name());

    auto hb = drone::util::ScopedHeartbeat("tracker", true);

    constexpr uint64_t kStatusInterval = 50;  // Log tracker status every N cycles
    uint64_t           cycle_count     = 0;
    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        auto det_opt = input_queue.read();
        if (det_opt) {
            drone::util::FrameDiagnostics diag(det_opt->frame_sequence);

            auto tracked = [&]() {
                drone::util::ScopedDiagTimer timer(diag, "Track");
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
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
static void fusion_thread(drone::TripleBuffer<TrackedObjectList>&                  tracked_queue,
                          drone::ipc::IPublisher<drone::ipc::DetectedObjectList>&  det_pub,
                          drone::ipc::ISubscriber<drone::ipc::Pose>&               pose_sub,
                          drone::ipc::ISubscriber<drone::ipc::RadarDetectionList>& radar_sub,
                          std::atomic<bool>& running, IFusionEngine& engine, int fusion_rate_hz,
                          drone::TripleBuffer<drone::hal::DepthMap>* depth_buf) {
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

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());

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
                drone::util::ScopedDiagTimer timer(diag, "Fuse");
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
static void depth_thread(drone::ipc::ISubscriber<drone::ipc::VideoFrame>& video_sub,
                         drone::TripleBuffer<drone::hal::DepthMap>&       output_queue,
                         std::atomic<bool>& running, drone::hal::IDepthEstimator& estimator,
                         int max_fps) {
    DRONE_LOG_INFO("[Depth] Thread started — backend: {}, max_fps: {}", estimator.name(), max_fps);

    auto hb = drone::util::ScopedHeartbeat("depth", true);

    // Rate limiting — avoid running faster than the model can process
    const auto min_period = (max_fps > 0) ? std::chrono::milliseconds(1000 / max_fps)
                                          : std::chrono::milliseconds(0);
    auto       last_run   = std::chrono::steady_clock::now();

    uint64_t frame_count = 0;
    uint64_t fail_count  = 0;
    while (running.load(std::memory_order_relaxed)) {
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
static void radar_read_thread(drone::hal::IRadar&                                     radar,
                              drone::ipc::IPublisher<drone::ipc::RadarDetectionList>& radar_pub,
                              std::atomic<bool>& running, int update_rate_hz) {
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

    while (running.load(std::memory_order_relaxed)) {
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

    // ── Internal triple buffers (lock-free latest-value handoff) ──
    drone::TripleBuffer<Detection2DList>   inference_to_tracker;
    drone::TripleBuffer<TrackedObjectList> tracker_to_fusion;
    // Depth map triple buffer — only used when depth estimator is enabled
    drone::TripleBuffer<drone::hal::DepthMap> depth_to_fusion;

    // ── Launch threads ──────────────────────────────────────
    std::thread t_inference(inference_thread, std::ref(*video_sub), std::ref(inference_to_tracker),
                            std::ref(g_running), std::ref(*detector));

    std::thread t_tracker(tracker_thread, std::ref(inference_to_tracker),
                          std::ref(tracker_to_fusion), std::ref(g_running), std::ref(*tracker));

    // Subscribe to drone pose for the camera→world transform in the fusion thread
    auto pose_sub = ctx.bus.subscribe<drone::ipc::Pose>(drone::ipc::topics::SLAM_POSE);

    // Subscribe to radar detections for multi-sensor fusion
    auto radar_sub =
        ctx.bus.subscribe<drone::ipc::RadarDetectionList>(drone::ipc::topics::RADAR_DETECTIONS);

    const int fusion_rate_hz =
        std::clamp(ctx.cfg.get<int>(drone::cfg_key::perception::fusion::RATE_HZ, 30), 1, 100);
    std::thread t_fusion(fusion_thread, std::ref(tracker_to_fusion), std::ref(*det_pub),
                         std::ref(*pose_sub), std::ref(*radar_sub), std::ref(g_running),
                         std::ref(*fusion_engine), fusion_rate_hz,
                         depth_enabled ? &depth_to_fusion : nullptr);

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
                              std::ref(g_running), std::ref(*depth_estimator), depth_max_fps);
    }

    // Launch radar read thread if HAL is active and publisher is ready
    std::thread t_radar;
    if (radar && radar_pub && radar_pub->is_ready()) {
        t_radar = std::thread(radar_read_thread, std::ref(*radar), std::ref(*radar_pub),
                              std::ref(g_running), radar_update_rate_hz);
    } else if (radar) {
        DRONE_LOG_WARN("[Radar] HAL active but publisher not ready — radar read thread disabled");
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

    drone::systemd::notify_stopping();
    DRONE_LOG_INFO("Shutting down...");
    t_inference.join();
    t_tracker.join();
    t_fusion.join();
    if (t_depth.joinable()) t_depth.join();
    if (t_radar.joinable()) t_radar.join();

    DRONE_LOG_INFO("=== Perception process stopped ===");
    return 0;
}
