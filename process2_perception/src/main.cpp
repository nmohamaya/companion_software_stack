// process2_perception/src/main.cpp
// Process 2 — Perception: inference, tracking, fusion pipeline.
// Reads video frames from SHM, runs detection → tracking → fusion,
// publishes fused objects to SHM.

#include "hal/hal_factory.h"
#include "hal/iradar.h"
#include "ipc/ipc_types.h"
#include "ipc/message_bus_factory.h"
#include "ipc/zenoh_liveliness.h"
#include "perception/detector_interface.h"
#include "perception/fusion_engine.h"
#include "perception/ifusion_engine.h"
#include "perception/itracker.h"
#include "perception/kalman_tracker.h"
#include "perception/types.h"
#include "perception/ukf_fusion_engine.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/diagnostic.h"
#include "util/log_config.h"
#include "util/scoped_timer.h"
#include "util/sd_notify.h"
#include "util/signal_handler.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"
#include "util/triple_buffer.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

#include <spdlog/spdlog.h>

using namespace drone::perception;

static std::atomic<bool> g_running{true};

// ── Inference thread ────────────────────────────────────────
static void inference_thread(drone::ipc::ISubscriber<drone::ipc::VideoFrame>& video_sub,
                             drone::TripleBuffer<Detection2DList>&            output_queue,
                             std::atomic<bool>& running, IDetector& detector) {
    spdlog::info("[Inference] Thread started — using detector: {}", detector.name());

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
                spdlog::info("[Inference] Processed {} frames (writes={})", frame_count,
                             output_queue.write_count());
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    spdlog::info("[Inference] Thread stopped — {} frames, {} writes", frame_count,
                 output_queue.write_count());
}

// ── Tracker thread ──────────────────────────────────────────
static void tracker_thread(drone::TripleBuffer<Detection2DList>&   input_queue,
                           drone::TripleBuffer<TrackedObjectList>& output_queue,
                           std::atomic<bool>& running, ITracker& tracker) {
    spdlog::info("[Tracker] Thread started — backend: {}", tracker.name());

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
                spdlog::info("[Tracker] Status: backend={}, cycles={}, writes={}", tracker.name(),
                             cycle_count, output_queue.write_count());
            }

            if (diag.has_warnings() || diag.has_errors()) {
                diag.log_summary("Tracker");
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    spdlog::info("[Tracker] Thread stopped — {} cycles, {} writes", cycle_count,
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
                          std::atomic<bool>& running, IFusionEngine& engine, int fusion_rate_hz) {
    spdlog::info("[Fusion] Thread started — backend: {}, rate: {} Hz", engine.name(),
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

        // Drain latest pose (non-blocking — keep the most recent)
        {
            drone::ipc::Pose p{};
            while (pose_sub.receive(p)) {
                latest_pose = p;
                has_pose    = true;
            }
        }

        // Drain latest radar detections (non-blocking — keep the most recent)
        {
            drone::ipc::RadarDetectionList radar_list{};
            bool                           got_radar = false;
            while (radar_sub.receive(radar_list)) {
                got_radar = true;
            }
            if (got_radar) {
                engine.set_radar_detections(radar_list);
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

            // ── Camera-frame → World-frame transform ──────────────
            // fusion_engine returns position_3d in camera body frame:
            //   cam.x = forward (along boresight)
            //   cam.y = right
            //   cam.z = down
            // Our world frame: translation[0]=North, [1]=East, [2]=Up.
            // Drone heading = yaw extracted from quaternion (w,x,y,z).
            // Camera boresight = drone body +X = world North when yaw=0.
            //
            // Rotation (yaw only — ignore small pitch/roll for range estimation):
            //   world_north = cam.x * cos(yaw) - cam.y * sin(yaw)  + drone_north
            //   world_east  = cam.x * sin(yaw) + cam.y * cos(yaw)  + drone_east
            //   world_up    = drone_up - cam.z   (cam Z down = world -Z)
            if (has_pose) {
                // Extract yaw from quaternion (w, x, y, z)
                const double qw  = latest_pose.quaternion[0];
                const double qx  = latest_pose.quaternion[1];
                const double qy  = latest_pose.quaternion[2];
                const double qz  = latest_pose.quaternion[3];
                const float  yaw = static_cast<float>(
                    std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));
                const float cos_y = std::cos(yaw);
                const float sin_y = std::sin(yaw);

                const float dn = static_cast<float>(latest_pose.translation[0]);
                const float de = static_cast<float>(latest_pose.translation[1]);
                const float du = static_cast<float>(latest_pose.translation[2]);

                for (auto& obj : fused.objects) {
                    const float cx      = obj.position_3d.x();           // camera forward
                    const float cy      = obj.position_3d.y();           // camera right
                    const float cz      = obj.position_3d.z();           // camera down
                    obj.position_3d.x() = dn + cx * cos_y - cy * sin_y;  // world North
                    obj.position_3d.y() = de + cx * sin_y + cy * cos_y;  // world East
                    obj.position_3d.z() = du - cz;                       // world Up
                    // Rotate velocity the same way
                    const float vx      = obj.velocity_3d.x();
                    const float vy      = obj.velocity_3d.y();
                    obj.velocity_3d.x() = vx * cos_y - vy * sin_y;
                    obj.velocity_3d.y() = vx * sin_y + vy * cos_y;
                }
            }

            // Only publish once we have a valid pose — positions are meaningless
            // in camera frame and would produce incorrect avoidance reactions.
            if (!has_pose) {
                spdlog::debug("[Fusion] Skipping publish — no pose available yet");
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
                auto& src      = fused.objects[i];
                auto& dst      = shm_list.objects[i];
                dst.track_id   = src.track_id;
                dst.class_id   = static_cast<drone::ipc::ObjectClass>(src.class_id);
                dst.confidence = src.confidence;
                dst.position_x = src.position_3d.x();
                dst.position_y = src.position_3d.y();
                dst.position_z = src.position_3d.z();
                dst.velocity_x = src.velocity_3d.x();
                dst.velocity_y = src.velocity_3d.y();
                dst.velocity_z = src.velocity_3d.z();
                dst.heading    = src.heading;
                dst.has_camera = src.has_camera;
                dst.has_radar  = src.has_radar;
            }
            det_pub.publish(shm_list);
            ++fusion_count;

            if (diag.has_warnings() || diag.has_errors()) {
                diag.log_summary("Fusion");
            } else if (fusion_count % 100 == 0) {
                spdlog::info("[Fusion] {} cycles, {} fused objects this frame", fusion_count,
                             fused.objects.size());
            }
        }

        // Rate-limited sleep — consistent cadence regardless of work done
        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }
    spdlog::info("[Fusion] Thread stopped after {} cycles (reads={})", fusion_count,
                 tracked_queue.read_count());
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
        spdlog::warn("[Radar] Invalid update rate {} Hz — clamping to [{}, {}]", update_rate_hz,
                     kMinRateHz, kMaxRateHz);
        effective_rate_hz = std::clamp(effective_rate_hz, kMinRateHz, kMaxRateHz);
    }

    const auto period =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(1)) /
        effective_rate_hz;

    spdlog::info("[Radar] Read thread started — backend: {}, rate: {} Hz, period: {} ms",
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

    spdlog::info("[Radar] Read thread stopped — {} publishes", read_count);
}

// ═══════════════════════════════════════════════════════════
// main()
// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "perception");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("perception", LogConfig::resolve_log_dir(), args.log_level, args.json_logs);

    drone::Config cfg;
    if (!cfg.load(args.config_path)) {
        spdlog::warn("Running with default configuration; failed to load '{}'", args.config_path);
    }

    spdlog::info("=== Perception process starting (PID {}) ===", getpid());

    // ── Create message bus (config-driven: shm or zenoh) ───
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Declare liveliness token (auto-dropped on exit/crash) ──
    drone::ipc::LivelinessToken liveliness_token("perception");

    // ── Subscribe to video frames from Process 1 ────────────
    auto video_sub = bus.subscribe<drone::ipc::VideoFrame>(drone::ipc::topics::VIDEO_MISSION_CAM);
    if (!video_sub->is_connected()) {
        spdlog::error("Cannot connect to video channel — is video_capture running?");
        return 1;
    }

    // ── Create publisher for detected objects → Process 4 ───
    auto det_pub =
        bus.advertise<drone::ipc::DetectedObjectList>(drone::ipc::topics::DETECTED_OBJECTS);
    if (!det_pub->is_ready()) {
        spdlog::error("Failed to create publisher: {}", drone::ipc::topics::DETECTED_OBJECTS);
        return 1;
    }

    // ── Create detector from config ────────────────────────────
    std::string detector_backend = cfg.get<std::string>("perception.detector.backend", "simulated");
    auto        detector         = create_detector(detector_backend, &cfg);
    spdlog::info("[Perception] Detector backend: {} ({})", detector_backend, detector->name());

    // ── Create tracker from config ────────────────────────────
    std::string tracker_backend = cfg.get<std::string>("perception.tracker.backend", "sort");
    auto        tracker         = create_tracker(tracker_backend, &cfg);
    spdlog::info("[Perception] Tracker  backend: {} ({})", tracker_backend, tracker->name());

    // ── Create fusion engine from config ────────────────────
    CalibrationData calib;
    calib.camera_intrinsics         = Eigen::Matrix3f::Identity();
    calib.camera_intrinsics(0, 0)   = cfg.get<float>("perception.fusion.fx", 500.0f);
    calib.camera_intrinsics(1, 1)   = cfg.get<float>("perception.fusion.fy", 500.0f);
    calib.camera_intrinsics(0, 2)   = cfg.get<float>("perception.fusion.cx", 960.0f);
    calib.camera_intrinsics(1, 2)   = cfg.get<float>("perception.fusion.cy", 540.0f);
    calib.camera_height_m           = cfg.get<float>("perception.fusion.camera_height_m", 1.5f);
    calib.assumed_obstacle_height_m = cfg.get<float>("perception.fusion.assumed_obstacle_height_m",
                                                     3.0f);

    std::string fusion_backend = cfg.get<std::string>("perception.fusion.backend", "camera_only");
    auto        fusion_engine  = create_fusion_engine(fusion_backend, calib, &cfg);
    spdlog::info("[Perception] Fusion   backend: {} ({})", fusion_backend, fusion_engine->name());

    // ── Create radar HAL + publisher (optional) ────────────
    bool radar_enabled = cfg.get<bool>("perception.radar.enabled", false);
    std::unique_ptr<drone::hal::IRadar>                                     radar;
    std::unique_ptr<drone::ipc::IPublisher<drone::ipc::RadarDetectionList>> radar_pub;
    int radar_update_rate_hz = cfg.get<int>("perception.radar.update_rate_hz", 20);

    if (radar_enabled) {
        try {
            radar = drone::hal::create_radar(cfg, "perception.radar");
            if (!radar->init()) {
                spdlog::error("[Radar] HAL init() failed — radar disabled");
                radar.reset();
            } else {
                radar_pub = bus.advertise<drone::ipc::RadarDetectionList>(
                    drone::ipc::topics::RADAR_DETECTIONS);
                spdlog::info("[Perception] Radar HAL: {} — publishing to {}", radar->name(),
                             drone::ipc::topics::RADAR_DETECTIONS);
            }
        } catch (const std::exception& e) {
            spdlog::error("[Radar] Failed to create HAL backend: {} — radar disabled", e.what());
            radar.reset();
        }
    } else {
        spdlog::info("[Perception] Radar disabled (perception.radar.enabled=false)");
    }

    // ── Internal triple buffers (lock-free latest-value handoff) ──
    drone::TripleBuffer<Detection2DList>   inference_to_tracker;
    drone::TripleBuffer<TrackedObjectList> tracker_to_fusion;

    // ── Launch threads ──────────────────────────────────────
    std::thread t_inference(inference_thread, std::ref(*video_sub), std::ref(inference_to_tracker),
                            std::ref(g_running), std::ref(*detector));

    std::thread t_tracker(tracker_thread, std::ref(inference_to_tracker),
                          std::ref(tracker_to_fusion), std::ref(g_running), std::ref(*tracker));

    // Subscribe to drone pose for the camera→world transform in the fusion thread
    auto pose_sub = bus.subscribe<drone::ipc::Pose>(drone::ipc::topics::SLAM_POSE);

    // Subscribe to radar detections for multi-sensor fusion
    auto radar_sub =
        bus.subscribe<drone::ipc::RadarDetectionList>(drone::ipc::topics::RADAR_DETECTIONS);

    const int   fusion_rate_hz = std::clamp(cfg.get<int>("perception.fusion.rate_hz", 30), 1, 100);
    std::thread t_fusion(fusion_thread, std::ref(tracker_to_fusion), std::ref(*det_pub),
                         std::ref(*pose_sub), std::ref(*radar_sub), std::ref(g_running),
                         std::ref(*fusion_engine), fusion_rate_hz);

    // Launch radar read thread if HAL is active and publisher is ready
    std::thread t_radar;
    if (radar && radar_pub && radar_pub->is_ready()) {
        t_radar = std::thread(radar_read_thread, std::ref(*radar), std::ref(*radar_pub),
                              std::ref(g_running), radar_update_rate_hz);
    } else if (radar) {
        spdlog::warn("[Radar] HAL active but publisher not ready — radar read thread disabled");
    }

    // ── Thread watchdog + health publisher ──────────────────
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub =
        bus.advertise<drone::ipc::ThreadHealth>(drone::ipc::topics::THREAD_HEALTH_PERCEPTION);
    drone::util::ThreadHealthPublisher health_publisher(*thread_health_pub, "perception", watchdog);

    spdlog::info("All perception threads started — READY");
    drone::systemd::notify_ready();

    // ── Main loop ───────────────────────────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        drone::systemd::notify_watchdog();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        health_publisher.publish_snapshot();
        spdlog::info("[HealthCheck] perception alive");
    }

    drone::systemd::notify_stopping();
    spdlog::info("Shutting down...");
    t_inference.join();
    t_tracker.join();
    t_fusion.join();
    if (t_radar.joinable()) t_radar.join();

    spdlog::info("=== Perception process stopped ===");
    return 0;
}
