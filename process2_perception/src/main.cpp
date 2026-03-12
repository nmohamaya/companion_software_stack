// process2_perception/src/main.cpp
// Process 2 — Perception: inference, tracking, fusion pipeline.
// Reads video frames from SHM, runs detection → tracking → fusion,
// publishes fused objects to SHM.

#include "ipc/message_bus_factory.h"
#include "ipc/shm_types.h"
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
#include "util/spsc_ring.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <atomic>
#include <chrono>
#include <thread>

#include <spdlog/spdlog.h>

using namespace drone::perception;

static std::atomic<bool> g_running{true};

// ── Inference thread ────────────────────────────────────────
static void inference_thread(drone::ipc::ISubscriber<drone::ipc::ShmVideoFrame>& video_sub,
                             drone::SPSCRing<Detection2DList, 4>&                output_queue,
                             std::atomic<bool>& running, IDetector& detector) {
    spdlog::info("[Inference] Thread started — using detector: {}", detector.name());

    auto hb = drone::util::ScopedHeartbeat("inference", true);

    uint64_t frame_count      = 0;
    uint64_t queue_drop_count = 0;
    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        drone::ipc::ShmVideoFrame frame;
        bool                      got_frame = video_sub.receive(frame);

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

            if (!output_queue.try_push(std::move(det_list))) {
                ++queue_drop_count;
                diag.add_warning("Queue", "inference→tracker SPSC overflow (drop #" +
                                              std::to_string(queue_drop_count) + ")");
            }
            ++frame_count;

            if (diag.has_warnings() || diag.has_errors()) {
                diag.log_summary("Inference");
            } else if (frame_count % 100 == 0) {
                spdlog::info("[Inference] Processed {} frames ({} queue drops)", frame_count,
                             queue_drop_count);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    spdlog::info("[Inference] Thread stopped — {} frames, {} queue drops", frame_count,
                 queue_drop_count);
}

// ── Tracker thread ──────────────────────────────────────────
static void tracker_thread(drone::SPSCRing<Detection2DList, 4>&   input_queue,
                           drone::SPSCRing<TrackedObjectList, 4>& output_queue,
                           std::atomic<bool>& running, ITracker& tracker) {
    spdlog::info("[Tracker] Thread started — backend: {}", tracker.name());

    auto hb = drone::util::ScopedHeartbeat("tracker", true);

    uint64_t cycle_count      = 0;
    uint64_t queue_drop_count = 0;
    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        auto det_opt = input_queue.try_pop();
        if (det_opt) {
            drone::util::FrameDiagnostics diag(det_opt->frame_sequence);

            auto tracked = [&]() {
                drone::util::ScopedDiagTimer timer(diag, "Track");
                return tracker.update(*det_opt);
            }();

            diag.add_metric("Track", "n_tracks", static_cast<double>(tracked.objects.size()));

            if (!tracked.objects.empty()) {
                if (!output_queue.try_push(std::move(tracked))) {
                    ++queue_drop_count;
                    diag.add_warning("Queue", "tracker→fusion SPSC overflow (drop #" +
                                                  std::to_string(queue_drop_count) + ")");
                }
            }
            ++cycle_count;

            if (diag.has_warnings() || diag.has_errors()) {
                diag.log_summary("Tracker");
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    spdlog::info("[Tracker] Thread stopped — {} cycles, {} queue drops", cycle_count,
                 queue_drop_count);
}

// ── Fusion thread ───────────────────────────────────────────
// pose_sub  — IPC subscriber for drone/slam/pose (ShmPose).
//             Used to rotate camera-frame detections into world frame.
static void fusion_thread(drone::SPSCRing<TrackedObjectList, 4>&                     tracked_queue,
                          drone::ipc::IPublisher<drone::ipc::ShmDetectedObjectList>& det_pub,
                          drone::ipc::ISubscriber<drone::ipc::ShmPose>&              pose_sub,
                          std::atomic<bool>& running, IFusionEngine& engine) {
    spdlog::info("[Fusion] Thread started — backend: {}", engine.name());

    auto hb = drone::util::ScopedHeartbeat("fusion", true);

    uint64_t fusion_count = 0;

    // Cached latest drone pose (world frame: North, East, Up)
    drone::ipc::ShmPose latest_pose{};
    bool                has_pose = false;

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());

        // Drain latest pose (non-blocking — keep the most recent)
        {
            drone::ipc::ShmPose p{};
            while (pose_sub.receive(p)) {
                latest_pose = p;
                has_pose    = true;
            }
        }

        // Fuse when tracking data arrives
        if (auto topt = tracked_queue.try_pop()) {
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
            drone::ipc::ShmDetectedObjectList shm_list{};
            shm_list.timestamp_ns   = fused.timestamp_ns;
            shm_list.frame_sequence = fused.frame_sequence;
            shm_list.num_objects =
                std::min(static_cast<uint32_t>(fused.objects.size()),
                         static_cast<uint32_t>(drone::ipc::MAX_DETECTED_OBJECTS));

            for (uint32_t i = 0; i < shm_list.num_objects; ++i) {
                auto& src       = fused.objects[i];
                auto& dst       = shm_list.objects[i];
                dst.track_id    = src.track_id;
                dst.class_id    = static_cast<drone::ipc::ObjectClass>(src.class_id);
                dst.confidence  = src.confidence;
                dst.position_x  = src.position_3d.x();
                dst.position_y  = src.position_3d.y();
                dst.position_z  = src.position_3d.z();
                dst.velocity_x  = src.velocity_3d.x();
                dst.velocity_y  = src.velocity_3d.y();
                dst.velocity_z  = src.velocity_3d.z();
                dst.heading     = src.heading;
                dst.has_camera  = src.has_camera;
                dst.has_thermal = src.has_thermal;
            }
            det_pub.publish(shm_list);
            ++fusion_count;

            if (diag.has_warnings() || diag.has_errors()) {
                diag.log_summary("Fusion");
            } else if (fusion_count % 100 == 0) {
                spdlog::info("[Fusion] {} cycles, {} fused objects this frame", fusion_count,
                             fused.objects.size());
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    spdlog::info("[Fusion] Thread stopped after {} cycles", fusion_count);
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
    auto video_sub =
        bus.subscribe<drone::ipc::ShmVideoFrame>(drone::ipc::shm_names::VIDEO_MISSION_CAM);
    if (!video_sub->is_connected()) {
        spdlog::error("Cannot connect to video channel — is video_capture running?");
        return 1;
    }

    // ── Create publisher for detected objects → Process 4 ───
    auto det_pub =
        bus.advertise<drone::ipc::ShmDetectedObjectList>(drone::ipc::shm_names::DETECTED_OBJECTS);
    if (!det_pub->is_ready()) {
        spdlog::error("Failed to create publisher: {}", drone::ipc::shm_names::DETECTED_OBJECTS);
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

    // ── Internal SPSC queues ────────────────────────────────
    drone::SPSCRing<Detection2DList, 4>   inference_to_tracker;
    drone::SPSCRing<TrackedObjectList, 4> tracker_to_fusion;

    // ── Launch threads ──────────────────────────────────────
    std::thread t_inference(inference_thread, std::ref(*video_sub), std::ref(inference_to_tracker),
                            std::ref(g_running), std::ref(*detector));

    std::thread t_tracker(tracker_thread, std::ref(inference_to_tracker),
                          std::ref(tracker_to_fusion), std::ref(g_running), std::ref(*tracker));

    // Subscribe to drone pose for the camera→world transform in the fusion thread
    auto pose_sub = bus.subscribe<drone::ipc::ShmPose>(drone::ipc::shm_names::SLAM_POSE);

    std::thread t_fusion(fusion_thread, std::ref(tracker_to_fusion), std::ref(*det_pub),
                         std::ref(*pose_sub), std::ref(g_running), std::ref(*fusion_engine));

    // ── Thread watchdog + health publisher ──────────────────
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub =
        bus.advertise<drone::ipc::ShmThreadHealth>(drone::ipc::shm_names::THREAD_HEALTH_PERCEPTION);
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

    spdlog::info("=== Perception process stopped ===");
    return 0;
}
