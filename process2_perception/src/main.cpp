// process2_perception/src/main.cpp
// Process 2 — Perception: inference, tracking, fusion pipeline.
// Reads video frames from SHM, runs detection → tracking → fusion,
// publishes fused objects to SHM.

#include "ipc/message_bus_factory.h"
#include "ipc/shm_types.h"
#include "ipc/zenoh_liveliness.h"
#include "perception/detector_interface.h"
#include "perception/fusion_engine.h"
#include "perception/kalman_tracker.h"
#include "perception/types.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/log_config.h"
#include "util/scoped_timer.h"
#include "util/signal_handler.h"
#include "util/spsc_ring.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <atomic>
#include <chrono>
#include <random>
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

    uint64_t frame_count = 0;
    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        drone::ipc::ShmVideoFrame frame;
        bool                      got_frame = video_sub.receive(frame);

        if (got_frame) {
            ScopedTimer timer("Inference", 33.0);

            auto            dets = detector.detect(frame.pixel_data, frame.width, frame.height,
                                                   frame.channels);
            Detection2DList det_list;
            det_list.detections     = std::move(dets);
            det_list.timestamp_ns   = frame.timestamp_ns;
            det_list.frame_sequence = frame.sequence_number;

            output_queue.try_push(std::move(det_list));
            ++frame_count;

            if (frame_count % 100 == 0) {
                spdlog::info("[Inference] Processed {} frames, latest: {} dets", frame_count,
                             det_list.detections.size());
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    spdlog::info("[Inference] Thread stopped after {} frames", frame_count);
}

// ── Tracker thread ──────────────────────────────────────────
static void tracker_thread(drone::SPSCRing<Detection2DList, 4>&   input_queue,
                           drone::SPSCRing<TrackedObjectList, 4>& output_queue,
                           std::atomic<bool>&                     running) {
    spdlog::info("[Tracker] Thread started (SORT algorithm)");
    MultiObjectTracker tracker;

    auto hb = drone::util::ScopedHeartbeat("tracker", true);

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        auto det_opt = input_queue.try_pop();
        if (det_opt) {
            ScopedTimer timer("Tracker", 10.0);
            auto        tracked = tracker.update(*det_opt);

            if (!tracked.objects.empty()) {
                output_queue.try_push(std::move(tracked));
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    spdlog::info("[Tracker] Thread stopped");
}

// ── Simulated LiDAR thread ──────────────────────────────────
static void lidar_thread(drone::SPSCRing<std::vector<LiDARCluster>, 4>& output_queue,
                         std::atomic<bool>& running, const drone::Config& cfg) {
    spdlog::info("[LiDAR] Simulated LiDAR thread started");

    auto                                  hb = drone::util::ScopedHeartbeat("lidar", false);
    std::mt19937                          rng(12345);
    std::uniform_real_distribution<float> pos_dist(-20.0f, 20.0f);
    std::uniform_real_distribution<float> z_dist(0.0f, 3.0f);
    std::uniform_int_distribution<int>    n_dist(0, 4);

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        int                       n = n_dist(rng);
        std::vector<LiDARCluster> clusters;
        for (int i = 0; i < n; ++i) {
            LiDARCluster c;
            c.centroid = {pos_dist(rng), pos_dist(rng), z_dist(rng)};
            c.bbox_min = c.centroid - Eigen::Vector3f(1.0f, 0.8f, 0.8f);
            c.bbox_max = c.centroid + Eigen::Vector3f(1.0f, 0.8f, 0.8f);
            c.distance = c.centroid.norm();
            clusters.push_back(c);
        }
        output_queue.try_push(std::move(clusters));
        std::this_thread::sleep_for(std::chrono::milliseconds(
            cfg.get<int>("perception.lidar_rate_ms", 100)));  // configurable
    }
    spdlog::info("[LiDAR] Thread stopped");
}

// ── Simulated Radar thread ──────────────────────────────────
static void radar_thread(drone::SPSCRing<RadarDetectionList, 4>& output_queue,
                         std::atomic<bool>& running, const drone::Config& cfg) {
    spdlog::info("[Radar] Simulated radar thread started");

    auto                                  hb = drone::util::ScopedHeartbeat("radar", false);
    std::mt19937                          rng(67890);
    std::uniform_real_distribution<float> range_dist(5.0f, 50.0f);
    std::uniform_real_distribution<float> az_dist(-1.0f, 1.0f);
    std::uniform_real_distribution<float> vel_dist(-20.0f, 20.0f);
    std::uniform_int_distribution<int>    n_dist(0, 6);

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        RadarDetectionList rdl;
        rdl.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
        int n = n_dist(rng);
        for (int i = 0; i < n; ++i) {
            RadarDetection rd;
            rd.range           = range_dist(rng);
            rd.azimuth         = az_dist(rng);
            rd.elevation       = 0.0f;
            rd.velocity_radial = vel_dist(rng);
            rd.rcs             = 10.0f;
            rd.timestamp_ns    = rdl.timestamp_ns;
            rdl.detections.push_back(rd);
        }
        output_queue.try_push(std::move(rdl));
        std::this_thread::sleep_for(std::chrono::milliseconds(
            cfg.get<int>("perception.radar_rate_ms", 50)));  // configurable
    }
    spdlog::info("[Radar] Thread stopped");
}

// ── Fusion thread ───────────────────────────────────────────
static void fusion_thread(drone::SPSCRing<TrackedObjectList, 4>&                     tracked_queue,
                          drone::SPSCRing<std::vector<LiDARCluster>, 4>&             lidar_queue,
                          drone::SPSCRing<RadarDetectionList, 4>&                    radar_queue,
                          drone::ipc::IPublisher<drone::ipc::ShmDetectedObjectList>& det_pub,
                          std::atomic<bool>& running, const drone::Config& cfg) {
    spdlog::info("[Fusion] Thread started");

    auto hb = drone::util::ScopedHeartbeat("fusion", true);

    // Setup calibration (identity transforms for simulation)
    CalibrationData calib;
    calib.camera_intrinsics       = Eigen::Matrix3f::Identity();
    calib.camera_intrinsics(0, 0) = cfg.get<float>("perception.fusion.fx", 500.0f);
    calib.camera_intrinsics(1, 1) = cfg.get<float>("perception.fusion.fy", 500.0f);
    calib.camera_intrinsics(0, 2) = cfg.get<float>("perception.fusion.cx", 960.0f);
    calib.camera_intrinsics(1, 2) = cfg.get<float>("perception.fusion.cy", 540.0f);
    calib.T_cam_lidar             = Eigen::Matrix4f::Identity();
    calib.T_cam_radar             = Eigen::Matrix4f::Identity();

    FusionEngine engine(calib);

    std::vector<LiDARCluster> latest_lidar;
    RadarDetectionList        latest_radar;

    uint64_t fusion_count = 0;

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        // Consume latest LiDAR & radar
        if (auto lopt = lidar_queue.try_pop()) latest_lidar = std::move(*lopt);
        if (auto ropt = radar_queue.try_pop()) latest_radar = std::move(*ropt);

        // Fuse when tracking data arrives
        if (auto topt = tracked_queue.try_pop()) {
            ScopedTimer timer("Fusion", 15.0);

            auto fused = engine.fuse(*topt, latest_lidar, latest_radar);

            // Publish to SHM
            drone::ipc::ShmDetectedObjectList shm_list{};
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
                dst.has_lidar  = src.has_lidar;
                dst.has_radar  = src.has_radar;
            }
            det_pub.publish(shm_list);
            ++fusion_count;

            if (fusion_count % 100 == 0) {
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
    auto video_sub = drone::ipc::bus_subscribe<drone::ipc::ShmVideoFrame>(
        bus, drone::ipc::shm_names::VIDEO_MISSION_CAM);
    if (!video_sub->is_connected()) {
        spdlog::error("Cannot connect to video channel — is video_capture running?");
        return 1;
    }

    // ── Create publisher for detected objects → Process 4 ───
    auto det_pub = drone::ipc::bus_advertise<drone::ipc::ShmDetectedObjectList>(
        bus, drone::ipc::shm_names::DETECTED_OBJECTS);
    if (!det_pub->is_ready()) {
        spdlog::error("Failed to create publisher: {}", drone::ipc::shm_names::DETECTED_OBJECTS);
        return 1;
    }

    // ── Create detector from config ────────────────────────────
    std::string detector_backend = cfg.get<std::string>("perception.detector.backend", "simulated");
    auto        detector         = create_detector(detector_backend, &cfg);
    spdlog::info("[Perception] Detector backend: {} ({})", detector_backend, detector->name());

    // ── Internal SPSC queues ────────────────────────────────
    drone::SPSCRing<Detection2DList, 4>           inference_to_tracker;
    drone::SPSCRing<TrackedObjectList, 4>         tracker_to_fusion;
    drone::SPSCRing<std::vector<LiDARCluster>, 4> lidar_to_fusion;
    drone::SPSCRing<RadarDetectionList, 4>        radar_to_fusion;

    // ── Launch threads ──────────────────────────────────────
    std::thread t_inference(inference_thread, std::ref(*video_sub), std::ref(inference_to_tracker),
                            std::ref(g_running), std::ref(*detector));

    std::thread t_tracker(tracker_thread, std::ref(inference_to_tracker),
                          std::ref(tracker_to_fusion), std::ref(g_running));

    std::thread t_lidar(lidar_thread, std::ref(lidar_to_fusion), std::ref(g_running),
                        std::cref(cfg));

    std::thread t_radar(radar_thread, std::ref(radar_to_fusion), std::ref(g_running),
                        std::cref(cfg));

    std::thread t_fusion(fusion_thread, std::ref(tracker_to_fusion), std::ref(lidar_to_fusion),
                         std::ref(radar_to_fusion), std::ref(*det_pub), std::ref(g_running),
                         std::cref(cfg));

    // ── Thread watchdog + health publisher ──────────────────
    drone::util::ThreadWatchdog watchdog;
    auto thread_health_pub = drone::ipc::bus_advertise<drone::ipc::ShmThreadHealth>(
        bus, drone::ipc::shm_names::THREAD_HEALTH_PERCEPTION);
    drone::util::ThreadHealthPublisher health_publisher(*thread_health_pub, "perception", watchdog);

    spdlog::info("All perception threads started — READY");

    // ── Main loop ───────────────────────────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        health_publisher.publish_snapshot();
        spdlog::info("[HealthCheck] perception alive");
    }

    spdlog::info("Shutting down...");
    t_inference.join();
    t_tracker.join();
    t_lidar.join();
    t_radar.join();
    t_fusion.join();

    spdlog::info("=== Perception process stopped ===");
    return 0;
}
