# tests/CMakeLists.txt — Unit & integration tests using Google Test
find_package(GTest REQUIRED)
include(GoogleTest)

# Fix Anaconda libstdc++ conflict during test discovery
set(CMAKE_CROSSCOMPILING_EMULATOR "env" "LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu")

# ── Helper: create a test target ─────────────────────────────
# Usage:  add_drone_test(name source1 source2 ...
#                        [PROPERTIES key val ...])
# Any key/value pairs after PROPERTIES are forwarded to
# gtest_discover_tests() — handy for RESOURCE_LOCK, TIMEOUT, etc.
function(add_drone_test TEST_NAME)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "PROPERTIES")

    add_executable(${TEST_NAME} ${ARG_UNPARSED_ARGUMENTS})
    target_link_libraries(${TEST_NAME} PRIVATE
        GTest::gtest_main
        drone_ipc
        drone_util
        Eigen3::Eigen
        Threads::Threads
    )
    target_include_directories(${TEST_NAME} PRIVATE
        ${PROJECT_SOURCE_DIR}/process2_perception/include
        ${PROJECT_SOURCE_DIR}/process3_slam_vio_nav/include
        ${PROJECT_SOURCE_DIR}/process4_mission_planner/include
        ${PROJECT_SOURCE_DIR}/process5_comms/include
        ${PROJECT_SOURCE_DIR}/process6_payload_manager/include
        ${PROJECT_SOURCE_DIR}/process7_system_monitor/include
    )
    target_link_libraries(${TEST_NAME} PRIVATE drone_hal)
    # MockSysInfo and other test-only implementations are gated behind DRONE_ENABLE_MOCK
    target_compile_definitions(${TEST_NAME} PRIVATE DRONE_ENABLE_MOCK)
    if(ARG_PROPERTIES)
        gtest_discover_tests(${TEST_NAME} PROPERTIES ${ARG_PROPERTIES})
    else()
        gtest_discover_tests(${TEST_NAME})
    endif()
endfunction()

# ── SPSC Ring tests ──────────────────────────────────────────
add_drone_test(test_spsc_ring       test_spsc_ring.cpp)

# ── Triple Buffer tests ─────────────────────────────────────
add_drone_test(test_triple_buffer   test_triple_buffer.cpp)

# ── Kalman tracker tests ─────────────────────────────────────
if(ENABLE_PROCESS_PERCEPTION)
add_drone_test(test_kalman_tracker  test_kalman_tracker.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/kalman_tracker.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/bytetrack_tracker.cpp
)
endif()

# ── Mission FSM tests ────────────────────────────────────────
add_drone_test(test_mission_fsm     test_mission_fsm.cpp)

# ── Fault Manager tests ─────────────────────────────────────
add_drone_test(test_fault_manager   test_fault_manager.cpp)

# ── Geofence tests ──────────────────────────────────────────
add_drone_test(test_geofence        test_geofence.cpp)

# ── 3D Obstacle Avoider tests ───────────────────────────────
add_drone_test(test_obstacle_avoider_3d test_obstacle_avoider_3d.cpp)

# ── Fusion engine tests ──────────────────────────────────────
if(ENABLE_PROCESS_PERCEPTION)
add_drone_test(test_fusion_engine   test_fusion_engine.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/fusion_engine.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/ukf_fusion_engine.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/kalman_tracker.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/bytetrack_tracker.cpp
)
endif()

# ── World transform tests (Issue #421) ──────────────────────────
add_drone_test(test_world_transform  test_world_transform.cpp)

# ── Config system tests ──────────────────────────────────────
add_drone_test(test_config          test_config.cpp)
target_compile_definitions(test_config PRIVATE
    PROJECT_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)

# ── Config validator tests ───────────────────────────────────
add_drone_test(test_config_validator test_config_validator.cpp)
target_compile_definitions(test_config_validator PRIVATE
    PROJECT_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)

# ── IClock interface tests ───────────────────────────────────
add_drone_test(test_iclock           test_iclock.cpp)

# ── Result<T,E> type tests ───────────────────────────────────
add_drone_test(test_result          test_result.cpp)

# ── LatencyTracker tests ─────────────────────────────────────
add_drone_test(test_latency_tracker test_latency_tracker.cpp)

# ── LatencyProfiler tests (Issue #571 — Epic #523) ──────────
# Per-stage latency profiler used by the benchmark harness. Header-only,
# so no additional sources; lives in common/util alongside latency_tracker.
add_drone_test(test_latency_profiler test_latency_profiler.cpp)

# ── LatencyProfiler wiring smoke test (Issue #571 wiring PR) ─────
# Simulates the P2/P4 profiler-wiring pattern end-to-end: concurrent
# workers record via ScopedLatency, dump to JSON on disk, parse back.
# Guards the wiring contract without needing a full scenario run.
add_drone_test(test_latency_profiler_dump test_latency_profiler_dump.cpp)

# ── GT emitter tests (Issue #594 — Epic #523, PR 1 Cosys backend) ─
# Shared-layer tests only (GtClassMap + to_json_line). The Cosys backend
# itself needs a live AirSim server and is covered by scenario tests #29/#30.
# When built with HAVE_COSYS_AIRSIM, the Cosys .cpp is linked in and the
# factory resolves at link time; otherwise the backend .cpp is empty.
add_drone_test(test_gt_emitter
    test_gt_emitter.cpp
    benchmark/gt_emitter.cpp
    benchmark/cosys_gt_emitter.cpp
)
target_include_directories(test_gt_emitter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_definitions(test_gt_emitter PRIVATE
    PROJECT_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)

# ── Thread Heartbeat & Watchdog tests ────────────────────────
add_drone_test(test_thread_heartbeat test_thread_heartbeat.cpp)

# ── Thread Health Publisher tests ────────────────────────────
add_drone_test(test_thread_health_publisher test_thread_health_publisher.cpp)

# ── Comms tests ──────────────────────────────────────────────
add_drone_test(test_comms           test_comms.cpp)

# ── Payload Manager tests ────────────────────────────────────
add_drone_test(test_payload_manager test_payload_manager.cpp)

# ── System Monitor tests ─────────────────────────────────────
add_drone_test(test_system_monitor  test_system_monitor.cpp)

# ── HAL tests ────────────────────────────────────────────────
add_drone_test(test_hal             test_hal.cpp)

# ── HAL Camera Lifetime tests (Issue #452) ───────────────────
add_drone_test(test_hal_camera_lifetime test_hal_camera_lifetime.cpp)

# ── MavlinkFCLink tests ─────────────────────────────────────
add_drone_test(test_mavlink_fc_link test_mavlink_fc_link.cpp)

# ── GazeboCameraBackend tests ───────────────────────────────
add_drone_test(test_gazebo_camera   test_gazebo_camera.cpp)

# ── GazeboIMUBackend tests ──────────────────────────────────
add_drone_test(test_gazebo_imu      test_gazebo_imu.cpp)

# ── Message Bus / API Interface tests ────────────────────────
add_drone_test(test_message_bus     test_message_bus.cpp)

# ── JSON logging sink tests ──────────────────────────────────
add_drone_test(test_json_log_sink   test_json_log_sink.cpp)

# ── ILogger interface tests ─────────────────────────────────
add_drone_test(test_ilogger         test_ilogger.cpp)

# ── Correlation ID tests ─────────────────────────────────────
add_drone_test(test_correlation     test_correlation.cpp)

# ── Zenoh IPC + MessageBusFactory tests ──────────────────────
# Zenoh tests open sessions + allocate 32 MB SHM pools.  RESOURCE_LOCK
# serialises them so ctest -j never runs multiple Zenoh binaries at once.
add_drone_test(test_zenoh_ipc       test_zenoh_ipc.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")

# ── Phase E: Network transport tests ─────────────────────────
add_drone_test(test_zenoh_network   test_zenoh_network.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")

# ── Test helper binary: test_crasher ─────────────────────────
# Minimal binary used by ProcessManager tests. Not a test itself,
# just a controlled child process that exits/crashes/hangs on command.
add_executable(test_crasher test_crasher.cpp)
target_compile_features(test_crasher PRIVATE cxx_std_17)
# Disable sanitizers for test_crasher: it intentionally raises SIGSEGV ("crash" mode)
# and sanitizers intercept the signal as a DEADLYSIGNAL, changing exit behavior.
if(ENABLE_ASAN OR ENABLE_TSAN OR ENABLE_UBSAN)
    target_compile_options(test_crasher PRIVATE -fno-sanitize=all)
    target_link_options(test_crasher PRIVATE -fno-sanitize=all)
endif()

# ── Process Manager tests ────────────────────────────────────
# Fork+exec real processes — serialise to avoid PID/signal conflicts.
# ── Process Manager tests ────────────────────────────────
# Fork+exec real processes — serialise to avoid PID/signal conflicts.
add_drone_test(test_process_manager test_process_manager.cpp
    PROPERTIES RESOURCE_LOCK "fork_exec")
add_dependencies(test_process_manager test_crasher)

# ── Restart Policy tests (Phase 4) ──────────────────────────
add_drone_test(test_restart_policy  test_restart_policy.cpp)
target_compile_definitions(test_restart_policy PRIVATE
    PROJECT_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)

# ── Process Graph tests (Phase 4) ───────────────────────────
add_drone_test(test_process_graph   test_process_graph.cpp)

# ── Phase F: Liveliness token tests ──────────────────────────
add_drone_test(test_zenoh_liveliness test_zenoh_liveliness.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")

# ── Zenoh branch-coverage tests ─────────────────────────────
add_drone_test(test_zenoh_coverage  test_zenoh_coverage.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")

# ── Process Strategy Interface tests ─────────────────────────
add_drone_test(test_process_interfaces test_process_interfaces.cpp)

# ── systemd sd_notify wrapper tests ─────────────────────────
add_drone_test(test_sd_notify test_sd_notify.cpp)
if(ENABLE_SYSTEMD)
    target_include_directories(test_sd_notify PRIVATE ${SYSTEMD_INCLUDE_DIRS})
    target_link_libraries(test_sd_notify PRIVATE ${SYSTEMD_LIBRARIES})
endif()

# ── SimulatedDetector tests (Issue #253) ────────────────────
if(ENABLE_PROCESS_PERCEPTION)
add_drone_test(test_simulated_detector test_simulated_detector.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/simulated_detector.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/detector_factory.cpp
)

# ── ColorContourDetector tests ───────────────────────────────
add_drone_test(test_color_contour_detector test_color_contour_detector.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/simulated_detector.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/detector_factory.cpp
)

# ── OpenCV YOLOv8 detector tests ────────────────────────────
add_drone_test(test_opencv_yolo_detector test_opencv_yolo_detector.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/simulated_detector.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/detector_factory.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/opencv_yolo_detector.cpp
)
target_compile_definitions(test_opencv_yolo_detector PRIVATE
    YOLO_MODEL_PATH="${PROJECT_SOURCE_DIR}/models/yolov8n.onnx"
)
if(OPENCV_FOUND)
    target_link_libraries(test_opencv_yolo_detector PRIVATE ${OpenCV_LIBS})
    target_compile_definitions(test_opencv_yolo_detector PRIVATE HAS_OPENCV=1)
endif()

# ── Depth estimator + UKF depth fusion tests (Issue #430) ──
add_drone_test(test_depth_estimator  test_depth_estimator.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/fusion_engine.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/ukf_fusion_engine.cpp
)
endif()

# ── Depth Anything V2 backend tests (Issue #455) ───────────
# Requires OpenCV: the DA V2 .cpp is only compiled into drone_hal when OPENCV_FOUND.
if(OPENCV_FOUND)
    add_drone_test(test_depth_anything_v2 test_depth_anything_v2.cpp)
    target_compile_definitions(test_depth_anything_v2 PRIVATE
        DA_V2_MODEL_PATH="${PROJECT_SOURCE_DIR}/models/depth_anything_v2_vits.onnx"
        HAS_OPENCV=1
    )
    target_link_libraries(test_depth_anything_v2 PRIVATE ${OpenCV_LIBS})
endif()

# ── Cosys-AirSim HAL backend tests (Issue #434) ────────────
add_drone_test(test_cosys_hal_backends test_cosys_hal_backends.cpp)

# ── Cosys camera config-lookup tests (Issue #499, Bug #1) ──
# Verifies the per-section → top-level → default precedence ladder used by
# CosysCameraBackend. Whole test body is gated on HAVE_COSYS_AIRSIM (set as
# INTERFACE definition on drone_hal when the SDK is found); without the SDK
# the translation unit compiles to an empty TU, which is harmless.
add_drone_test(test_cosys_camera_config test_cosys_camera_config.cpp)

# ── Cosys-AirSim FC link tests (Issue #490) ───────────────
# SimpleFlight-via-AirSim-RPC backend — tests run without a live AirSim
# server; all command paths should return false when disconnected.
add_drone_test(test_cosys_fc_link test_cosys_fc_link.cpp)

# ── Diagnostic Collector tests ──────────────────────────────
add_drone_test(test_diagnostic      test_diagnostic.cpp)

# ── VIO Feature Extractor tests ─────────────────────────────
add_drone_test(test_feature_extractor test_feature_extractor.cpp)

# ── VIO Stereo Matcher tests ────────────────────────────────
add_drone_test(test_stereo_matcher  test_stereo_matcher.cpp)

# ── IMU Pre-integrator tests ────────────────────────────────
if(ENABLE_PROCESS_SLAM_VIO_NAV)
add_drone_test(test_imu_preintegrator test_imu_preintegrator.cpp
    ${PROJECT_SOURCE_DIR}/process3_slam_vio_nav/src/imu_preintegrator.cpp
)

# ── VIO Backend tests ───────────────────────────────────────
add_drone_test(test_vio_backend     test_vio_backend.cpp
    ${PROJECT_SOURCE_DIR}/process3_slam_vio_nav/src/imu_preintegrator.cpp
)
endif()

# ── Static Obstacle Layer tests (Issue #154) ───────────────
add_drone_test(test_static_obstacle_layer test_static_obstacle_layer.cpp)

# ── IPC Validation tests (Issues #179, #181, #185) ─────────
add_drone_test(test_ipc_validation test_ipc_validation.cpp)

# ── GCS Command Handler tests (Issue #154) ─────────────────
add_drone_test(test_gcs_command_handler test_gcs_command_handler.cpp)

# ── Fault Response Executor tests (Issue #154) ─────────────
add_drone_test(test_fault_response_executor test_fault_response_executor.cpp)

# ── ByteTrack Tracker tests (Issue #163) ────────────────────
if(ENABLE_PROCESS_PERCEPTION)
add_drone_test(test_bytetrack_tracker test_bytetrack_tracker.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/kalman_tracker.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/bytetrack_tracker.cpp
)
endif()

# ── D* Lite Path Planner tests (Issue #158) ────────────────
add_drone_test(test_dstar_lite_planner test_dstar_lite_planner.cpp)

# ── Dynamic Obstacle Prediction tests (Issue #256) ───────
add_drone_test(test_obstacle_prediction test_obstacle_prediction.cpp)

# ── Mission State Tick tests (Issue #154) ──────────────────
add_drone_test(test_mission_state_tick test_mission_state_tick.cpp)

# ── Radar HAL tests (Issue #209) ──────────────────────────
add_drone_test(test_radar_hal          test_radar_hal.cpp)

# ── Gazebo Radar Backend tests (Issue #212) ───────────────
add_drone_test(test_gazebo_radar       test_gazebo_radar.cpp)

# ── Rate Clamp tests (Issue #220) ─────────────────────────
add_drone_test(test_rate_clamp         test_rate_clamp.cpp)

# ── Flight Recorder tests (Issue #40) ─────────────────────
add_drone_test(test_flight_recorder    test_flight_recorder.cpp)
target_link_libraries(test_flight_recorder PRIVATE drone_recorder)
target_include_directories(test_flight_recorder PRIVATE
    ${PROJECT_SOURCE_DIR}/common/recorder/include
)
# ── Replay Dispatch tests (Issue #352) ───────────────────
add_drone_test(test_replay_dispatch    test_replay_dispatch.cpp)
target_link_libraries(test_replay_dispatch PRIVATE drone_recorder)
target_include_directories(test_replay_dispatch PRIVATE
    ${PROJECT_SOURCE_DIR}/common/recorder/include
)
# ── Collision Recovery tests (Issue #226) ─────────────────
add_drone_test(test_collision_recovery test_collision_recovery.cpp)
# ── Gimbal Auto-Tracker tests (Issue #257) ────────────────
add_drone_test(test_gimbal_auto_tracker test_gimbal_auto_tracker.cpp)

# ── TopicResolver tests (Issue #289) ─────────────────────
add_drone_test(test_topic_resolver  test_topic_resolver.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")

# ── ProcessContext tests (Issue #291) ────────────────────
add_drone_test(test_process_context test_process_context.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")
target_compile_definitions(test_process_context PRIVATE
    PROJECT_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)

# ── EventBus tests (Issue #293) ─────────────────────────
add_drone_test(test_event_bus       test_event_bus.cpp)

# ── ISerializer / RawSerializer tests (Issue #294) ──────
add_drone_test(test_serializer      test_serializer.cpp)

# ── PluginLoader tests (Issue #295 — Epic #284) ────────────
# Only built when ENABLE_PLUGINS=ON.  Builds a mock .so and
# loads it at runtime to verify PluginLoader / PluginHandle /
# PluginRegistry lifetime management.
if(ENABLE_PLUGINS)
    # Build the mock plugin as a shared library
    add_library(test_plugin_mock SHARED test_plugin_mock.cpp)
    target_include_directories(test_plugin_mock PRIVATE
        ${PROJECT_SOURCE_DIR}/common/hal/include
    )
    set_target_properties(test_plugin_mock PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    )

    # Build and register the test
    add_drone_test(test_plugin_loader test_plugin_loader.cpp)
    target_link_libraries(test_plugin_loader PRIVATE ${CMAKE_DL_LIBS})
    target_compile_definitions(test_plugin_loader PRIVATE
        TEST_PLUGIN_MOCK_PATH="${CMAKE_BINARY_DIR}/lib/test_plugin_mock.so"
    )
    add_dependencies(test_plugin_loader test_plugin_mock)
endif()

# ═══════════════════════════════════════════════════════════
# Integration tests (Issue #292 — Epic #284 Platform Modularity)
# ═══════════════════════════════════════════════════════════
# These tests use the IntegrationTestHarness (shared MessageBus,
# MockClock, CapturingLogger) to test multi-process IPC flows
# in a single test binary.  Zenoh RESOURCE_LOCK prevents parallel
# session exhaustion.

add_drone_test(test_integration_ipc_roundtrip
    integration/test_ipc_roundtrip.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")
target_include_directories(test_integration_ipc_roundtrip PRIVATE
    ${PROJECT_SOURCE_DIR}/tests
)
target_compile_definitions(test_integration_ipc_roundtrip PRIVATE
    PROJECT_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)

add_drone_test(test_integration_fault_cascade
    integration/test_fault_cascade.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")
target_include_directories(test_integration_fault_cascade PRIVATE
    ${PROJECT_SOURCE_DIR}/tests
)
target_compile_definitions(test_integration_fault_cascade PRIVATE
    PROJECT_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)

# ── Perception Pipeline Drain tests (Issue #446) ───────────
add_drone_test(test_perception_drain test_perception_drain.cpp)

# ── Model Presets + Config Overlay tests (Issue #433) ───────
add_drone_test(test_model_presets    test_model_presets.cpp)
target_compile_definitions(test_model_presets PRIVATE
    PROJECT_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
)

# ── Perception metrics framework tests (Issue #570 — Epic #523) ─
# Pure C++, no ML deps. Measures TP/FP/FN, per-class AP, MOTA/MOTP,
# confusion matrix. Used by the benchmark harness for regression gating.
add_drone_test(test_perception_metrics
    test_perception_metrics.cpp
    benchmark/perception_metrics.cpp
)
target_include_directories(test_perception_metrics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ── Baseline capture tests (Issue #573 — Epic #523) ─────────────
# Validates BaselineCapture metric accumulation, per-class breakdown,
# JSON round-trip, and latency passthrough. Links perception_metrics
# for TP/FP/FN computation (same lib the capture class delegates to).
add_drone_test(test_baseline_capture
    test_baseline_capture.cpp
    benchmark/baseline_capture.cpp
    benchmark/perception_metrics.cpp
)
target_include_directories(test_baseline_capture PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_baseline_capture PRIVATE
    nlohmann_json::nlohmann_json
)

# ── Baseline comparator tests (Issue #572 — Epic #523) ──────────
# Validates regression detection against configurable thresholds:
# recall, precision, mAP, MOTA, latency p95.
add_drone_test(test_baseline_comparator
    test_baseline_comparator.cpp
    benchmark/baseline_comparator.cpp
    benchmark/baseline_capture.cpp
    benchmark/perception_metrics.cpp
)
target_include_directories(test_baseline_comparator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_baseline_comparator PRIVATE
    nlohmann_json::nlohmann_json
)

# ── Baseline comparison CLI tool (Issue #572) ────────────────────
# Standalone executable: compare current run against stored baseline.
add_executable(compare_to_baseline
    benchmark/compare_to_baseline_main.cpp
    benchmark/baseline_comparator.cpp
    benchmark/baseline_capture.cpp
    benchmark/perception_metrics.cpp
)
target_include_directories(compare_to_baseline PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(compare_to_baseline PRIVATE
    nlohmann_json::nlohmann_json
)
