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
    if(ARG_PROPERTIES)
        gtest_discover_tests(${TEST_NAME} PROPERTIES ${ARG_PROPERTIES})
    else()
        gtest_discover_tests(${TEST_NAME})
    endif()
endfunction()

# ── IPC tests (POSIX SHM — serialise to avoid /dev/shm collisions) ───
add_drone_test(test_shm_ipc         test_shm_ipc.cpp
    PROPERTIES RESOURCE_LOCK "posix_shm")

# ── SPSC Ring tests ──────────────────────────────────────────
add_drone_test(test_spsc_ring       test_spsc_ring.cpp)

# ── Kalman tracker tests ─────────────────────────────────────
add_drone_test(test_kalman_tracker  test_kalman_tracker.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/kalman_tracker.cpp
)

# ── Mission FSM tests ────────────────────────────────────────
add_drone_test(test_mission_fsm     test_mission_fsm.cpp)

# ── Fault Manager tests ─────────────────────────────────────
add_drone_test(test_fault_manager   test_fault_manager.cpp)

# ── Fusion engine tests ──────────────────────────────────────
add_drone_test(test_fusion_engine   test_fusion_engine.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/fusion_engine.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/ukf_fusion_engine.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/kalman_tracker.cpp
)

# ── Config system tests ──────────────────────────────────────
add_drone_test(test_config          test_config.cpp)

# ── Config validator tests ───────────────────────────────────
add_drone_test(test_config_validator test_config_validator.cpp)

# ── Result<T,E> type tests ───────────────────────────────────
add_drone_test(test_result          test_result.cpp)

# ── LatencyTracker tests ─────────────────────────────────────
add_drone_test(test_latency_tracker test_latency_tracker.cpp)

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
