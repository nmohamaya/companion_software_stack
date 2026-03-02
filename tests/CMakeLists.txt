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

# ── Fusion engine tests ──────────────────────────────────────
add_drone_test(test_fusion_engine   test_fusion_engine.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/fusion_engine.cpp
    ${PROJECT_SOURCE_DIR}/process2_perception/src/kalman_tracker.cpp
)

# ── Config system tests ──────────────────────────────────────
add_drone_test(test_config          test_config.cpp)

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

# ── Zenoh IPC + MessageBusFactory tests ──────────────────────
# Zenoh tests open sessions + allocate 32 MB SHM pools.  RESOURCE_LOCK
# serialises them so ctest -j never runs multiple Zenoh binaries at once.
add_drone_test(test_zenoh_ipc       test_zenoh_ipc.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")

# ── Phase E: Network transport tests ─────────────────────────
add_drone_test(test_zenoh_network   test_zenoh_network.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")

# ── Phase F: Liveliness token tests ──────────────────────────
add_drone_test(test_zenoh_liveliness test_zenoh_liveliness.cpp
    PROPERTIES RESOURCE_LOCK "zenoh_session")

# ── Process Strategy Interface tests ─────────────────────────
add_drone_test(test_process_interfaces test_process_interfaces.cpp)

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
