# tests/CMakeLists.txt — Unit & integration tests using Google Test
find_package(GTest REQUIRED)
include(GoogleTest)

# Fix Anaconda libstdc++ conflict during test discovery
set(CMAKE_CROSSCOMPILING_EMULATOR "env" "LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu")

# ── Helper: create a test target ─────────────────────────────
function(add_drone_test TEST_NAME)
    add_executable(${TEST_NAME} ${ARGN})
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
    gtest_discover_tests(${TEST_NAME})
endfunction()

# ── IPC tests ────────────────────────────────────────────────
add_drone_test(test_shm_ipc         test_shm_ipc.cpp)

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

# ── Message Bus / API Interface tests ────────────────────────
add_drone_test(test_message_bus     test_message_bus.cpp)

# ── Process Strategy Interface tests ─────────────────────────
add_drone_test(test_process_interfaces test_process_interfaces.cpp)
