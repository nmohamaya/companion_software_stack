# ============================================================================
# CMakeLists.txt — Drone Companion Computer Software Stack (Super-Build)
# ============================================================================
# Builds 7 independent processes + 2 shared interface libraries.
#
# To build:
#   mkdir -p build && cd build
#   cmake -DCMAKE_BUILD_TYPE=Release ..
#   make -j$(nproc)
#
# All executables land in build/bin/
# ============================================================================

cmake_minimum_required(VERSION 3.16)
project(DroneCompanionStack
    VERSION 1.0.0
    LANGUAGES CXX
)

# ── Global settings ─────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Put all binaries in one place
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# ── Find system dependencies ────────────────────────────────
find_package(Threads       REQUIRED)
find_package(spdlog        REQUIRED)
find_package(Eigen3        REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)

# ── Optional: Zenoh IPC backend ─────────────────────────────
option(ENABLE_ZENOH "Build with Zenoh IPC backend (requires zenohc + zenoh-cpp)" OFF)

# Security: require a Zenoh config with TLS/auth for production builds.
# Pass -DZENOH_CONFIG_PATH=/path/to/zenoh.json5 for a secure config.
# Pass -DALLOW_INSECURE_ZENOH=ON to bypass (development/testing only).
set(ZENOH_CONFIG_PATH "" CACHE STRING
    "Path to a secure Zenoh configuration file (TLS + mutual auth recommended)")
option(ALLOW_INSECURE_ZENOH
    "Allow building Zenoh IPC backend without a secure config (NOT RECOMMENDED)" OFF)

if(ENABLE_ZENOH)
    if(NOT ZENOH_CONFIG_PATH AND NOT ALLOW_INSECURE_ZENOH)
        message(FATAL_ERROR
            "ENABLE_ZENOH is ON but no secure Zenoh configuration was provided.\n"
            "Set ZENOH_CONFIG_PATH to a config that enables transport encryption "
            "and authentication (e.g. TLS + mutual auth), or re-run CMake with "
            "-DALLOW_INSECURE_ZENOH=ON if you accept the risks (dev/test only).")
    elseif(NOT ZENOH_CONFIG_PATH AND ALLOW_INSECURE_ZENOH)
        message(WARNING
            "Building Zenoh IPC WITHOUT a secure configuration "
            "(ALLOW_INSECURE_ZENOH=ON). Traffic may be unencrypted/unauthenticated.")
    endif()

    find_package(zenohc REQUIRED)
    if(ZENOH_CONFIG_PATH)
        add_compile_definitions(HAVE_ZENOH=1 ZENOH_CONFIG_PATH="${ZENOH_CONFIG_PATH}")
    else()
        add_compile_definitions(HAVE_ZENOH=1)
    endif()
    message(STATUS "  Zenoh        : ENABLED (zenohc found) — ZenohMessageBus available")
else()
    message(STATUS "  Zenoh        : DISABLED (using POSIX SHM backend)")
endif()

# ── Optional dependencies (simulator / real-hardware backends) ──
find_package(MAVSDK QUIET)
if(MAVSDK_FOUND)
    message(STATUS "  MAVSDK       : ${MAVSDK_VERSION} — MavlinkFCLink backend available")
else()
    message(STATUS "  MAVSDK       : NOT FOUND — MavlinkFCLink backend disabled")
endif()

find_package(OpenCV QUIET COMPONENTS core imgproc dnn)
if(OpenCV_FOUND)
    set(OPENCV_FOUND TRUE)
    message(STATUS "  OpenCV       : ${OpenCV_VERSION} — YOLOv8 detector available")
else()
    set(OPENCV_FOUND FALSE)
    message(STATUS "  OpenCV       : NOT FOUND — YOLOv8 detector disabled")
endif()

find_package(gz-transport13 QUIET)
find_package(gz-msgs10      QUIET)
if(gz-transport13_FOUND AND gz-msgs10_FOUND)
    set(GAZEBO_FOUND TRUE)
    message(STATUS "  Gazebo libs  : gz-transport ${gz-transport13_VERSION}, gz-msgs ${gz-msgs10_VERSION} — Gazebo backends available")
else()
    set(GAZEBO_FOUND FALSE)
    message(STATUS "  Gazebo libs  : NOT FOUND — Gazebo camera/IMU backends disabled")
endif()

# ── Shared interface libraries ───────────────────────────────
add_subdirectory(common/ipc)
add_subdirectory(common/util)
add_subdirectory(common/hal)

# ── Process executables ──────────────────────────────────────
add_subdirectory(process1_video_capture)
add_subdirectory(process2_perception)
add_subdirectory(process3_slam_vio_nav)
add_subdirectory(process4_mission_planner)
add_subdirectory(process5_comms)
add_subdirectory(process6_payload_manager)
add_subdirectory(process7_system_monitor)

# ── Tests ────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)

# ── Summary ──────────────────────────────────────────────────
message(STATUS "════════════════════════════════════════════")
message(STATUS "  Drone Companion Stack v${PROJECT_VERSION}")
message(STATUS "  C++ Standard : C++${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type   : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Binaries     : ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "  MAVSDK       : ${MAVSDK_FOUND}")
message(STATUS "  Gazebo       : ${GAZEBO_FOUND}")
message(STATUS "  OpenCV       : ${OPENCV_FOUND}")
message(STATUS "  Zenoh        : ${ENABLE_ZENOH}")
message(STATUS "════════════════════════════════════════════")
