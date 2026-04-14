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

# ── Sanitizer support ───────────────────────────────────────
# Usage: cmake -DENABLE_ASAN=ON, -DENABLE_TSAN=ON, or -DENABLE_UBSAN=ON
# ASan and TSan are mutually exclusive (both instrument memory accesses).
# Sanitizers require Debug or RelWithDebInfo for useful stack traces.
option(ENABLE_ASAN  "Enable Address Sanitizer (memory errors, leaks)"      OFF)
option(ENABLE_TSAN  "Enable Thread Sanitizer (data races)"                 OFF)
option(ENABLE_UBSAN "Enable Undefined Behavior Sanitizer"                  OFF)

# Validate mutual exclusivity
if(ENABLE_ASAN AND ENABLE_TSAN)
    message(FATAL_ERROR
        "ENABLE_ASAN and ENABLE_TSAN are mutually exclusive. "
        "ASan and TSan both instrument memory accesses and cannot be combined.")
endif()

# Apply sanitizer flags
if(ENABLE_ASAN)
    message(STATUS "  Sanitizer    : AddressSanitizer (ASan) ENABLED")
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls)
    add_link_options(-fsanitize=address)
endif()

if(ENABLE_TSAN)
    message(STATUS "  Sanitizer    : ThreadSanitizer (TSan) ENABLED")
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -fno-optimize-sibling-calls -Wno-tsan)
    add_link_options(-fsanitize=thread)
endif()

if(ENABLE_UBSAN)
    message(STATUS "  Sanitizer    : UndefinedBehaviorSanitizer (UBSan) ENABLED")
    add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=undefined)
endif()

if(NOT ENABLE_ASAN AND NOT ENABLE_TSAN AND NOT ENABLE_UBSAN)
    message(STATUS "  Sanitizer    : NONE")
endif()

# ── Code coverage support ───────────────────────────────────
# Usage: cmake -DENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug
# Requires gcov + lcov. Generate reports after running ctest:
#   lcov --capture --directory build --output-file coverage.info
#   lcov --remove coverage.info '/usr/*' --output-file coverage.info
#   genhtml coverage.info --output-directory coverage-report
option(ENABLE_COVERAGE "Enable gcov code coverage instrumentation" OFF)

if(ENABLE_COVERAGE)
    message(STATUS "  Coverage     : ENABLED (gcov)")
    add_compile_options(--coverage)
    add_link_options(--coverage)
else()
    message(STATUS "  Coverage     : OFF")
endif()

# ── Find system dependencies ────────────────────────────────
find_package(Threads       REQUIRED)
find_package(fmt           REQUIRED)
find_package(spdlog        REQUIRED)
find_package(Eigen3        REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)

# ── Optional: runtime plugin loading ────────────────────────
# When enabled, HAL factories gain a "plugin" backend that loads
# .so files at runtime via dlopen.  OFF by default — only enable
# in trusted environments (arbitrary .so loading = arbitrary code).
option(ENABLE_PLUGINS "Enable runtime plugin loading via dlopen" OFF)

if(ENABLE_PLUGINS)
    add_compile_definitions(HAVE_PLUGINS=1)
    message(STATUS "  Plugins      : ENABLED (dlopen)")
else()
    message(STATUS "  Plugins      : DISABLED")
endif()

# ── Optional: systemd integration ───────────────────────────
# When enabled, P7 (system_monitor) calls sd_notify() for watchdog support.
# Requires libsystemd-dev.  Harmless to enable on non-systemd systems at
# build time — the binary simply never receives WATCHDOG_USEC at runtime.
option(ENABLE_SYSTEMD "Build with systemd sd_notify() watchdog support" OFF)

if(ENABLE_SYSTEMD)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SYSTEMD REQUIRED libsystemd)
    add_compile_definitions(HAVE_SYSTEMD=1)
    message(STATUS "  systemd      : ENABLED (libsystemd ${SYSTEMD_VERSION})")
else()
    message(STATUS "  systemd      : DISABLED")
endif()

# ── Zenoh IPC backend (required) ────────────────────────────
# Zenoh is the sole IPC transport. The swappable IMessageBus interface
# allows adding future backends (iceoryx, DDS) without process changes.
#
# Security: pass -DZENOH_CONFIG_PATH=/path/to/zenoh.json5 for TLS+auth.
# Pass -DALLOW_INSECURE_ZENOH=ON to bypass (development/testing only).
set(ZENOH_CONFIG_PATH "" CACHE STRING
    "Path to a secure Zenoh configuration file (TLS + mutual auth recommended)")
option(ALLOW_INSECURE_ZENOH
    "Allow building without a secure Zenoh config (NOT RECOMMENDED)" OFF)

if(NOT ZENOH_CONFIG_PATH AND NOT ALLOW_INSECURE_ZENOH)
    message(FATAL_ERROR
        "No secure Zenoh configuration was provided.\n"
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
    add_compile_definitions(ZENOH_CONFIG_PATH="${ZENOH_CONFIG_PATH}")
endif()
message(STATUS "  Zenoh        : REQUIRED (zenohc ${zenohc_VERSION})")

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

# Try Gazebo Ionic (gz-transport14 / gz-msgs11) first, then Harmonic (13/10).
find_package(gz-transport14 QUIET)
find_package(gz-msgs11      QUIET)
if(gz-transport14_FOUND AND gz-msgs11_FOUND)
    set(GAZEBO_FOUND TRUE)
    set(GZ_TRANSPORT_VER 14)
    set(GZ_MSGS_VER 11)
    message(STATUS "  Gazebo libs  : gz-transport ${gz-transport14_VERSION}, gz-msgs ${gz-msgs11_VERSION} — Gazebo Ionic backends available")
else()
    find_package(gz-transport13 QUIET)
    find_package(gz-msgs10      QUIET)
    if(gz-transport13_FOUND AND gz-msgs10_FOUND)
        set(GAZEBO_FOUND TRUE)
        set(GZ_TRANSPORT_VER 13)
        set(GZ_MSGS_VER 10)
        message(STATUS "  Gazebo libs  : gz-transport ${gz-transport13_VERSION}, gz-msgs ${gz-msgs10_VERSION} — Gazebo Harmonic backends available")
    else()
        set(GAZEBO_FOUND FALSE)
        message(STATUS "  Gazebo libs  : NOT FOUND — Gazebo camera/IMU backends disabled")
    endif()
endif()

# ── Optional: Cosys-AirSim simulation backend ─────────────
find_package(AirSim QUIET)
if(AirSim_FOUND)
    set(COSYS_AIRSIM_FOUND TRUE)
    message(STATUS "  Cosys-AirSim : ${AirSim_VERSION} — Cosys-AirSim HAL backends available")
else()
    set(COSYS_AIRSIM_FOUND FALSE)
    message(STATUS "  Cosys-AirSim : NOT FOUND — Cosys-AirSim HAL backends disabled")
endif()

# ── ML model presets ───────────────────────────────────────
include(cmake/ModelPresets.cmake)
configure_file(config/model_defaults.h.in
    ${CMAKE_BINARY_DIR}/generated/model_defaults.h @ONLY)
include_directories(${CMAKE_BINARY_DIR}/generated)

# ── Shared interface libraries ───────────────────────────────
add_subdirectory(common/ipc)
add_subdirectory(common/util)
add_subdirectory(common/hal)
add_subdirectory(common/recorder)

# ── Per-process build options ────────────────────────────────
option(ENABLE_PROCESS_VIDEO_CAPTURE   "Build process1_video_capture"   ON)
option(ENABLE_PROCESS_PERCEPTION      "Build process2_perception"      ON)
option(ENABLE_PROCESS_SLAM_VIO_NAV    "Build process3_slam_vio_nav"    ON)
option(ENABLE_PROCESS_MISSION_PLANNER "Build process4_mission_planner" ON)
option(ENABLE_PROCESS_COMMS           "Build process5_comms"           ON)
option(ENABLE_PROCESS_PAYLOAD_MANAGER "Build process6_payload_manager" ON)
option(ENABLE_PROCESS_SYSTEM_MONITOR  "Build process7_system_monitor"  ON)

# ── Process executables ──────────────────────────────────────
if(ENABLE_PROCESS_VIDEO_CAPTURE)
    add_subdirectory(process1_video_capture)
endif()
if(ENABLE_PROCESS_PERCEPTION)
    add_subdirectory(process2_perception)
endif()
if(ENABLE_PROCESS_SLAM_VIO_NAV)
    add_subdirectory(process3_slam_vio_nav)
endif()
if(ENABLE_PROCESS_MISSION_PLANNER)
    add_subdirectory(process4_mission_planner)
endif()
if(ENABLE_PROCESS_COMMS)
    add_subdirectory(process5_comms)
endif()
if(ENABLE_PROCESS_PAYLOAD_MANAGER)
    add_subdirectory(process6_payload_manager)
endif()
if(ENABLE_PROCESS_SYSTEM_MONITOR)
    add_subdirectory(process7_system_monitor)
endif()

# ── Tools ────────────────────────────────────────────────────
add_subdirectory(tools/fault_injector)
add_subdirectory(tools/flight_replay)

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
message(STATUS "  Cosys-AirSim : ${COSYS_AIRSIM_FOUND}")
message(STATUS "  OpenCV       : ${OPENCV_FOUND}")
message(STATUS "  Zenoh        : REQUIRED")
message(STATUS "  Model Preset : ${MODEL_PRESET}")
message(STATUS "  Plugins      : ${ENABLE_PLUGINS}")
message(STATUS "  systemd      : ${ENABLE_SYSTEMD}")
message(STATUS "  ── Processes ──")
message(STATUS "  P1 video_capture   : ${ENABLE_PROCESS_VIDEO_CAPTURE}")
message(STATUS "  P2 perception      : ${ENABLE_PROCESS_PERCEPTION}")
message(STATUS "  P3 slam_vio_nav    : ${ENABLE_PROCESS_SLAM_VIO_NAV}")
message(STATUS "  P4 mission_planner : ${ENABLE_PROCESS_MISSION_PLANNER}")
message(STATUS "  P5 comms           : ${ENABLE_PROCESS_COMMS}")
message(STATUS "  P6 payload_manager : ${ENABLE_PROCESS_PAYLOAD_MANAGER}")
message(STATUS "  P7 system_monitor  : ${ENABLE_PROCESS_SYSTEM_MONITOR}")
message(STATUS "════════════════════════════════════════════")
