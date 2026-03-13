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
find_package(spdlog        REQUIRED)
find_package(Eigen3        REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)

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

# ── Tools ────────────────────────────────────────────────────
add_subdirectory(tools/fault_injector)

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
message(STATUS "  Zenoh        : REQUIRED")
message(STATUS "  systemd      : ${ENABLE_SYSTEMD}")
message(STATUS "════════════════════════════════════════════")
