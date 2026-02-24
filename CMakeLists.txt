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
message(STATUS "════════════════════════════════════════════")
