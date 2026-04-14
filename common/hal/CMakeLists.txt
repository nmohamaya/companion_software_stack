# common/hal/CMakeLists.txt — Header-only HAL library
add_library(drone_hal INTERFACE)
target_include_directories(drone_hal INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_hal INTERFACE
    drone_util
    Eigen3::Eigen
    spdlog::spdlog
)

# Generated model_defaults.h (from cmake/ModelPresets.cmake)
if(DEFINED GENERATED_INCLUDE_DIR)
    target_include_directories(drone_hal INTERFACE ${GENERATED_INCLUDE_DIR})
endif()

# ── Optional: MAVSDK backend ────────────────────────────────
if(MAVSDK_FOUND)
    target_compile_definitions(drone_hal INTERFACE HAVE_MAVSDK)
    target_link_libraries(drone_hal INTERFACE MAVSDK::mavsdk)
endif()

# ── Optional: Gazebo transport backend ──────────────────────
if(GAZEBO_FOUND)
    target_compile_definitions(drone_hal INTERFACE HAVE_GAZEBO)
    target_link_libraries(drone_hal INTERFACE
        gz-transport${GZ_TRANSPORT_VER}::core
        gz-msgs${GZ_MSGS_VER}::core
    )
endif()

# ── Optional: Cosys-AirSim backend ────────────────────────
# NOTE: Currently uses INTERFACE target_sources because the .cpp stubs are
# empty (ifdef guards only). When real AirSim SDK code is added, migrate to
# a STATIC library (drone_hal_cosys) to avoid compiling in every consumer.
if(COSYS_AIRSIM_FOUND)
    target_compile_definitions(drone_hal INTERFACE HAVE_COSYS_AIRSIM)
    target_sources(drone_hal INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cosys_camera.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cosys_radar.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cosys_imu.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cosys_depth.cpp
    )
endif()

# ── Optional: Depth Anything V2 (OpenCV DNN) ──────────────
# Uses OpenCV DNN to run DA V2 ONNX model for monocular metric depth estimation.
# Issue #455.
# NOTE: HAS_OPENCV is NOT defined here (INTERFACE would leak to all consumers,
# breaking tests that compile detector_factory.cpp without YOLO sources).
# HAS_OPENCV stays per-target: process2_perception and specific test targets.
# The DA V2 source compiles safely without HAS_OPENCV (returns error at runtime).
# TODO: When more OpenCV-dependent HAL backends accumulate, split into a separate
# drone_hal_opencv STATIC library to avoid INTERFACE link leaking ${OpenCV_LIBS}
# to all consumers. See PR #456 review discussion.
if(OPENCV_FOUND)
    target_sources(drone_hal INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/depth_anything_v2.cpp
    )
    target_link_libraries(drone_hal INTERFACE ${OpenCV_LIBS})
endif()
