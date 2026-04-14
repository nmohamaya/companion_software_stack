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
if(COSYS_AIRSIM_FOUND)
    target_compile_definitions(drone_hal INTERFACE HAVE_COSYS_AIRSIM)
    target_sources(drone_hal INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cosys_camera.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cosys_radar.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cosys_imu.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cosys_depth.cpp
    )
endif()
