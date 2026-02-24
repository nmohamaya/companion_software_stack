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
