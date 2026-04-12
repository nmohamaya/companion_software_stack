# common/util/CMakeLists.txt — Header-only utility library
add_library(drone_util INTERFACE)
target_include_directories(drone_util INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_util INTERFACE
    fmt::fmt
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    pthread
)
