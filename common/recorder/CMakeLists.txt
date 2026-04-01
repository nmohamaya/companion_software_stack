# common/recorder/CMakeLists.txt — Header-only flight data recorder library
add_library(drone_recorder INTERFACE)
target_include_directories(drone_recorder INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_recorder INTERFACE drone_ipc)
