# common/ipc/CMakeLists.txt — Header-only IPC library (Zenoh backend)
add_library(drone_ipc INTERFACE)
target_include_directories(drone_ipc INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_ipc INTERFACE zenohc::lib pthread)
target_compile_definitions(drone_ipc INTERFACE ZENOHCXX_ZENOHC)
