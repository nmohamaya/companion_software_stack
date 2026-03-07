# process3_slam_vio_nav/CMakeLists.txt
add_executable(slam_vio_nav
    src/main.cpp
    src/imu_preintegrator.cpp
)
target_include_directories(slam_vio_nav PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(slam_vio_nav PRIVATE
    drone_ipc drone_util drone_hal Eigen3::Eigen
)
install(TARGETS slam_vio_nav DESTINATION bin)
