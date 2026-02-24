# process1_video_capture/CMakeLists.txt
add_executable(video_capture src/main.cpp)
target_include_directories(video_capture PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(video_capture PRIVATE drone_ipc drone_util drone_hal)
install(TARGETS video_capture DESTINATION bin)
