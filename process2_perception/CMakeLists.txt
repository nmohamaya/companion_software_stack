# process2_perception/CMakeLists.txt
add_executable(perception
    src/main.cpp
    src/simulated_detector.cpp
    src/detector_factory.cpp
    src/kalman_tracker.cpp
    src/bytetrack_tracker.cpp
    src/fusion_engine.cpp
    src/ukf_fusion_engine.cpp
    src/opencv_yolo_detector.cpp
)
target_include_directories(perception PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(perception PRIVATE
    drone_ipc drone_util Eigen3::Eigen
)

# Optional OpenCV + YOLOv8 DNN detector
if(OPENCV_FOUND)
    target_link_libraries(perception PRIVATE ${OpenCV_LIBS})
    target_compile_definitions(perception PRIVATE HAS_OPENCV=1)
endif()

install(TARGETS perception DESTINATION bin)
