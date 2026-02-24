cmake_minimum_required(VERSION 3.16)
project(payload_manager LANGUAGES CXX)

add_executable(payload_manager src/main.cpp)
target_include_directories(payload_manager PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(payload_manager PRIVATE drone_ipc drone_util)
target_compile_features(payload_manager PRIVATE cxx_std_17)
