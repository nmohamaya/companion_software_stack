cmake_minimum_required(VERSION 3.16)
project(comms LANGUAGES CXX)

add_executable(comms src/main.cpp)
target_include_directories(comms PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(comms PRIVATE drone_ipc drone_util)
target_compile_features(comms PRIVATE cxx_std_17)
