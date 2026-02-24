cmake_minimum_required(VERSION 3.16)
project(system_monitor LANGUAGES CXX)

add_executable(system_monitor src/main.cpp)
target_include_directories(system_monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(system_monitor PRIVATE drone_ipc drone_util)
target_compile_features(system_monitor PRIVATE cxx_std_17)
