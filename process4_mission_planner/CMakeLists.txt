cmake_minimum_required(VERSION 3.16)
project(mission_planner LANGUAGES CXX)

add_executable(mission_planner src/main.cpp)
target_include_directories(mission_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mission_planner PRIVATE drone_ipc drone_util)
target_compile_features(mission_planner PRIVATE cxx_std_17)
