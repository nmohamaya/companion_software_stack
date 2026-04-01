# tools/flight_replay/CMakeLists.txt
# Flight replay tool — reads recorded .flog files and republishes to IPC.

add_executable(flight_replay main.cpp)

target_link_libraries(flight_replay PRIVATE
    drone_ipc
    drone_util
    drone_recorder
    Threads::Threads
    spdlog::spdlog
)

target_include_directories(flight_replay PRIVATE
    ${PROJECT_SOURCE_DIR}/common/ipc/include
    ${PROJECT_SOURCE_DIR}/common/util/include
    ${PROJECT_SOURCE_DIR}/common/recorder/include
)
