# tools/fault_injector/CMakeLists.txt
# Fault injection CLI tool — writes to IPC channels to simulate faults.

add_executable(fault_injector main.cpp)

target_link_libraries(fault_injector PRIVATE
    drone_ipc
    drone_util
    Threads::Threads
    spdlog::spdlog
    nlohmann_json::nlohmann_json
)

target_include_directories(fault_injector PRIVATE
    ${PROJECT_SOURCE_DIR}/common/ipc/include
    ${PROJECT_SOURCE_DIR}/common/util/include
)
