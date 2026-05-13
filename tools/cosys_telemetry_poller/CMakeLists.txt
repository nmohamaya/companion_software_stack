# tools/cosys_telemetry_poller/CMakeLists.txt
# Cosys-AirSim sim-side telemetry poller for scenario runs (Issue #612 / #607).
# Polls simGetCollisionInfo + simGetGroundTruthKinematics at 10 Hz, writes
# JSONL alongside the other run logs.  Consumed by tools/plot_voxel_trace.py
# to overlay true drone pose + collision events on the perception-side
# voxel trace.

add_executable(cosys_telemetry_poller main.cpp)

target_link_libraries(cosys_telemetry_poller PRIVATE
    AirSim::All
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    Threads::Threads
)

target_include_directories(cosys_telemetry_poller PRIVATE
    ${PROJECT_SOURCE_DIR}/common/hal/include
    ${PROJECT_SOURCE_DIR}/common/util/include
)

target_compile_definitions(cosys_telemetry_poller PRIVATE HAVE_COSYS_AIRSIM=1)

install(TARGETS cosys_telemetry_poller DESTINATION bin)
