# tools/cosys_populate_scene/CMakeLists.txt
# Simulator-agnostic scene loader (Cosys-AirSim adapter).
# Part of epic #480 — proving-ground scenarios.

add_executable(cosys_populate_scene main.cpp)

target_link_libraries(cosys_populate_scene PRIVATE
    AirSim::All
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    Threads::Threads
)

target_include_directories(cosys_populate_scene PRIVATE
    ${PROJECT_SOURCE_DIR}/common/hal/include
    ${PROJECT_SOURCE_DIR}/common/util/include
)

target_compile_definitions(cosys_populate_scene PRIVATE HAVE_COSYS_AIRSIM=1)
