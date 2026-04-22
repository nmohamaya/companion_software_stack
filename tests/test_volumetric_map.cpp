// tests/test_volumetric_map.cpp
// Unit tests for IVolumetricMap HAL interface + SimulatedVolumetricMap.
#include "hal/hal_factory.h"
#include "hal/ivolumetric_map.h"
#include "hal/simulated_volumetric_map.h"
#include "util/config.h"

#include <cstdio>
#include <fstream>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::hal;

// ── Temp config helper ──

static std::vector<std::string> g_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char tmpl[] = "/tmp/test_vmap_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);
    if (fd < 0) {
        std::string   path = "/tmp/test_vmap_" + std::to_string(getpid()) + ".json";
        std::ofstream ofs(path);
        ofs << json_content;
        g_temp_files.push_back(path);
        return path;
    }
    ::close(fd);
    std::string   path(tmpl);
    std::ofstream ofs(path);
    ofs << json_content;
    g_temp_files.push_back(path);
    return path;
}

struct TempFileCleanup {
    ~TempFileCleanup() {
        for (auto& f : g_temp_files) std::remove(f.c_str());
    }
};
static TempFileCleanup g_cleanup;

// ── Tests ──

TEST(VolumetricMap, SimulatedName) {
    SimulatedVolumetricMap map;
    EXPECT_EQ(map.name(), "SimulatedVolumetricMap");
}

TEST(VolumetricMap, InitValid) {
    SimulatedVolumetricMap map;
    EXPECT_TRUE(map.init(0.5f));
    EXPECT_FLOAT_EQ(map.resolution(), 0.5f);
}

TEST(VolumetricMap, InitInvalidResolution) {
    SimulatedVolumetricMap map;
    EXPECT_FALSE(map.init(0.0f));
    EXPECT_FALSE(map.init(-1.0f));
}

TEST(VolumetricMap, InsertAndQuery) {
    SimulatedVolumetricMap map(0.5f);
    ASSERT_TRUE(map.init(0.5f));

    VoxelUpdate update;
    update.position_m     = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    update.occupancy      = 0.9f;
    update.semantic_label = 1;
    update.confidence     = 0.8f;
    update.timestamp_ns   = 1000;

    EXPECT_TRUE(map.insert({update}));
    EXPECT_EQ(map.size(), 1u);

    auto result = map.query(Eigen::Vector3f(1.1f, 2.1f, 3.1f));
    ASSERT_TRUE(result.has_value());
    EXPECT_FLOAT_EQ(result->occupancy, 0.9f);
    EXPECT_EQ(result->semantic_label, 1);
    EXPECT_FLOAT_EQ(result->confidence, 0.8f);
}

TEST(VolumetricMap, QueryMissReturnsNullopt) {
    SimulatedVolumetricMap map(0.5f);
    ASSERT_TRUE(map.init(0.5f));

    auto result = map.query(Eigen::Vector3f(100.0f, 200.0f, 300.0f));
    EXPECT_FALSE(result.has_value());
}

TEST(VolumetricMap, Clear) {
    SimulatedVolumetricMap map(0.5f);
    ASSERT_TRUE(map.init(0.5f));

    VoxelUpdate update;
    update.position_m = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    ASSERT_TRUE(map.insert({update}));
    EXPECT_EQ(map.size(), 1u);

    map.clear();
    EXPECT_EQ(map.size(), 0u);
}

TEST(VolumetricMap, MultipleInsertsSameVoxel) {
    SimulatedVolumetricMap map(1.0f);
    ASSERT_TRUE(map.init(1.0f));

    VoxelUpdate u1;
    u1.position_m     = Eigen::Vector3f(0.3f, 0.3f, 0.3f);
    u1.semantic_label = 1;
    u1.confidence     = 0.5f;

    VoxelUpdate u2;
    u2.position_m     = Eigen::Vector3f(0.7f, 0.7f, 0.7f);
    u2.semantic_label = 2;
    u2.confidence     = 0.9f;

    ASSERT_TRUE(map.insert({u1, u2}));
    EXPECT_EQ(map.size(), 1u);

    auto result = map.query(Eigen::Vector3f(0.5f, 0.5f, 0.5f));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->semantic_label, 2);
}

TEST(VolumetricMap, FactorySimulated) {
    auto          path = create_temp_config(R"({
        "perception": { "volumetric_map": { "backend": "simulated" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto map = drone::hal::create_volumetric_map(cfg);
    ASSERT_NE(map, nullptr);
    EXPECT_EQ(map->name(), "SimulatedVolumetricMap");
}

TEST(VolumetricMap, FactoryUnknownThrows) {
    auto          path = create_temp_config(R"({
        "perception": { "volumetric_map": { "backend": "nonexistent" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    EXPECT_THROW(drone::hal::create_volumetric_map(cfg), std::runtime_error);
}
