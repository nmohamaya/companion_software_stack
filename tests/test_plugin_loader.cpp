// tests/test_plugin_loader.cpp
// Unit tests for PluginLoader, PluginHandle, and PluginRegistry.
//
// These tests load the test_plugin_mock.so shared library at runtime
// and verify correct lifetime management, error handling, and
// move semantics.
//
// Requires: cmake -DENABLE_PLUGINS=ON

#ifdef HAVE_PLUGINS

#include "hal/icamera.h"
#include "util/plugin_loader.h"

#include <string>

#include <gtest/gtest.h>

namespace {

// Path to the mock plugin .so — injected via CMake compile definition.
#ifndef TEST_PLUGIN_MOCK_PATH
#error "TEST_PLUGIN_MOCK_PATH must be defined by CMake"
#endif
const std::string kMockPluginPath = TEST_PLUGIN_MOCK_PATH;

}  // namespace

// ── PluginLoader::load — success cases ─────────────────────

TEST(PluginLoaderTest, LoadSucceeds) {
    auto result = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath);
    ASSERT_TRUE(result.is_ok()) << "Expected success, got: " << result.error();

    auto& handle = result.value();
    ASSERT_NE(handle.get(), nullptr);
    EXPECT_EQ(handle->name(), "MockPluginCamera");
}

TEST(PluginLoaderTest, LoadWithCustomSymbol) {
    auto result = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath,
                                                                       "create_camera_v2");
    ASSERT_TRUE(result.is_ok()) << "Expected success, got: " << result.error();
    EXPECT_EQ(result.value()->name(), "MockPluginCamera");
}

TEST(PluginLoaderTest, LoadedPluginIsUsable) {
    auto result = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath);
    ASSERT_TRUE(result.is_ok());

    auto& cam = result.value();
    EXPECT_TRUE(cam->open(640, 480, 30));
    EXPECT_TRUE(cam->is_open());

    auto frame = cam->capture();
    EXPECT_TRUE(frame.valid);
    EXPECT_EQ(frame.width, 640u);
    EXPECT_EQ(frame.height, 480u);

    cam->close();
    EXPECT_FALSE(cam->is_open());
}

// ── PluginLoader::load — error cases ───────────────────────

TEST(PluginLoaderTest, MissingFileReturnsError) {
    auto result =
        drone::util::PluginLoader::load<drone::hal::ICamera>("/nonexistent/path/plugin.so");
    ASSERT_TRUE(result.is_err());
    EXPECT_NE(result.error().find("not found"), std::string::npos) << "Error: " << result.error();
}

TEST(PluginLoaderTest, MissingSymbolReturnsError) {
    auto result = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath,
                                                                       "nonexistent_symbol");
    ASSERT_TRUE(result.is_err());
    EXPECT_NE(result.error().find("nonexistent_symbol"), std::string::npos)
        << "Error: " << result.error();
}

TEST(PluginLoaderTest, NotARegularFileReturnsError) {
    auto result = drone::util::PluginLoader::load<drone::hal::ICamera>("/tmp");
    ASSERT_TRUE(result.is_err());
    EXPECT_NE(result.error().find("not a regular file"), std::string::npos)
        << "Error: " << result.error();
}

// ── PluginHandle — move semantics ──────────────────────────

TEST(PluginHandleTest, MoveConstructor) {
    auto result = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath);
    ASSERT_TRUE(result.is_ok());

    auto handle1 = std::move(result.value());
    ASSERT_NE(handle1.get(), nullptr);
    EXPECT_EQ(handle1->name(), "MockPluginCamera");

    // Move to handle2
    auto handle2(std::move(handle1));
    EXPECT_EQ(handle1.get(), nullptr);  // NOLINT — testing moved-from state
    ASSERT_NE(handle2.get(), nullptr);
    EXPECT_EQ(handle2->name(), "MockPluginCamera");
}

TEST(PluginHandleTest, MoveAssignment) {
    auto result1 = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath);
    auto result2 = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath);
    ASSERT_TRUE(result1.is_ok());
    ASSERT_TRUE(result2.is_ok());

    auto handle = std::move(result1.value());
    handle      = std::move(result2.value());

    ASSERT_NE(handle.get(), nullptr);
    EXPECT_EQ(handle->name(), "MockPluginCamera");
}

// ── PluginHandle — release_instance ────────────────────────

TEST(PluginHandleTest, ReleaseInstance) {
    auto result = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath);
    ASSERT_TRUE(result.is_ok());

    auto& handle   = result.value();
    auto  instance = handle.release_instance();

    // Handle no longer owns the instance
    EXPECT_EQ(handle.get(), nullptr);

    // Instance is still valid
    ASSERT_NE(instance, nullptr);
    EXPECT_EQ(instance->name(), "MockPluginCamera");

    // The dl_handle is still alive in the PluginHandle, so the instance
    // can safely call virtual methods.
    EXPECT_TRUE(instance->open(320, 240, 15));
    instance->close();
}

// ── PluginRegistry ─────────────────────────────────────────

TEST(PluginRegistryTest, ExtractReturnsValidUniquePtr) {
    auto result = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath);
    ASSERT_TRUE(result.is_ok());

    const auto count_before = drone::util::PluginRegistry::instance().handle_count();
    auto instance = drone::util::PluginRegistry::instance().extract(std::move(result.value()));

    ASSERT_NE(instance, nullptr);
    EXPECT_EQ(instance->name(), "MockPluginCamera");
    EXPECT_EQ(drone::util::PluginRegistry::instance().handle_count(), count_before + 1);

    // Instance can be used normally through unique_ptr
    EXPECT_TRUE(instance->open(640, 480, 30));
    auto frame = instance->capture();
    EXPECT_TRUE(frame.valid);
    instance->close();
}

TEST(PluginRegistryTest, MultipleHandlesStored) {
    const auto count_before = drone::util::PluginRegistry::instance().handle_count();

    auto r1 = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath);
    auto r2 = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath);
    ASSERT_TRUE(r1.is_ok());
    ASSERT_TRUE(r2.is_ok());

    auto i1 = drone::util::PluginRegistry::instance().extract(std::move(r1.value()));
    auto i2 = drone::util::PluginRegistry::instance().extract(std::move(r2.value()));

    EXPECT_EQ(drone::util::PluginRegistry::instance().handle_count(), count_before + 2);
    EXPECT_EQ(i1->name(), "MockPluginCamera");
    EXPECT_EQ(i2->name(), "MockPluginCamera");
}

// ── PluginHandle — dereference operators ───────────────────

TEST(PluginHandleTest, DereferenceOperators) {
    auto result = drone::util::PluginLoader::load<drone::hal::ICamera>(kMockPluginPath);
    ASSERT_TRUE(result.is_ok());

    auto& handle = result.value();

    // operator*
    drone::hal::ICamera& ref = *handle;
    EXPECT_EQ(ref.name(), "MockPluginCamera");

    // operator->
    EXPECT_EQ(handle->name(), "MockPluginCamera");

    // get()
    EXPECT_NE(handle.get(), nullptr);
}

#else  // !HAVE_PLUGINS

#include <gtest/gtest.h>

TEST(PluginLoaderTest, PluginsDisabled) {
    // This test exists so the binary always has at least one test.
    // When HAVE_PLUGINS is not defined, plugin loading is not available.
    GTEST_SKIP() << "Plugin loading is disabled (ENABLE_PLUGINS=OFF)";
}

#endif  // HAVE_PLUGINS
