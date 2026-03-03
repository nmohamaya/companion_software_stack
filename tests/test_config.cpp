// tests/test_config.cpp
// Unit tests for the JSON configuration system.
#include "util/config.h"

#include <cstdio>
#include <fstream>

#include <gtest/gtest.h>

class ConfigTest : public ::testing::Test {
protected:
    std::string tmp_path_;

    void write_json(const std::string& content) {
        tmp_path_ = "/tmp/drone_test_config_" + std::to_string(::getpid()) + ".json";
        std::ofstream ofs(tmp_path_);
        ofs << content;
        ofs.close();
    }

    void TearDown() override {
        if (!tmp_path_.empty()) std::remove(tmp_path_.c_str());
    }
};

TEST_F(ConfigTest, LoadValidFile) {
    write_json(R"({
        "log_level": "debug",
        "video": { "width": 1280, "height": 720 }
    })");

    drone::Config cfg;
    EXPECT_TRUE(cfg.load(tmp_path_));
    EXPECT_TRUE(cfg.loaded());
}

TEST_F(ConfigTest, LoadNonExistentFile) {
    drone::Config cfg;
    EXPECT_FALSE(cfg.load("/nonexistent/path/config.json"));
    EXPECT_FALSE(cfg.loaded());
}

TEST_F(ConfigTest, LoadInvalidJSON) {
    write_json("{ broken json !!!");
    drone::Config cfg;
    EXPECT_FALSE(cfg.load(tmp_path_));
    EXPECT_FALSE(cfg.loaded());
}

TEST_F(ConfigTest, GetTopLevelString) {
    write_json(R"({"log_level": "debug"})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));
    EXPECT_EQ(cfg.get<std::string>("log_level", "info"), "debug");
}

TEST_F(ConfigTest, GetDefaultOnMissingKey) {
    write_json(R"({})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));
    EXPECT_EQ(cfg.get<std::string>("log_level", "info"), "info");
    EXPECT_EQ(cfg.get<int>("missing_int", 42), 42);
}

TEST_F(ConfigTest, GetNestedValue) {
    write_json(R"({
        "video": {
            "mission_cam": {
                "width": 1920,
                "height": 1080,
                "fps": 30
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));

    EXPECT_EQ(cfg.get<int>("video.mission_cam.width", 0), 1920);
    EXPECT_EQ(cfg.get<int>("video.mission_cam.height", 0), 1080);
    EXPECT_EQ(cfg.get<int>("video.mission_cam.fps", 0), 30);
}

TEST_F(ConfigTest, GetDeeplyNestedDefault) {
    write_json(R"({"a": {"b": {}}})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));
    EXPECT_EQ(cfg.get<int>("a.b.c.d", 99), 99);
}

TEST_F(ConfigTest, HasKey) {
    write_json(R"({"a": {"b": 1}})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));
    EXPECT_TRUE(cfg.has("a"));
    EXPECT_TRUE(cfg.has("a.b"));
    EXPECT_FALSE(cfg.has("a.c"));
    EXPECT_FALSE(cfg.has("x"));
}

TEST_F(ConfigTest, Section) {
    write_json(R"({"parent": {"child1": 1, "child2": "hello"}})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));

    auto sec = cfg.section("parent");
    EXPECT_EQ(sec["child1"].get<int>(), 1);
    EXPECT_EQ(sec["child2"].get<std::string>(), "hello");
}

TEST_F(ConfigTest, MissingSectionReturnsEmptyObject) {
    write_json(R"({})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));
    auto sec = cfg.section("nonexistent");
    EXPECT_TRUE(sec.is_object());
    EXPECT_TRUE(sec.empty());
}

TEST_F(ConfigTest, TypeMismatchReturnsDefault) {
    write_json(R"({"value": "not_a_number"})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));
    EXPECT_EQ(cfg.get<int>("value", 42), 42);
}

TEST_F(ConfigTest, FloatValues) {
    write_json(R"({"pi": 3.14159, "nested": {"gain": 0.8}})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));
    EXPECT_FLOAT_EQ(cfg.get<float>("pi", 0.0f), 3.14159f);
    EXPECT_FLOAT_EQ(cfg.get<float>("nested.gain", 0.0f), 0.8f);
}

TEST_F(ConfigTest, BoolValues) {
    write_json(R"({"enabled": true, "disabled": false})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));
    EXPECT_TRUE(cfg.get<bool>("enabled", false));
    EXPECT_FALSE(cfg.get<bool>("disabled", true));
}

TEST_F(ConfigTest, LoadDefaultConfigFile) {
    // Test loading the actual project default config
    drone::Config cfg;
    // This may or may not succeed depending on CWD, so don't ASSERT
    bool loaded = cfg.load("config/default.json");
    if (loaded) {
        EXPECT_EQ(cfg.get<std::string>("log_level", ""), "info");
        EXPECT_EQ(cfg.get<int>("video_capture.mission_cam.width", 0), 1920);
        EXPECT_EQ(cfg.get<int>("perception.tracker.min_hits", 0), 3);
        EXPECT_EQ(cfg.get<int>("slam.vio_rate_hz", 0), 100);
    }
}

// ═══════════════════════════════════════════════════════════
// Result-based Config API (load_config / require)
// ═══════════════════════════════════════════════════════════

TEST_F(ConfigTest, LoadConfigSuccess) {
    write_json(R"({"key": "value"})");
    drone::Config cfg;
    auto          r = cfg.load_config(tmp_path_);
    EXPECT_TRUE(r.is_ok());
    EXPECT_TRUE(cfg.loaded());
}

TEST_F(ConfigTest, LoadConfigFileNotFound) {
    drone::Config cfg;
    auto          r = cfg.load_config("/nonexistent/file.json");
    EXPECT_TRUE(r.is_err());
    EXPECT_EQ(r.error().code(), drone::util::ErrorCode::FileNotFound);
    EXPECT_FALSE(cfg.loaded());
}

TEST_F(ConfigTest, LoadConfigParseError) {
    write_json("{invalid json!!!");
    drone::Config cfg;
    auto          r = cfg.load_config(tmp_path_);
    EXPECT_TRUE(r.is_err());
    EXPECT_EQ(r.error().code(), drone::util::ErrorCode::ParseError);
    EXPECT_FALSE(cfg.loaded());
}

TEST_F(ConfigTest, RequireExistingKey) {
    write_json(R"({"port": 8080, "name": "drone1"})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));

    auto port = cfg.require<int>("port");
    EXPECT_TRUE(port.is_ok());
    EXPECT_EQ(port.value(), 8080);

    auto name = cfg.require<std::string>("name");
    EXPECT_TRUE(name.is_ok());
    EXPECT_EQ(name.value(), "drone1");
}

TEST_F(ConfigTest, RequireMissingKey) {
    write_json(R"({})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));

    auto r = cfg.require<int>("missing_key");
    EXPECT_TRUE(r.is_err());
    EXPECT_EQ(r.error().code(), drone::util::ErrorCode::MissingKey);
}

TEST_F(ConfigTest, RequireNestedKey) {
    write_json(R"({"video": {"width": 1920}})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));

    auto w = cfg.require<int>("video.width");
    EXPECT_TRUE(w.is_ok());
    EXPECT_EQ(w.value(), 1920);

    auto h = cfg.require<int>("video.height");
    EXPECT_TRUE(h.is_err());
    EXPECT_EQ(h.error().code(), drone::util::ErrorCode::MissingKey);
}

TEST_F(ConfigTest, RequireTypeMismatch) {
    write_json(R"({"value": "not_a_number"})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));

    auto r = cfg.require<int>("value");
    EXPECT_TRUE(r.is_err());
    // nlohmann throws type_error which we catch as TypeMismatch
    EXPECT_EQ(r.error().code(), drone::util::ErrorCode::TypeMismatch);
}

TEST_F(ConfigTest, RequireWithValueOr) {
    write_json(R"({"a": 10})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));

    EXPECT_EQ(cfg.require<int>("a").value_or(99), 10);
    EXPECT_EQ(cfg.require<int>("missing").value_or(99), 99);
}

TEST_F(ConfigTest, RequireWithMap) {
    write_json(R"({"port": 8080})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp_path_));

    auto doubled = cfg.require<int>("port").map([](int p) { return p * 2; });
    EXPECT_TRUE(doubled.is_ok());
    EXPECT_EQ(doubled.value(), 16160);
}
