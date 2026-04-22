// tests/test_config_validator.cpp
// Unit tests for ConfigSchema / validate().
#include "util/config_validator.h"

#include <cstdio>
#include <fstream>

#include <gtest/gtest.h>

using drone::util::ConfigSchema;
using drone::util::validate;

class ConfigValidatorTest : public ::testing::Test {
protected:
    std::string   tmp_path_;
    drone::Config cfg_;

    void load_json(const std::string& content) {
        tmp_path_ = "/tmp/drone_test_validator_" + std::to_string(::getpid()) + ".json";
        std::ofstream ofs(tmp_path_);
        ofs << content;
        ofs.close();
        ASSERT_TRUE(cfg_.load(tmp_path_));
    }

    void TearDown() override {
        if (!tmp_path_.empty()) std::remove(tmp_path_.c_str());
    }
};

// ─── Valid config passes ─────────────────────────────────────

TEST_F(ConfigValidatorTest, ValidConfigPasses) {
    load_json(R"({
        "log_level": "info",
        "slam": { "vio_rate_hz": 100 }
    })");

    ConfigSchema s;
    s.required<int>("slam.vio_rate_hz").range(1, 10000);
    s.optional<std::string>("log_level").one_of({"trace", "debug", "info", "warn", "error"});

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

// ─── Missing required key ────────────────────────────────────

TEST_F(ConfigValidatorTest, MissingRequiredKey) {
    load_json(R"({})");

    ConfigSchema s;
    s.required<int>("slam.vio_rate_hz");

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    ASSERT_EQ(r.error().size(), 1u);
    EXPECT_NE(r.error()[0].find("Missing required key"), std::string::npos);
    EXPECT_NE(r.error()[0].find("slam.vio_rate_hz"), std::string::npos);
}

// ─── Missing optional key → OK ───────────────────────────────

TEST_F(ConfigValidatorTest, MissingOptionalKeyOk) {
    load_json(R"({})");

    ConfigSchema s;
    s.optional<int>("nonexistent_key").range(1, 100);

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

// ─── Out of range ────────────────────────────────────────────

TEST_F(ConfigValidatorTest, OutOfRange) {
    load_json(R"({"rate": 0})");

    ConfigSchema s;
    s.required<int>("rate").range(1, 1000);

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    ASSERT_EQ(r.error().size(), 1u);
    EXPECT_NE(r.error()[0].find("Out of range"), std::string::npos);
}

TEST_F(ConfigValidatorTest, OutOfRangeAbove) {
    load_json(R"({"rate": 99999})");

    ConfigSchema s;
    s.required<int>("rate").range(1, 1000);

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    ASSERT_EQ(r.error().size(), 1u);
    EXPECT_NE(r.error()[0].find("Out of range"), std::string::npos);
}

TEST_F(ConfigValidatorTest, InRangeBoundary) {
    load_json(R"({"rate": 1})");

    ConfigSchema s;
    s.required<int>("rate").range(1, 1000);

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

// ─── Wrong type ──────────────────────────────────────────────

TEST_F(ConfigValidatorTest, TypeMismatch) {
    load_json(R"({"rate": "fast"})");

    ConfigSchema s;
    s.required<int>("rate");

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    ASSERT_EQ(r.error().size(), 1u);
    EXPECT_NE(r.error()[0].find("Type mismatch"), std::string::npos);
}

// ─── one_of check ────────────────────────────────────────────

TEST_F(ConfigValidatorTest, OneOfValid) {
    load_json(R"({"log_level": "debug"})");

    ConfigSchema s;
    s.required<std::string>("log_level").one_of({"trace", "debug", "info"});

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

TEST_F(ConfigValidatorTest, OneOfInvalid) {
    load_json(R"({"log_level": "verbose"})");

    ConfigSchema s;
    s.required<std::string>("log_level").one_of({"trace", "debug", "info"});

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    ASSERT_EQ(r.error().size(), 1u);
    EXPECT_NE(r.error()[0].find("Invalid value"), std::string::npos);
    EXPECT_NE(r.error()[0].find("verbose"), std::string::npos);
}

// ─── required_section ────────────────────────────────────────

TEST_F(ConfigValidatorTest, RequiredSectionPresent) {
    load_json(R"({"slam": {"vio_rate_hz": 100}})");

    ConfigSchema s;
    s.required_section("slam");

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

TEST_F(ConfigValidatorTest, RequiredSectionMissing) {
    load_json(R"({})");

    ConfigSchema s;
    s.required_section("slam");

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    ASSERT_EQ(r.error().size(), 1u);
    EXPECT_NE(r.error()[0].find("Missing required section"), std::string::npos);
}

// ─── Multiple errors collected ───────────────────────────────

TEST_F(ConfigValidatorTest, MultipleErrors) {
    load_json(R"({"log_level": "verbose"})");

    ConfigSchema s;
    s.required<int>("rate").range(1, 100);
    s.required<std::string>("log_level").one_of({"info", "debug"});
    s.required_section("slam");

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    // Should collect all 3 errors: missing rate, invalid log_level, missing slam
    EXPECT_EQ(r.error().size(), 3u);
}

// ─── Custom predicate ────────────────────────────────────────

TEST_F(ConfigValidatorTest, CustomPredicatePass) {
    load_json(R"({"port": 8080})");

    ConfigSchema s;
    s.required<int>("port").satisfies([](const int& p) { return p > 1024; }, "port must be > 1024");

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

TEST_F(ConfigValidatorTest, CustomPredicateFail) {
    load_json(R"({"port": 80})");

    ConfigSchema s;
    s.required<int>("port").satisfies([](const int& p) { return p > 1024; }, "port must be > 1024");

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    ASSERT_EQ(r.error().size(), 1u);
    EXPECT_NE(r.error()[0].find("port must be > 1024"), std::string::npos);
}

// ─── Double range ────────────────────────────────────────────

TEST_F(ConfigValidatorTest, DoubleRange) {
    load_json(R"({"gain": 0.5})");

    ConfigSchema s;
    s.required<double>("gain").range(0.0, 1.0);

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

TEST_F(ConfigValidatorTest, DoubleOutOfRange) {
    load_json(R"({"gain": 1.5})");

    ConfigSchema s;
    s.required<double>("gain").range(0.0, 1.0);

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
}

// ─── Pre-built process schemas ───────────────────────────────

TEST_F(ConfigValidatorTest, DefaultConfigPassesAllSchemas) {
    // Load the actual default.json — it should pass all schemas
    drone::Config cfg;
#ifdef PROJECT_CONFIG_DIR
    std::string config_path = std::string(PROJECT_CONFIG_DIR) + "/default.json";
#else
    std::string config_path = "config/default.json";
#endif
    bool loaded = cfg.load(config_path);
    if (!loaded) {
        GTEST_SKIP() << "config/default.json not available at " << config_path;
    }

    auto check = [&](const char* name, ConfigSchema schema) {
        auto r = validate(cfg, schema);
        if (r.is_err()) {
            for (const auto& e : r.error()) {
                ADD_FAILURE() << name << ": " << e;
            }
        }
    };

    check("video_capture", drone::util::video_capture_schema());
    check("perception", drone::util::perception_schema());
    check("slam", drone::util::slam_schema());
    check("mission_planner", drone::util::mission_planner_schema());
    check("comms", drone::util::comms_schema());
    check("payload_manager", drone::util::payload_manager_schema());
    check("system_monitor", drone::util::system_monitor_schema());
}

// ─── Bool field ──────────────────────────────────────────────

TEST_F(ConfigValidatorTest, BoolField) {
    load_json(R"({"enabled": true})");

    ConfigSchema s;
    s.required<bool>("enabled");

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

TEST_F(ConfigValidatorTest, BoolFieldTypeMismatch) {
    load_json(R"({"enabled": "yes"})");

    ConfigSchema s;
    s.required<bool>("enabled");

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    EXPECT_NE(r.error()[0].find("Type mismatch"), std::string::npos);
}

// ─── Nested key validation ───────────────────────────────────

TEST_F(ConfigValidatorTest, DeeplyNestedKey) {
    load_json(R"({"a": {"b": {"c": 42}}})");

    ConfigSchema s;
    s.required<int>("a.b.c").range(0, 100);

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

// ─── Custom rule on schema ───────────────────────────────────

TEST_F(ConfigValidatorTest, CustomRuleOnSchema) {
    load_json(R"({"battery_warn": 20, "battery_crit": 10})");

    ConfigSchema s;
    s.required<int>("battery_warn");
    s.required<int>("battery_crit");
    s.custom([](const drone::Config& cfg, std::vector<std::string>& errors) {
        auto warn = cfg.get<int>("battery_warn", 0);
        auto crit = cfg.get<int>("battery_crit", 0);
        if (crit >= warn) {
            errors.push_back("battery_crit must be less than battery_warn");
        }
    });

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

TEST_F(ConfigValidatorTest, CustomRuleFails) {
    load_json(R"({"battery_warn": 10, "battery_crit": 20})");

    ConfigSchema s;
    s.required<int>("battery_warn");
    s.required<int>("battery_crit");
    s.custom([](const drone::Config& cfg, std::vector<std::string>& errors) {
        auto warn = cfg.get<int>("battery_warn", 0);
        auto crit = cfg.get<int>("battery_crit", 0);
        if (crit >= warn) {
            errors.push_back("battery_crit must be less than battery_warn");
        }
    });

    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    EXPECT_EQ(r.error().size(), 1u);
    EXPECT_NE(r.error()[0].find("battery_crit must be less than battery_warn"), std::string::npos);
}

// ═══════════════════════════════════════════════════════════
// Per-class section validation (Epic #519)
// ���═══��══════════════════════════════════════════════════════

TEST_F(ConfigValidatorTest, PerClassSectionValidPasses) {
    load_json(R"({
        "per_class": {
            "influence": { "default": 5.0, "person": 3.0, "drone": 4.0, "_comment": "test" }
        }
    })");
    ConfigSchema s;
    s.per_class_section("per_class.influence");
    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

TEST_F(ConfigValidatorTest, PerClassSectionTypoRejected) {
    load_json(R"({
        "per_class": {
            "influence": { "default": 5.0, "personn": 3.0 }
        }
    })");
    ConfigSchema s;
    s.per_class_section("per_class.influence");
    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_err());
    ASSERT_EQ(r.error().size(), 1u);
    EXPECT_NE(r.error()[0].find("personn"), std::string::npos);
}

TEST_F(ConfigValidatorTest, PerClassSectionMissingIsOk) {
    load_json(R"({ "other": 1 })");
    ConfigSchema s;
    s.per_class_section("per_class.influence");
    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}

TEST_F(ConfigValidatorTest, PerClassSectionCommentKeysAllowed) {
    load_json(R"({
        "sec": { "_comment_why": "explanation", "default": 1.0, "tree": 2.0 }
    })");
    ConfigSchema s;
    s.per_class_section("sec");
    auto r = validate(cfg_, s);
    EXPECT_TRUE(r.is_ok());
}
