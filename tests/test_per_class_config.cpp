// tests/test_per_class_config.cpp
// Unit tests for drone::util per-class config utility (Epic #519).
#include "util/per_class_config.h"

#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using namespace drone::util;

// ── Helpers ─────────────────────────────────────────────────

static drone::Config make_config(const nlohmann::json& data) {
    // Write to temp file and load — Config requires a file path.
    const std::string path = "/tmp/test_per_class_config_" +
                             std::to_string(reinterpret_cast<uintptr_t>(&data)) + ".json";
    std::ofstream ofs(path);
    ofs << data.dump();
    ofs.close();
    drone::Config cfg;
    cfg.load(path);
    std::remove(path.c_str());
    return cfg;
}

// ═══════════════════════════════════════════════════════════════
// class_name_to_index
// ═══════════════════════════════════════════════════════════════

TEST(PerClassConfig, ClassNameToIndex_AllValid) {
    EXPECT_EQ(class_name_to_index("unknown"), 0);
    EXPECT_EQ(class_name_to_index("person"), 1);
    EXPECT_EQ(class_name_to_index("vehicle_car"), 2);
    EXPECT_EQ(class_name_to_index("vehicle_truck"), 3);
    EXPECT_EQ(class_name_to_index("drone"), 4);
    EXPECT_EQ(class_name_to_index("animal"), 5);
    EXPECT_EQ(class_name_to_index("building"), 6);
    EXPECT_EQ(class_name_to_index("tree"), 7);
}

TEST(PerClassConfig, ClassNameToIndex_InvalidReturnsNeg1) {
    EXPECT_EQ(class_name_to_index("personn"), -1);
    EXPECT_EQ(class_name_to_index(""), -1);
    EXPECT_EQ(class_name_to_index("PERSON"), -1);
}

// ═══════════════════════════════════════════════════════════════
// load_per_class
// ═══════════════════════════════════════════════════════════════

TEST(PerClassConfig, LoadPerClass_AllClassesSpecified) {
    nlohmann::json data   = {{"test_section",
                              {{"unknown", 1.0},
                               {"person", 2.0},
                               {"vehicle_car", 3.0},
                               {"vehicle_truck", 4.0},
                               {"drone", 5.0},
                               {"animal", 6.0},
                               {"building", 7.0},
                               {"tree", 8.0}}}};
    auto           cfg    = make_config(data);
    auto           result = load_per_class<float>(cfg, "test_section", 0.0f);

    EXPECT_FLOAT_EQ(result[0], 1.0f);
    EXPECT_FLOAT_EQ(result[1], 2.0f);
    EXPECT_FLOAT_EQ(result[2], 3.0f);
    EXPECT_FLOAT_EQ(result[3], 4.0f);
    EXPECT_FLOAT_EQ(result[4], 5.0f);
    EXPECT_FLOAT_EQ(result[5], 6.0f);
    EXPECT_FLOAT_EQ(result[6], 7.0f);
    EXPECT_FLOAT_EQ(result[7], 8.0f);
}

TEST(PerClassConfig, LoadPerClass_DefaultFallback) {
    nlohmann::json data   = {{"test_section", {{"default", 5.0}, {"person", 2.0}}}};
    auto           cfg    = make_config(data);
    auto           result = load_per_class<float>(cfg, "test_section", 99.0f);

    EXPECT_FLOAT_EQ(result[0], 5.0f);  // unknown → default
    EXPECT_FLOAT_EQ(result[1], 2.0f);  // person → override
    EXPECT_FLOAT_EQ(result[4], 5.0f);  // drone → default
    EXPECT_FLOAT_EQ(result[7], 5.0f);  // tree → default
}

TEST(PerClassConfig, LoadPerClass_NoDefault_UsesFallback) {
    nlohmann::json data   = {{"test_section", {{"person", 2.0}}}};
    auto           cfg    = make_config(data);
    auto           result = load_per_class<float>(cfg, "test_section", 99.0f);

    EXPECT_FLOAT_EQ(result[0], 99.0f);  // fallback
    EXPECT_FLOAT_EQ(result[1], 2.0f);   // person override
    EXPECT_FLOAT_EQ(result[4], 99.0f);  // fallback
}

TEST(PerClassConfig, LoadPerClass_MissingSection) {
    nlohmann::json data   = {{"other_section", 42}};
    auto           cfg    = make_config(data);
    auto           result = load_per_class<float>(cfg, "test_section", 7.0f);

    for (uint8_t i = 0; i < kPerClassCount; ++i) {
        EXPECT_FLOAT_EQ(result[i], 7.0f);
    }
}

TEST(PerClassConfig, LoadPerClass_UnknownKeysCollected) {
    nlohmann::json           data = {{"sec", {{"default", 1.0}, {"personn", 2.0}, {"cat", 3.0}}}};
    auto                     cfg  = make_config(data);
    std::vector<std::string> unknown;
    auto                     result = load_per_class<float>(cfg, "sec", 0.0f, &unknown);

    EXPECT_EQ(unknown.size(), 2u);
    // All values should be the default since the keys were unrecognized.
    EXPECT_FLOAT_EQ(result[1], 1.0f);
}

TEST(PerClassConfig, LoadPerClass_CommentKeysIgnored) {
    nlohmann::json data = {
        {"sec", {{"default", 1.0}, {"_comment", "this is a comment"}, {"person", 2.0}}}};
    auto                     cfg = make_config(data);
    std::vector<std::string> unknown;
    auto                     result = load_per_class<float>(cfg, "sec", 0.0f, &unknown);

    EXPECT_TRUE(unknown.empty());
    EXPECT_FLOAT_EQ(result[1], 2.0f);
}

TEST(PerClassConfig, LoadPerClass_TypeMismatchUsesFallback) {
    // "default" is a string but we load as float → type mismatch, should use compiled fallback.
    nlohmann::json data   = {{"sec", {{"default", "not_a_number"}, {"person", "also_not"}}}};
    auto           cfg    = make_config(data);
    auto           result = load_per_class<float>(cfg, "sec", 42.0f);

    for (uint8_t i = 0; i < kPerClassCount; ++i) {
        EXPECT_FLOAT_EQ(result[i], 42.0f);
    }
}

TEST(PerClassConfig, LoadPerClass_StringType) {
    nlohmann::json data = {
        {"sec", {{"default", "constant_velocity"}, {"drone", "constant_acceleration"}}}};
    auto cfg    = make_config(data);
    auto result = load_per_class<std::string>(cfg, "sec", "constant_velocity");

    EXPECT_EQ(result[0], "constant_velocity");
    EXPECT_EQ(result[4], "constant_acceleration");
}

// ═══════════════════════════════════════════════════════════════
// validate_per_class_section
// ═══════════════════════════════════════════════════════════════

TEST(PerClassConfig, Validate_AllValid) {
    nlohmann::json data = {
        {"sec", {{"default", 1.0}, {"person", 2.0}, {"drone", 3.0}, {"_comment_x", "ok"}}}};
    auto cfg    = make_config(data);
    auto errors = validate_per_class_section(cfg, "sec");
    EXPECT_TRUE(errors.empty());
}

TEST(PerClassConfig, Validate_UnknownClassName) {
    nlohmann::json data   = {{"sec", {{"personn", 2.0}}}};
    auto           cfg    = make_config(data);
    auto           errors = validate_per_class_section(cfg, "sec");
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_NE(errors[0].find("personn"), std::string::npos);
}

TEST(PerClassConfig, Validate_MissingSectionReturnsEmpty) {
    nlohmann::json data   = {{"other", 1}};
    auto           cfg    = make_config(data);
    auto           errors = validate_per_class_section(cfg, "sec");
    EXPECT_TRUE(errors.empty());
}
