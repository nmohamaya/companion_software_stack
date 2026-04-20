// tests/test_gt_emitter.cpp
//
// Unit tests for the ground-truth emitter shared layer (Issue #594 PR 1).
// Focus: GtClassMap pattern matching and JSONL serialisation. The Cosys
// backend itself needs a live AirSim server and is covered by the scenario
// integration tests (#29, #30).

#include "benchmark/gt_emitter.h"
#include "util/config.h"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace db = drone::benchmark;

namespace {

// Materialise a tiny JSON config with the given `gt_class_map` block, load it
// via drone::Config, return the parsed map. Saves each test from having to
// write its own temp-file boilerplate.
db::GtClassMap load_map_from_json(const nlohmann::json& gt_class_map_block) {
    nlohmann::json root = {{"gt_class_map", gt_class_map_block}};
    const auto     path = std::filesystem::temp_directory_path() / "drone_test_gt_class_map.json";
    {
        std::ofstream out(path);
        out << root.dump();
    }
    drone::Config               cfg;
    [[maybe_unused]] const auto ok = cfg.load(path.string());
    std::error_code             ec;
    std::filesystem::remove(path, ec);
    return db::GtClassMap::load(cfg);
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────────
// GtClassMap — pattern matching
// ────────────────────────────────────────────────────────────────────────────

TEST(GtClassMap, EmptyMapLooksUpEverythingAsNullopt) {
    db::GtClassMap map{{}};
    EXPECT_TRUE(map.empty());
    EXPECT_FALSE(map.lookup("anything").has_value());
}

TEST(GtClassMap, ExactMatchMatchesOnlyExactName) {
    db::GtClassMap map({{"SM_Car_01", db::GtClassMap::Entry{2, "car"}}});
    EXPECT_EQ(map.size(), 1U);
    const auto hit = map.lookup("SM_Car_01");
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->class_id, 2U);
    EXPECT_EQ(hit->class_name, "car");
    // Prefix of the exact pattern must NOT match (no wildcard).
    EXPECT_FALSE(map.lookup("SM_Car_02").has_value());
    EXPECT_FALSE(map.lookup("SM_Car").has_value());
}

TEST(GtClassMap, TrailingWildcardMatchesPrefixes) {
    db::GtClassMap map({{"SK_Mannequin*", db::GtClassMap::Entry{0, "person"}}});
    EXPECT_TRUE(map.lookup("SK_Mannequin").has_value());     // exactly the prefix
    EXPECT_TRUE(map.lookup("SK_Mannequin_01").has_value());  // with suffix
    EXPECT_TRUE(map.lookup("SK_MannequinFoo").has_value());  // no separator required
    EXPECT_FALSE(map.lookup("Mannequin_01").has_value());    // prefix differs
    EXPECT_FALSE(map.lookup("SK_Man").has_value());          // shorter than pattern prefix
}

TEST(GtClassMap, FirstMatchWins) {
    db::GtClassMap map({
        {"SM_*", db::GtClassMap::Entry{99, "generic"}},
        {"SM_Car*", db::GtClassMap::Entry{2, "car"}},  // would also match "SM_Car_01"
    });
    const auto     hit = map.lookup("SM_Car_01");
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->class_id, 99U) << "first-registered pattern should win";
}

TEST(GtClassMap, LoadFromConfigReadsGtClassMapSection) {
    const auto map = load_map_from_json({
        {"SK_Mannequin*", {{"class_id", 0}, {"class_name", "person"}}},
        {"SM_Car*", {{"class_id", 2}, {"class_name", "car"}}},
        {"SM_TrafficCone*", {{"class_id", 10}, {"class_name", "obstacle"}}},
    });
    EXPECT_EQ(map.size(), 3U);

    const auto person = map.lookup("SK_Mannequin_01");
    ASSERT_TRUE(person.has_value());
    EXPECT_EQ(person->class_name, "person");

    const auto car = map.lookup("SM_Car_42");
    ASSERT_TRUE(car.has_value());
    EXPECT_EQ(car->class_id, 2U);

    EXPECT_FALSE(map.lookup("DroppedObject").has_value());
}

TEST(GtClassMap, LoadFromConfigReturnsEmptyWhenKeyAbsent) {
    drone::Config cfg;
    // No gt_class_map at all — load() should produce an empty map, not crash.
    const auto map = db::GtClassMap::load(cfg);
    EXPECT_TRUE(map.empty());
}

TEST(GtClassMap, LoadFromConfigSkipsMalformedEntries) {
    const auto map = load_map_from_json({
        {"Good*", {{"class_id", 7}, {"class_name", "thing"}}},
        {"NoClass", nlohmann::json("not-an-object")},
        {"MissingName", {{"class_id", 3}}},
    });
    EXPECT_EQ(map.size(), 1U);  // only "Good*" survives validation
    EXPECT_TRUE(map.lookup("Good_01").has_value());
}

TEST(GtClassMap, PatternsAccessorReturnsRegisteredPatterns) {
    db::GtClassMap map({
        {"SK_Mannequin*", {0, "person"}},
        {"SM_Car*", {2, "car"}},
    });
    const auto     patterns = map.patterns();
    ASSERT_EQ(patterns.size(), 2U);
    EXPECT_EQ(patterns[0], "SK_Mannequin*");
    EXPECT_EQ(patterns[1], "SM_Car*");
}

// ────────────────────────────────────────────────────────────────────────────
// JSONL serialisation
// ────────────────────────────────────────────────────────────────────────────

TEST(GtEmitterJson, SingleLineSerialisesAndParsesRoundTrip) {
    db::FrameGroundTruth f;
    f.timestamp_ns               = 1'700'000'000'000'000'000ULL;
    f.frame_sequence             = 42;
    f.camera_pose.translation[0] = 10.0F;
    f.camera_pose.translation[1] = 20.0F;
    f.camera_pose.translation[2] = 5.0F;
    f.camera_pose.quaternion[0]  = 1.0F;
    f.objects.push_back(
        db::GtDetection{0, "person", db::BBox2D{100, 200, 50, 80}, 0xdeadbeefU, 0.1F, 12.5F});
    f.objects.push_back(
        db::GtDetection{2, "car", db::BBox2D{400, 300, 120, 60}, 0xabcd1234U, 0.0F, 8.2F});

    const std::string line = db::to_json_line(f);

    // Single-line JSON — no embedded newlines.
    EXPECT_EQ(line.find('\n'), std::string::npos);

    // Parses cleanly and preserves every field.
    const auto j = nlohmann::json::parse(line);
    EXPECT_EQ(j["timestamp_ns"].get<uint64_t>(), f.timestamp_ns);
    EXPECT_EQ(j["frame_sequence"].get<uint64_t>(), 42U);
    ASSERT_TRUE(j.contains("camera_pose"));
    EXPECT_EQ(j["camera_pose"]["translation"][0].get<float>(), 10.0F);
    ASSERT_EQ(j["objects"].size(), 2U);
    EXPECT_EQ(j["objects"][0]["class_name"].get<std::string>(), "person");
    EXPECT_EQ(j["objects"][0]["gt_track_id"].get<uint32_t>(), 0xdeadbeefU);
    EXPECT_EQ(j["objects"][0]["bbox"]["x"].get<float>(), 100.0F);
    EXPECT_FLOAT_EQ(j["objects"][0]["occlusion"].get<float>(), 0.1F);
    EXPECT_FLOAT_EQ(j["objects"][1]["distance_m"].get<float>(), 8.2F);
}

TEST(GtEmitterJson, EmptyObjectsArraySerialisesCleanly) {
    db::FrameGroundTruth f;
    f.timestamp_ns = 1;
    const auto j   = nlohmann::json::parse(db::to_json_line(f));
    EXPECT_TRUE(j["objects"].is_array());
    EXPECT_EQ(j["objects"].size(), 0U);
}

TEST(GtEmitterJson, EscapesQuotesAndBackslashesInClassName) {
    // Defensive: class_name comes from scenario config. A tampered or
    // typo'd config shouldn't produce malformed JSON.
    db::FrameGroundTruth f;
    f.objects.push_back(
        db::GtDetection{0, "name with \"quotes\" and \\slash", db::BBox2D{0, 0, 1, 1}, 1, 0, 0});
    const std::string line = db::to_json_line(f);
    // Parser round-trips the escaped characters back to the original bytes.
    const auto j = nlohmann::json::parse(line);
    EXPECT_EQ(j["objects"][0]["class_name"].get<std::string>(), "name with \"quotes\" and \\slash");
}
