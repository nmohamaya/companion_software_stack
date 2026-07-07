// tests/test_contracts_manifest.cpp — wire-contract drift gate (Issue #806 / ADR-017)
//
// Holds three artifacts mutually consistent:
//   1. the compiled wire structs + kWireVersion   (common/ipc — the code SSOT)
//   2. contracts/topics.json                      (the reviewed, machine-readable manifest)
//   3. kBinding below                             (the topic ↔ struct ↔ sizeof witness table)
//
// A wire-struct size change, topic rename, or kWireVersion bump that doesn't
// consciously touch contracts/ fails here instead of shipping silent ABI drift.
// Policy for making these tests green again: contracts/VERSIONING.md.

#include "ipc/ipc_types.h"
#include "ipc/wire_format.h"

#include <fstream>
#include <map>
#include <optional>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace {

using drone::ipc::topics::DETECTED_OBJECTS;
using drone::ipc::topics::FAULT_OVERRIDES;
using drone::ipc::topics::FC_COMMANDS;
using drone::ipc::topics::FC_STATE;
using drone::ipc::topics::GCS_COMMANDS;
using drone::ipc::topics::MISSION_STATUS;
using drone::ipc::topics::MISSION_UPLOAD;
using drone::ipc::topics::PAYLOAD_COMMANDS;
using drone::ipc::topics::PAYLOAD_STATUS;
using drone::ipc::topics::RADAR_DETECTIONS;
using drone::ipc::topics::SEMANTIC_VOXELS;
using drone::ipc::topics::SLAM_POSE;
using drone::ipc::topics::SYSTEM_HEALTH;
using drone::ipc::topics::TRAJECTORY_CMD;
using drone::ipc::topics::VIDEO_MISSION_CAM;
using drone::ipc::topics::VIDEO_STEREO_CAM;

struct Binding {
    const char* topic;
    const char* struct_name;
    std::size_t compiled_size;
};

// The witness table: which struct each topic carries, bound to the *compiled*
// sizeof. Topic strings come from topics:: so a constant rename breaks the
// build here, not silently in the manifest. New topics register here AND in
// contracts/topics.json (VERSIONING.md rule 5) — the completeness tests below
// hold the two lists identical in both directions.
const Binding kBinding[] = {
    {VIDEO_MISSION_CAM, "VideoFrame", sizeof(drone::ipc::VideoFrame)},
    {VIDEO_STEREO_CAM, "StereoFrame", sizeof(drone::ipc::StereoFrame)},
    {DETECTED_OBJECTS, "DetectedObjectList", sizeof(drone::ipc::DetectedObjectList)},
    {SEMANTIC_VOXELS, "SemanticVoxelBatch", sizeof(drone::ipc::SemanticVoxelBatch)},
    {SLAM_POSE, "Pose", sizeof(drone::ipc::Pose)},
    {MISSION_STATUS, "MissionStatus", sizeof(drone::ipc::MissionStatus)},
    {TRAJECTORY_CMD, "TrajectoryCmd", sizeof(drone::ipc::TrajectoryCmd)},
    {PAYLOAD_COMMANDS, "PayloadCommand", sizeof(drone::ipc::PayloadCommand)},
    {FC_COMMANDS, "FCCommand", sizeof(drone::ipc::FCCommand)},
    {FC_STATE, "FCState", sizeof(drone::ipc::FCState)},
    {GCS_COMMANDS, "GCSCommand", sizeof(drone::ipc::GCSCommand)},
    {MISSION_UPLOAD, "MissionUpload", sizeof(drone::ipc::MissionUpload)},
    {PAYLOAD_STATUS, "PayloadStatus", sizeof(drone::ipc::PayloadStatus)},
    {SYSTEM_HEALTH, "SystemHealth", sizeof(drone::ipc::SystemHealth)},
    {FAULT_OVERRIDES, "FaultOverrides", sizeof(drone::ipc::FaultOverrides)},
    {RADAR_DETECTIONS, "RadarDetectionList", sizeof(drone::ipc::RadarDetectionList)},
    {drone::ipc::topics::THREAD_HEALTH_VIDEO_CAPTURE, "ThreadHealth",
     sizeof(drone::ipc::ThreadHealth)},
    {drone::ipc::topics::THREAD_HEALTH_PERCEPTION, "ThreadHealth",
     sizeof(drone::ipc::ThreadHealth)},
    {drone::ipc::topics::THREAD_HEALTH_SLAM_VIO_NAV, "ThreadHealth",
     sizeof(drone::ipc::ThreadHealth)},
    {drone::ipc::topics::THREAD_HEALTH_MISSION_PLANNER, "ThreadHealth",
     sizeof(drone::ipc::ThreadHealth)},
    {drone::ipc::topics::THREAD_HEALTH_COMMS, "ThreadHealth", sizeof(drone::ipc::ThreadHealth)},
    {drone::ipc::topics::THREAD_HEALTH_PAYLOAD_MANAGER, "ThreadHealth",
     sizeof(drone::ipc::ThreadHealth)},
    {drone::ipc::topics::THREAD_HEALTH_SYSTEM_MONITOR, "ThreadHealth",
     sizeof(drone::ipc::ThreadHealth)},
};

std::string manifest_path() {
#ifdef PROJECT_CONTRACTS_DIR
    return std::string(PROJECT_CONTRACTS_DIR) + "/topics.json";
#else
    return "contracts/topics.json";  // fallback: run from repo root
#endif
}

// Loads + shape-validates the manifest once, cached by value (no heap handoff).
// Returns nullptr on any I/O, parse, or shape violation so every test fails
// with a clean assertion; the up-front key/type checks also mean the .at()
// calls in the tests below can never throw on a malformed manifest.
const nlohmann::json* load_manifest() {
    static const std::optional<nlohmann::json> manifest = []() -> std::optional<nlohmann::json> {
        std::ifstream in(manifest_path());
        if (!in.is_open()) {
            return std::nullopt;
        }
        auto parsed = nlohmann::json::parse(in, /*cb=*/nullptr, /*allow_exceptions=*/false);
        if (parsed.is_discarded() || !parsed.is_object()) {
            return std::nullopt;
        }
        if (!parsed.contains("kWireVersion") || !parsed["kWireVersion"].is_number_unsigned()) {
            return std::nullopt;
        }
        if (!parsed.contains("topics") || !parsed["topics"].is_array()) {
            return std::nullopt;
        }
        for (const auto& entry : parsed["topics"]) {
            if (!entry.is_object() || !entry.contains("topic") || !entry["topic"].is_string() ||
                !entry.contains("struct") || !entry["struct"].is_string() ||
                !entry.contains("sizeof_bytes") || !entry["sizeof_bytes"].is_number_unsigned()) {
                return std::nullopt;
            }
        }
        return parsed;
    }();
    return manifest ? &*manifest : nullptr;
}

// topic → manifest entry, keyed for the per-entry tests (23 entries — lookup
// cost is irrelevant; the map is for readable membership checks).
std::map<std::string, nlohmann::json> manifest_by_topic(const nlohmann::json& m) {
    std::map<std::string, nlohmann::json> out;
    for (const auto& entry : m.at("topics")) {
        out[entry.at("topic").get<std::string>()] = entry;
    }
    return out;
}

TEST(ContractsManifest, ManifestFileParses) {
    const auto* m = load_manifest();
    ASSERT_NE(m, nullptr) << "contracts/topics.json missing, invalid JSON, or wrong shape "
                             "(kWireVersion unsigned + topics[] of {topic,struct,sizeof_bytes}) at "
                          << manifest_path();
    ASSERT_TRUE(m->contains("kWireVersion"));
    ASSERT_TRUE(m->contains("topics"));
    ASSERT_FALSE(m->at("topics").empty());
}

TEST(ContractsManifest, WireVersionMatchesCode) {
    const auto* m = load_manifest();
    ASSERT_NE(m, nullptr);
    // A kWireVersion bump must be a conscious contracts/ change (VERSIONING.md rule 1).
    EXPECT_EQ(m->at("kWireVersion").get<unsigned>(), static_cast<unsigned>(drone::ipc::kWireVersion))
        << "kWireVersion changed in ipc/wire_format.h — update contracts/topics.json in the "
           "same commit (contracts/VERSIONING.md rule 1)";
}

TEST(ContractsManifest, EveryCodeTopicPresentInManifest) {
    const auto* m = load_manifest();
    ASSERT_NE(m, nullptr);
    const auto by_topic = manifest_by_topic(*m);
    for (const auto& b : kBinding) {
        EXPECT_TRUE(by_topic.count(b.topic))
            << "topic " << b.topic << " exists in code (topics::) but not in contracts/topics.json";
    }
}

TEST(ContractsManifest, NoUnknownTopicsInManifest) {
    const auto* m = load_manifest();
    ASSERT_NE(m, nullptr);
    for (const auto& entry : m->at("topics")) {
        const auto topic = entry.at("topic").get<std::string>();
        bool       known = false;
        for (const auto& b : kBinding) {
            known = known || (topic == b.topic);
        }
        EXPECT_TRUE(known) << "manifest topic " << topic
                           << " has no binding in test_contracts_manifest.cpp — removed/renamed "
                              "in code, or missing from the witness table (VERSIONING.md rule 5)";
    }
}

TEST(ContractsManifest, StructNamesMatchBinding) {
    const auto* m = load_manifest();
    ASSERT_NE(m, nullptr);
    const auto by_topic = manifest_by_topic(*m);
    for (const auto& b : kBinding) {
        auto it = by_topic.find(b.topic);
        if (it == by_topic.end()) {
            continue;  // reported by EveryCodeTopicPresentInManifest
        }
        EXPECT_EQ(it->second.at("struct").get<std::string>(), b.struct_name)
            << "topic " << b.topic << " carries a different struct than the manifest claims";
    }
}

TEST(ContractsManifest, StructSizesMatchCompiledStructs) {
    const auto* m = load_manifest();
    ASSERT_NE(m, nullptr);
    const auto by_topic = manifest_by_topic(*m);
    for (const auto& b : kBinding) {
        auto it = by_topic.find(b.topic);
        if (it == by_topic.end()) {
            continue;  // reported by EveryCodeTopicPresentInManifest
        }
        // THE drift gate: a field addition/removal changes sizeof and must land
        // together with a manifest update + kWireVersion bump (VERSIONING.md rule 1).
        // sizeof is part of the public IPC ABI until #806 Phase 3.
        EXPECT_EQ(it->second.at("sizeof_bytes").get<std::size_t>(), b.compiled_size)
            << "sizeof(" << b.struct_name << ") drifted from contracts/topics.json for topic "
            << b.topic << " — wire-format change without a contracts/ update";
    }
}

}  // namespace
