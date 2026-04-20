// tests/benchmark/gt_emitter.cpp
//
// Shared implementation for the GT emitter — class-map parsing and JSON
// serialisation. Backend-specific emitters live in their own .cpp files
// (gated behind HAVE_COSYS_AIRSIM / HAVE_GAZEBO).

#include "benchmark/gt_emitter.h"

#include "util/config.h"

#include <sstream>

#include <nlohmann/json.hpp>

namespace drone::benchmark {

// ────────────────────────────────────────────────────────────────────────────
// GtClassMap
// ────────────────────────────────────────────────────────────────────────────

GtClassMap::GtClassMap(std::vector<PatternEntry> patterns) : patterns_(std::move(patterns)) {}

GtClassMap GtClassMap::load(const drone::Config& scenario_cfg) {
    // Scenario configs declare `gt_class_map` as a top-level key. If absent,
    // the returned section is an empty object — we produce an empty map,
    // which disables GT emission for the scenario (callers short-circuit).
    const auto map_json = scenario_cfg.section("gt_class_map");
    if (!map_json.is_object() || map_json.empty()) {
        return GtClassMap{{}};
    }

    std::vector<PatternEntry> patterns;
    patterns.reserve(map_json.size());
    for (auto it = map_json.begin(); it != map_json.end(); ++it) {
        if (!it.value().is_object()) continue;
        const auto& v = it.value();
        if (!v.contains("class_id") || !v.contains("class_name")) continue;
        PatternEntry pe;
        pe.pattern          = it.key();
        pe.entry.class_id   = v.value("class_id", 0U);
        pe.entry.class_name = v.value("class_name", std::string{});
        patterns.push_back(std::move(pe));
    }
    return GtClassMap{std::move(patterns)};
}

std::vector<std::string> GtClassMap::patterns() const {
    std::vector<std::string> out;
    out.reserve(patterns_.size());
    for (const auto& pe : patterns_) {
        out.push_back(pe.pattern);
    }
    return out;
}

std::optional<GtClassMap::Entry> GtClassMap::lookup(std::string_view object_name) const {
    for (const auto& pe : patterns_) {
        // Trailing-`*` glob: pattern "SK_Mannequin*" matches any name that
        // starts with "SK_Mannequin". A pattern with no `*` is an exact match.
        const bool has_wildcard = !pe.pattern.empty() && pe.pattern.back() == '*';
        if (has_wildcard) {
            const std::string_view prefix{pe.pattern.data(), pe.pattern.size() - 1};
            if (object_name.size() >= prefix.size() &&
                object_name.compare(0, prefix.size(), prefix) == 0) {
                return pe.entry;
            }
        } else if (object_name == pe.pattern) {
            return pe.entry;
        }
    }
    return std::nullopt;
}

// ────────────────────────────────────────────────────────────────────────────
// JSONL serialisation
// ────────────────────────────────────────────────────────────────────────────

std::string to_json_line(const FrameGroundTruth& frame) {
    // Build via nlohmann::json for correctness (handles string escaping) but
    // serialise via dump(-1) for compact single-line output — matches the
    // #570 perception_metrics JSON style and parses cheaply line-by-line.
    nlohmann::json j;
    j["timestamp_ns"]   = frame.timestamp_ns;
    j["frame_sequence"] = frame.frame_sequence;

    nlohmann::json pose;
    pose["translation"] = {frame.camera_pose.translation[0], frame.camera_pose.translation[1],
                           frame.camera_pose.translation[2]};
    pose["quaternion"]  = {frame.camera_pose.quaternion[0], frame.camera_pose.quaternion[1],
                           frame.camera_pose.quaternion[2], frame.camera_pose.quaternion[3]};
    j["camera_pose"]    = std::move(pose);

    nlohmann::json objects = nlohmann::json::array();
    for (const auto& o : frame.objects) {
        nlohmann::json obj;
        obj["class_id"]    = o.class_id;
        obj["class_name"]  = o.class_name;
        obj["gt_track_id"] = o.gt_track_id;
        obj["bbox"]        = {{"x", o.bbox.x}, {"y", o.bbox.y}, {"w", o.bbox.w}, {"h", o.bbox.h}};
        obj["occlusion"]   = o.occlusion;
        obj["distance_m"]  = o.distance_m;
        objects.push_back(std::move(obj));
    }
    j["objects"] = std::move(objects);

    return j.dump();  // compact, single line, no trailing newline
}

}  // namespace drone::benchmark
