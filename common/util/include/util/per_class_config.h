// common/util/include/util/per_class_config.h
// Per-class config lookup utility — loads JSON sections like
//   { "default": 5.0, "person": 2.0, "drone": 4.0 }
// into std::array<T, 8> indexed by ObjectClass enum values.
#pragma once

#include "util/config.h"
#include "util/ilogger.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace drone::util {

inline constexpr uint8_t kPerClassCount = 8;

inline constexpr std::array<const char*, kPerClassCount> kClassName = {
    "unknown", "person", "vehicle_car", "vehicle_truck", "drone", "animal", "building", "tree",
};

/// Map a JSON class name to its index (0-7), or -1 if unrecognized.
inline int class_name_to_index(const std::string& name) {
    for (uint8_t i = 0; i < kPerClassCount; ++i) {
        if (name == kClassName[i]) return static_cast<int>(i);
    }
    return -1;
}

/// Load a per-class array from a config section.
///
/// The section at `section_key` should be a JSON object with class name keys
/// and a "default" fallback.  Unspecified classes get the "default" value;
/// if "default" is absent, they get `fallback`.
///
/// Keys prefixed with "_comment" are silently skipped (convention for JSON
/// comments).  Unrecognized class names are collected into `unknown_keys`
/// if non-null.
template<typename T>
std::array<T, kPerClassCount> load_per_class(const Config& cfg, const std::string& section_key,
                                             const T&                  fallback,
                                             std::vector<std::string>* unknown_keys = nullptr) {
    std::array<T, kPerClassCount> result;
    result.fill(fallback);

    auto section = cfg.section(section_key);
    if (!section.is_object() || section.empty()) return result;

    // Read "default" first — overrides the compiled fallback for all classes.
    T base = fallback;
    if (section.contains("default")) {
        try {
            base = section["default"].get<T>();
        } catch (...) {
            DRONE_LOG_WARN("[per_class_config] Type mismatch for 'default' in section '{}'",
                           section_key);
        }
    }
    result.fill(base);

    // Per-class overrides.
    for (auto it = section.begin(); it != section.end(); ++it) {
        const auto& key = it.key();
        if (key == "default") continue;
        if (key.rfind("_comment", 0) == 0) continue;

        int idx = class_name_to_index(key);
        if (idx < 0) {
            if (unknown_keys) unknown_keys->push_back(key);
            continue;
        }
        try {
            result[static_cast<uint8_t>(idx)] = it.value().get<T>();
        } catch (...) {
            DRONE_LOG_WARN("[per_class_config] Type mismatch for '{}' in section '{}'", key,
                           section_key);
        }
    }

    return result;
}

/// Validate that a per-class config section contains only recognized keys.
/// Returns a list of error messages for unrecognized keys (empty = valid).
inline std::vector<std::string> validate_per_class_section(const Config&      cfg,
                                                           const std::string& section_key) {
    std::vector<std::string> errors;
    auto                     section = cfg.section(section_key);
    if (!section.is_object()) return errors;

    for (auto it = section.begin(); it != section.end(); ++it) {
        const auto& key = it.key();
        if (key == "default") continue;
        if (key.rfind("_comment", 0) == 0) continue;
        if (class_name_to_index(key) < 0) {
            errors.push_back("Unknown class name '" + key + "' in config section '" + section_key +
                             "'");
        }
    }
    return errors;
}

}  // namespace drone::util
