// common/util/include/util/config_validator.h
// Startup-time JSON config validation using a programmatic schema.
//
// Usage:
//   using namespace drone::cfg_key;
//   auto schema = ConfigSchema()
//       .required<int>(slam::VIO_RATE_HZ).range(1, 10000)
//       .required<double>(mission_planner::TAKEOFF_ALTITUDE_M).range(0.5, 500.0)
//       .optional<std::string>(LOG_LEVEL).one_of({"trace","debug","info","warn","error","critical"})
//       .required_section(video_capture::SECTION);
//
//   auto result = validate(cfg, schema);
//   if (result.is_err()) {
//       for (auto& msg : result.error()) DRONE_LOG_ERROR("{}", msg);
//       return EXIT_FAILURE;
//   }
#pragma once
#include "util/config.h"
#include "util/config_keys.h"
#include "util/result.h"

#include <cmath>
#include <functional>
#include <initializer_list>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace drone::util {

// ── Validation rule (type-erased) ───────────────────────────
// Each rule checks one aspect of the config and appends errors to a list.
using ValidationRule = std::function<void(const Config&, std::vector<std::string>&)>;

// ── Helper: walk a dot-separated key to a JSON node ─────────
inline const nlohmann::json* walk_key(const nlohmann::json& root, const std::string& key) {
    const auto* node  = &root;
    size_t      start = 0;
    while (start < key.size()) {
        size_t      dot  = key.find('.', start);
        std::string part = key.substr(start, dot - start);
        if (!node->is_object() || !node->contains(part)) return nullptr;
        node = &(*node)[part];
        if (dot == std::string::npos) break;
        start = dot + 1;
    }
    return node;
}

// ── Abstract base for type-erased field rules ──────────────
// Uses virtual dispatch (vtable) instead of shared_ptr to avoid
// atomic refcount overhead — critical for real-time drone software.

class IFieldRuleBase {
public:
    virtual ~IFieldRuleBase()                          = default;
    [[nodiscard]] virtual ValidationRule build() const = 0;
};

// ── Builder for a single field rule ─────────────────────────
// Fluent API: schema.required<int>("key").range(1, 100)
//             schema.optional<std::string>("key").one_of({"a","b"})

class ConfigSchema;  // forward

template<typename T>
class FieldRule : public IFieldRuleBase {
public:
    FieldRule(ConfigSchema& schema, std::string key, bool required)
        : schema_(schema), key_(std::move(key)), required_(required) {}

    /// Value must be in [lo, hi].
    FieldRule& range(T lo, T hi) {
        range_lo_  = lo;
        range_hi_  = hi;
        has_range_ = true;
        return *this;
    }

    /// Value must be one of the given set (string / integral types).
    FieldRule& one_of(std::initializer_list<T> values) {
        allowed_ = std::set<T>(values);
        return *this;
    }

    /// Value must satisfy a custom predicate.
    FieldRule& satisfies(std::function<bool(const T&)> pred, std::string description) {
        custom_pred_        = std::move(pred);
        custom_description_ = std::move(description);
        return *this;
    }

    /// Finalize and return the schema (for continued chaining on the next field).
    ConfigSchema& done() { return schema_; }

    // Build the actual validation rule (called internally by ConfigSchema).
    [[nodiscard]] ValidationRule build() const override {
        // Copy captures for the lambda
        auto key         = key_;
        auto required    = required_;
        auto has_range   = has_range_;
        auto range_lo    = range_lo_;
        auto range_hi    = range_hi_;
        auto allowed     = allowed_;
        auto custom_pred = custom_pred_;
        auto custom_desc = custom_description_;

        return [=](const Config& cfg, std::vector<std::string>& errors) {
            const auto* node = walk_key(cfg.raw(), key);

            // 1. Presence check
            if (node == nullptr) {
                if (required) {
                    errors.push_back("Missing required key: " + key);
                }
                return;  // Optional + missing → OK, skip further checks
            }

            // 2. Type check
            T val{};
            try {
                val = node->get<T>();
            } catch (...) {
                errors.push_back("Type mismatch for key '" + key + "': expected " + type_name() +
                                 ", got " + node->type_name());
                return;
            }

            // 3. Range check
            if (has_range) {
                if (val < range_lo || val > range_hi) {
                    std::ostringstream oss;
                    oss << "Out of range for '" << key << "': " << val << " (expected [" << range_lo
                        << ", " << range_hi << "])";
                    errors.push_back(oss.str());
                }
            }

            // 4. one_of check
            if (!allowed.empty()) {
                if (allowed.find(val) == allowed.end()) {
                    std::ostringstream oss;
                    oss << "Invalid value for '" << key << "': ";
                    oss << val << " (expected one of: ";
                    bool first = true;
                    for (const auto& a : allowed) {
                        if (!first) oss << ", ";
                        oss << a;
                        first = false;
                    }
                    oss << ")";
                    errors.push_back(oss.str());
                }
            }

            // 5. Custom predicate
            if (custom_pred && !custom_pred(val)) {
                errors.push_back("Validation failed for '" + key + "': " + custom_desc);
            }
        };
    }

private:
    static std::string type_name() {
        if constexpr (std::is_same_v<T, int>) return "integer";
        if constexpr (std::is_same_v<T, double>) return "number";
        if constexpr (std::is_same_v<T, float>) return "number";
        if constexpr (std::is_same_v<T, bool>) return "boolean";
        if constexpr (std::is_same_v<T, std::string>) return "string";
        return "unknown";
    }

    ConfigSchema&                 schema_;
    std::string                   key_;
    bool                          required_  = false;
    bool                          has_range_ = false;
    T                             range_lo_{};
    T                             range_hi_{};
    std::set<T>                   allowed_;
    std::function<bool(const T&)> custom_pred_;
    std::string                   custom_description_;
};

// ── ConfigSchema ────────────────────────────────────────────
// Builder-pattern schema definition.  Collects field rules and section
// presence checks.

class ConfigSchema {
public:
    /// Declare a required field with type T.
    template<typename T>
    FieldRule<T>& required(const std::string& key) {
        auto  uptr = std::make_unique<FieldRule<T>>(*this, key, true);
        auto& ref  = *uptr;
        field_rules_.push_back(std::move(uptr));
        return ref;
    }

    /// Declare an optional field with type T (validated if present).
    template<typename T>
    FieldRule<T>& optional(const std::string& key) {
        auto  uptr = std::make_unique<FieldRule<T>>(*this, key, false);
        auto& ref  = *uptr;
        field_rules_.push_back(std::move(uptr));
        return ref;
    }

    /// Require that a JSON section (object) exists.
    ConfigSchema& required_section(const std::string& key) {
        auto k = key;
        extra_rules_.emplace_back([k](const Config& cfg, std::vector<std::string>& errors) {
            if (!cfg.has(k)) {
                errors.push_back("Missing required section: " + k);
            }
        });
        return *this;
    }

    /// Add a fully custom validation rule.
    ConfigSchema& custom(ValidationRule rule) {
        extra_rules_.push_back(std::move(rule));
        return *this;
    }

    /// Build all rules into a flat list.
    [[nodiscard]] std::vector<ValidationRule> build() const {
        std::vector<ValidationRule> rules;
        rules.reserve(field_rules_.size() + extra_rules_.size());
        for (const auto& fr : field_rules_) {
            rules.push_back(fr->build());
        }
        for (const auto& r : extra_rules_) {
            rules.push_back(r);
        }
        return rules;
    }

private:
    // Owning storage for type-erased field rules.  unique_ptr gives
    // stable heap addresses so returned references remain valid even
    // when the vector reallocates.
    std::vector<std::unique_ptr<IFieldRuleBase>> field_rules_;
    std::vector<ValidationRule>                  extra_rules_;
};

// ── validate() — run a schema against a Config ──────────────
// Returns ok() if all rules pass, or err(vector of messages).

[[nodiscard]] inline Result<void, std::vector<std::string>> validate(const Config&       cfg,
                                                                     const ConfigSchema& schema) {
    std::vector<std::string> errors;
    auto                     rules = schema.build();
    for (auto& rule : rules) {
        rule(cfg, errors);
    }
    if (errors.empty()) {
        return Result<void, std::vector<std::string>>::ok();
    }
    return Result<void, std::vector<std::string>>::err(std::move(errors));
}

// ── Common schemas ──────────────────────────────────────────
// Pre-built schemas for each process.  Call drone::util::common_schema()
// to get the shared rules, then add process-specific ones.

inline ConfigSchema common_schema() {
    ConfigSchema s;
    s.optional<std::string>(cfg_key::LOG_LEVEL)
        .one_of({"trace", "debug", "info", "warn", "error", "critical"});
    s.optional<std::string>(cfg_key::IPC_BACKEND).one_of({"zenoh"});  // "shm" removed in PR #151
    // vehicle_id must match TopicResolver's allowed chars: [a-zA-Z0-9_-] or empty
    s.optional<std::string>(cfg_key::VEHICLE_ID)
        .satisfies(
            [](const std::string& id) {
                return std::all_of(id.begin(), id.end(), [](char c) {
                    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                           (c >= '0' && c <= '9') || c == '_' || c == '-';
                });
            },
            "must be alphanumeric, dash, or underscore only (or empty)");
    return s;
}

inline ConfigSchema video_capture_schema() {
    auto s = common_schema();
    s.required_section(cfg_key::video_capture::SECTION);
    s.required<int>(cfg_key::video_capture::mission_cam::WIDTH).range(1, 7680);
    s.required<int>(cfg_key::video_capture::mission_cam::HEIGHT).range(1, 4320);
    s.required<int>(cfg_key::video_capture::mission_cam::FPS).range(1, 240);
    s.optional<std::string>(cfg_key::video_capture::mission_cam::BACKEND)
        .one_of({"simulated", "v4l2", "libargus", "gazebo"});
    s.optional<int>(cfg_key::video_capture::stereo_cam::WIDTH).range(1, 7680);
    s.optional<int>(cfg_key::video_capture::stereo_cam::HEIGHT).range(1, 4320);
    s.optional<int>(cfg_key::video_capture::stereo_cam::FPS).range(1, 240);
    s.optional<std::string>(cfg_key::video_capture::stereo_cam::BACKEND)
        .one_of({"simulated", "v4l2", "libargus", "gazebo"});
    return s;
}

inline ConfigSchema perception_schema() {
    auto s = common_schema();
    s.required_section(cfg_key::perception::SECTION);
    s.optional<double>(cfg_key::perception::detector::CONFIDENCE_THRESHOLD).range(0.0, 1.0);
    s.optional<double>(cfg_key::perception::detector::NMS_THRESHOLD).range(0.0, 1.0);
    s.optional<int>(cfg_key::perception::detector::MAX_DETECTIONS).range(1, 10000);
    s.optional<int>(cfg_key::perception::detector::MIN_CONTOUR_AREA).range(0, 100000);
    s.optional<int>(cfg_key::perception::detector::SUBSAMPLE).range(1, 16);
    s.optional<int>(cfg_key::perception::detector::MAX_FPS).range(0, 1000);
    s.optional<double>(cfg_key::perception::detector::CONFIDENCE_MAX).range(0.0, 1.0);
    s.optional<double>(cfg_key::perception::detector::CONFIDENCE_BASE).range(0.0, 1.0);
    s.optional<double>(cfg_key::perception::detector::CONFIDENCE_AREA_SCALE).range(0.0, 10000.0);
    s.optional<int>(cfg_key::perception::detector::MIN_BBOX_HEIGHT_PX).range(1, 10000);
    s.optional<double>(cfg_key::perception::detector::MAX_ASPECT_RATIO).range(0.1, 100.0);
    // Tracker
    s.optional<int>(cfg_key::perception::tracker::MAX_AGE).range(1, 1000);
    s.optional<int>(cfg_key::perception::tracker::MIN_HITS).range(1, 100);
    s.optional<double>(cfg_key::perception::tracker::MAX_ASSOCIATION_COST).range(0.0, 10000.0);
    s.optional<double>(cfg_key::perception::tracker::HIGH_CONF_THRESHOLD).range(0.0, 1.0);
    s.optional<double>(cfg_key::perception::tracker::LOW_CONF_THRESHOLD).range(0.0, 1.0);
    s.optional<double>(cfg_key::perception::tracker::MAX_IOU_COST).range(0.0, 1.0);
    // Radar
    s.optional<bool>(cfg_key::perception::radar::ENABLED);
    s.optional<int>(cfg_key::perception::radar::UPDATE_RATE_HZ).range(1, 1000);
    // Fusion
    s.optional<int>(cfg_key::perception::fusion::RATE_HZ).range(1, 1000);
    return s;
}

inline ConfigSchema slam_schema() {
    auto s = common_schema();
    s.required_section(cfg_key::slam::SECTION);
    s.required<int>(cfg_key::slam::VIO_RATE_HZ).range(1, 10000);
    s.optional<int>(cfg_key::slam::VISUAL_FRONTEND_RATE_HZ).range(1, 1000);
    s.optional<int>(cfg_key::slam::IMU_RATE_HZ).range(1, 10000);
    s.optional<double>(cfg_key::slam::keyframe::MIN_PARALLAX_PX).range(0.0, 1000.0);
    s.optional<double>(cfg_key::slam::keyframe::MIN_TRACKED_RATIO).range(0.0, 1.0);
    s.optional<double>(cfg_key::slam::keyframe::MAX_TIME_SEC).range(0.001, 60.0);
    // VIO quality thresholds
    s.optional<double>(cfg_key::slam::vio::GOOD_TRACE_MAX).range(0.0, 100.0);
    s.optional<double>(cfg_key::slam::vio::DEGRADED_TRACE_MAX).range(0.0, 100.0);
    return s;
}

inline ConfigSchema mission_planner_schema() {
    auto s = common_schema();
    s.required_section(cfg_key::mission_planner::SECTION);
    s.required<int>(cfg_key::mission_planner::UPDATE_RATE_HZ).range(1, 1000);
    s.required<double>(cfg_key::mission_planner::TAKEOFF_ALTITUDE_M).range(0.5, 500.0);
    s.optional<double>(cfg_key::mission_planner::ACCEPTANCE_RADIUS_M).range(0.1, 100.0);
    s.optional<double>(cfg_key::mission_planner::CRUISE_SPEED_MPS).range(0.1, 50.0);
    s.optional<double>(cfg_key::mission_planner::RTL_ACCEPTANCE_RADIUS_M).range(0.1, 100.0);
    s.optional<double>(cfg_key::mission_planner::LANDED_ALTITUDE_M).range(0.0, 50.0);
    s.optional<int>(cfg_key::mission_planner::RTL_MIN_DWELL_SECONDS).range(0, 600);
    // Path planner
    s.optional<double>(cfg_key::mission_planner::path_planner::RAMP_DIST_M).range(0.0, 100.0);
    s.optional<double>(cfg_key::mission_planner::path_planner::MIN_SPEED_MPS).range(0.0, 50.0);
    s.optional<int>(cfg_key::mission_planner::path_planner::SNAP_SEARCH_RADIUS).range(1, 100);
    s.optional<double>(cfg_key::mission_planner::path_planner::YAW_SMOOTHING_RATE).range(0.0, 1.0);
    s.optional<double>(cfg_key::mission_planner::path_planner::SNAP_APPROACH_BIAS).range(0.0, 1.0);
    // Occupancy grid
    s.optional<double>(cfg_key::mission_planner::occupancy_grid::RESOLUTION_M).range(0.01, 10.0);
    s.optional<double>(cfg_key::mission_planner::occupancy_grid::INFLATION_RADIUS_M)
        .range(0.0, 50.0);
    s.optional<double>(cfg_key::mission_planner::occupancy_grid::DYNAMIC_OBSTACLE_TTL_S)
        .range(0.0, 60.0);
    s.optional<double>(cfg_key::mission_planner::occupancy_grid::MIN_CONFIDENCE).range(0.0, 1.0);
    s.optional<double>(cfg_key::mission_planner::occupancy_grid::MIN_PROMOTION_DEPTH_CONFIDENCE)
        .range(0.0, 1.0);
    s.optional<int>(cfg_key::mission_planner::occupancy_grid::MAX_STATIC_CELLS).range(0, 1000000);
    s.optional<bool>(cfg_key::mission_planner::occupancy_grid::PREDICTION_ENABLED);
    s.optional<double>(cfg_key::mission_planner::occupancy_grid::PREDICTION_DT_S).range(0.0, 60.0);
    // Obstacle avoidance
    s.optional<double>(cfg_key::mission_planner::obstacle_avoidance::MIN_DISTANCE_M)
        .range(0.1, 100.0);
    s.optional<double>(cfg_key::mission_planner::obstacle_avoidance::INFLUENCE_RADIUS_M)
        .range(0.1, 100.0);
    s.optional<double>(cfg_key::mission_planner::obstacle_avoidance::REPULSIVE_GAIN)
        .range(0.0, 100.0);
    s.optional<double>(cfg_key::mission_planner::obstacle_avoidance::MAX_CORRECTION_MPS)
        .range(0.0, 50.0);
    s.optional<double>(cfg_key::mission_planner::obstacle_avoidance::MIN_CONFIDENCE).range(0.0, 1.0);
    s.optional<double>(cfg_key::mission_planner::obstacle_avoidance::PREDICTION_DT_S)
        .range(0.0, 60.0);
    s.optional<int>(cfg_key::mission_planner::obstacle_avoidance::MAX_AGE_MS).range(0, 60000);
    // Geofence
    s.optional<double>(cfg_key::mission_planner::geofence::ALTITUDE_FLOOR_M).range(-1000.0, 10000.0);
    s.optional<double>(cfg_key::mission_planner::geofence::ALTITUDE_CEILING_M).range(0.0, 10000.0);
    s.optional<double>(cfg_key::mission_planner::geofence::WARNING_MARGIN_M).range(0.0, 1000.0);
    s.optional<double>(cfg_key::mission_planner::geofence::ALTITUDE_TOLERANCE_M).range(0.0, 100.0);
    // Collision recovery
    s.optional<bool>(cfg_key::mission_planner::collision_recovery::ENABLED);
    s.optional<double>(cfg_key::mission_planner::collision_recovery::CLIMB_DELTA_M)
        .range(0.0, 100.0);
    s.optional<double>(cfg_key::mission_planner::collision_recovery::HOVER_DURATION_S)
        .range(0.0, 60.0);
    // fault_manager section
    s.optional<int>(cfg_key::fault_manager::POSE_STALE_TIMEOUT_MS).range(1, 60000);
    s.optional<double>(cfg_key::fault_manager::BATTERY_WARN_PERCENT).range(1.0, 100.0);
    s.optional<double>(cfg_key::fault_manager::BATTERY_CRIT_PERCENT).range(0.0, 100.0);
    return s;
}

inline ConfigSchema comms_schema() {
    auto s = common_schema();
    s.required_section(cfg_key::comms::SECTION);
    s.optional<int>(cfg_key::comms::mavlink::HEARTBEAT_RATE_HZ).range(1, 100);
    s.optional<int>(cfg_key::comms::mavlink::TX_RATE_HZ).range(1, 1000);
    s.optional<int>(cfg_key::comms::mavlink::RX_RATE_HZ).range(1, 1000);
    s.optional<int>(cfg_key::comms::mavlink::TIMEOUT_MS).range(1, 60000);
    s.optional<int>(cfg_key::comms::mavlink::BAUD_RATE).range(1200, 4000000);
    s.optional<int>(cfg_key::comms::gcs::UDP_PORT).range(1, 65535);
    s.optional<int>(cfg_key::comms::gcs::TELEMETRY_RATE_HZ).range(1, 100);
    return s;
}

inline ConfigSchema payload_manager_schema() {
    auto s = common_schema();
    s.required_section(cfg_key::payload_manager::SECTION);
    s.required<int>(cfg_key::payload_manager::UPDATE_RATE_HZ).range(1, 1000);
    s.optional<double>(cfg_key::payload_manager::gimbal::MAX_SLEW_RATE_DPS).range(0.1, 1000.0);
    s.optional<double>(cfg_key::payload_manager::gimbal::PITCH_MIN_DEG).range(-180.0, 0.0);
    s.optional<double>(cfg_key::payload_manager::gimbal::PITCH_MAX_DEG).range(0.0, 180.0);
    s.optional<double>(cfg_key::payload_manager::gimbal::YAW_MIN_DEG).range(-360.0, 0.0);
    s.optional<double>(cfg_key::payload_manager::gimbal::YAW_MAX_DEG).range(0.0, 360.0);
    s.optional<bool>(cfg_key::payload_manager::gimbal::auto_track::ENABLED);
    s.optional<double>(cfg_key::payload_manager::gimbal::auto_track::MIN_CONFIDENCE).range(0.0, 1.0);
    return s;
}

inline ConfigSchema system_monitor_schema() {
    auto s = common_schema();
    s.required_section(cfg_key::system_monitor::SECTION);
    s.optional<std::string>(cfg_key::system_monitor::PLATFORM).one_of({"linux", "jetson", "mock"});
    s.required<int>(cfg_key::system_monitor::UPDATE_RATE_HZ).range(1, 100);
    s.optional<int>(cfg_key::system_monitor::DISK_CHECK_INTERVAL_S).range(1, 3600);
    s.optional<double>(cfg_key::system_monitor::POWER_COEFF).range(0.0, 10.0);
    s.optional<double>(cfg_key::system_monitor::thresholds::CPU_WARN_PERCENT).range(0.0, 100.0);
    s.optional<double>(cfg_key::system_monitor::thresholds::MEM_WARN_PERCENT).range(0.0, 100.0);
    s.optional<double>(cfg_key::system_monitor::thresholds::TEMP_WARN_C).range(0.0, 200.0);
    s.optional<double>(cfg_key::system_monitor::thresholds::TEMP_CRIT_C).range(0.0, 200.0);
    s.optional<double>(cfg_key::system_monitor::thresholds::BATTERY_WARN_PERCENT).range(0.0, 100.0);
    s.optional<double>(cfg_key::system_monitor::thresholds::BATTERY_CRIT_PERCENT).range(0.0, 100.0);
    s.optional<double>(cfg_key::system_monitor::thresholds::DISK_CRIT_PERCENT).range(0.0, 100.0);
    return s;
}

// ── validate_or_exit() — validate and log errors ───────────
// Returns 0 on success, 1 on validation failure (logs errors via DRONE_LOG_ERROR).
// Catches exceptions from malformed configs (defense-in-depth).
[[nodiscard]] inline int validate_or_exit(const Config& cfg, const ConfigSchema& schema) {
    try {
        auto validation = validate(cfg, schema);
        if (!validation.is_ok()) {
            for (const auto& err : validation.error()) {
                DRONE_LOG_ERROR("[Config] {}", err);
            }
            DRONE_LOG_ERROR("Config validation failed — exiting");
            return 1;
        }
    } catch (const std::exception& ex) {
        DRONE_LOG_ERROR("Config validation error: {}", ex.what());
        DRONE_LOG_ERROR("Config validation failed — exiting");
        return 1;
    }
    return 0;
}

}  // namespace drone::util
