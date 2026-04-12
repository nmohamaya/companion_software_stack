// common/util/include/util/config_validator.h
// Startup-time JSON config validation using a programmatic schema.
//
// Usage:
//   auto schema = ConfigSchema()
//       .required<int>("slam.vio_rate_hz").range(1, 10000)
//       .required<double>("mission_planner.takeoff_altitude_m").range(0.5, 500.0)
//       .optional<std::string>("log_level").one_of({"trace","debug","info","warn","error","critical"})
//       .required_section("video_capture");
//
//   auto result = validate(cfg, schema);
//   if (result.is_err()) {
//       for (auto& msg : result.error()) DRONE_LOG_ERROR("{}", msg);
//       return EXIT_FAILURE;
//   }
#pragma once
#include "util/config.h"
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
    s.optional<std::string>("log_level")
        .one_of({"trace", "debug", "info", "warn", "error", "critical"});
    s.optional<std::string>("ipc_backend").one_of({"zenoh"});  // "shm" removed in PR #151
    // vehicle_id must match TopicResolver's allowed chars: [a-zA-Z0-9_-] or empty
    s.optional<std::string>("vehicle_id")
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
    s.required_section("video_capture");
    s.required<int>("video_capture.mission_cam.width").range(1, 7680);
    s.required<int>("video_capture.mission_cam.height").range(1, 4320);
    s.required<int>("video_capture.mission_cam.fps").range(1, 240);
    s.optional<std::string>("video_capture.mission_cam.backend")
        .one_of({"simulated", "v4l2", "libargus", "gazebo"});
    s.optional<int>("video_capture.stereo_cam.width").range(1, 7680);
    s.optional<int>("video_capture.stereo_cam.height").range(1, 4320);
    s.optional<int>("video_capture.stereo_cam.fps").range(1, 240);
    s.optional<std::string>("video_capture.stereo_cam.backend")
        .one_of({"simulated", "v4l2", "libargus", "gazebo"});
    return s;
}

inline ConfigSchema perception_schema() {
    auto s = common_schema();
    s.required_section("perception");
    // Detector
    s.optional<double>("perception.detector.confidence_threshold").range(0.0, 1.0);
    s.optional<double>("perception.detector.nms_threshold").range(0.0, 1.0);
    s.optional<int>("perception.detector.max_detections").range(1, 10000);
    s.optional<int>("perception.detector.min_contour_area").range(0, 100000);
    s.optional<int>("perception.detector.subsample").range(1, 16);
    s.optional<int>("perception.detector.max_fps").range(0, 1000);
    s.optional<double>("perception.detector.confidence_max").range(0.0, 1.0);
    s.optional<double>("perception.detector.confidence_base").range(0.0, 1.0);
    s.optional<double>("perception.detector.confidence_area_scale").range(0.0, 10000.0);
    s.optional<int>("perception.detector.min_bbox_height_px").range(1, 10000);
    s.optional<double>("perception.detector.max_aspect_ratio").range(0.1, 100.0);
    // Tracker
    s.optional<int>("perception.tracker.max_age").range(1, 1000);
    s.optional<int>("perception.tracker.min_hits").range(1, 100);
    s.optional<double>("perception.tracker.max_association_cost").range(0.0, 10000.0);
    s.optional<double>("perception.tracker.high_conf_threshold").range(0.0, 1.0);
    s.optional<double>("perception.tracker.low_conf_threshold").range(0.0, 1.0);
    s.optional<double>("perception.tracker.max_iou_cost").range(0.0, 1.0);
    // Radar
    s.optional<bool>("perception.radar.enabled");
    s.optional<int>("perception.radar.update_rate_hz").range(1, 1000);
    // Fusion
    s.optional<int>("perception.fusion.rate_hz").range(1, 1000);
    return s;
}

inline ConfigSchema slam_schema() {
    auto s = common_schema();
    s.required_section("slam");
    s.required<int>("slam.vio_rate_hz").range(1, 10000);
    s.optional<int>("slam.visual_frontend_rate_hz").range(1, 1000);
    s.optional<int>("slam.imu_rate_hz").range(1, 10000);
    s.optional<double>("slam.keyframe.min_parallax_px").range(0.0, 1000.0);
    s.optional<double>("slam.keyframe.min_tracked_ratio").range(0.0, 1.0);
    s.optional<double>("slam.keyframe.max_time_sec").range(0.001, 60.0);
    // VIO quality thresholds
    s.optional<double>("slam.vio.quality.good_trace_max").range(0.0, 100.0);
    s.optional<double>("slam.vio.quality.degraded_trace_max").range(0.0, 100.0);
    return s;
}

inline ConfigSchema mission_planner_schema() {
    auto s = common_schema();
    s.required_section("mission_planner");
    s.required<int>("mission_planner.update_rate_hz").range(1, 1000);
    s.required<double>("mission_planner.takeoff_altitude_m").range(0.5, 500.0);
    s.optional<double>("mission_planner.acceptance_radius_m").range(0.1, 100.0);
    s.optional<double>("mission_planner.cruise_speed_mps").range(0.1, 50.0);
    s.optional<double>("mission_planner.rtl_acceptance_radius_m").range(0.1, 100.0);
    s.optional<double>("mission_planner.landed_altitude_m").range(0.0, 50.0);
    s.optional<int>("mission_planner.rtl_min_dwell_seconds").range(0, 600);
    // Path planner
    s.optional<double>("mission_planner.path_planner.ramp_dist_m").range(0.0, 100.0);
    s.optional<double>("mission_planner.path_planner.min_speed_mps").range(0.0, 50.0);
    s.optional<int>("mission_planner.path_planner.snap_search_radius").range(1, 100);
    s.optional<double>("mission_planner.path_planner.yaw_smoothing_rate").range(0.0, 1.0);
    s.optional<double>("mission_planner.path_planner.snap_approach_bias").range(0.0, 1.0);
    // Occupancy grid
    s.optional<double>("mission_planner.occupancy_grid.resolution_m").range(0.01, 10.0);
    s.optional<double>("mission_planner.occupancy_grid.inflation_radius_m").range(0.0, 50.0);
    s.optional<double>("mission_planner.occupancy_grid.dynamic_obstacle_ttl_s").range(0.0, 60.0);
    s.optional<double>("mission_planner.occupancy_grid.min_confidence").range(0.0, 1.0);
    s.optional<double>("mission_planner.occupancy_grid.min_promotion_depth_confidence")
        .range(0.0, 1.0);
    s.optional<int>("mission_planner.occupancy_grid.max_static_cells").range(0, 1000000);
    s.optional<bool>("mission_planner.occupancy_grid.prediction_enabled");
    s.optional<double>("mission_planner.occupancy_grid.prediction_dt_s").range(0.0, 60.0);
    // Obstacle avoidance
    s.optional<double>("mission_planner.obstacle_avoidance.min_distance_m").range(0.1, 100.0);
    s.optional<double>("mission_planner.obstacle_avoidance.influence_radius_m").range(0.1, 100.0);
    s.optional<double>("mission_planner.obstacle_avoidance.repulsive_gain").range(0.0, 100.0);
    s.optional<double>("mission_planner.obstacle_avoidance.max_correction_mps").range(0.0, 50.0);
    s.optional<double>("mission_planner.obstacle_avoidance.min_confidence").range(0.0, 1.0);
    s.optional<double>("mission_planner.obstacle_avoidance.prediction_dt_s").range(0.0, 60.0);
    s.optional<int>("mission_planner.obstacle_avoidance.max_age_ms").range(0, 60000);
    // Geofence
    s.optional<double>("mission_planner.geofence.altitude_floor_m").range(-1000.0, 10000.0);
    s.optional<double>("mission_planner.geofence.altitude_ceiling_m").range(0.0, 10000.0);
    s.optional<double>("mission_planner.geofence.warning_margin_m").range(0.0, 1000.0);
    s.optional<double>("mission_planner.geofence.altitude_tolerance_m").range(0.0, 100.0);
    // Collision recovery
    s.optional<bool>("mission_planner.collision_recovery.enabled");
    s.optional<double>("mission_planner.collision_recovery.climb_delta_m").range(0.0, 100.0);
    s.optional<double>("mission_planner.collision_recovery.hover_duration_s").range(0.0, 60.0);
    // fault_manager section
    s.optional<int>("fault_manager.pose_stale_timeout_ms").range(1, 60000);
    s.optional<double>("fault_manager.battery_warn_percent").range(1.0, 100.0);
    s.optional<double>("fault_manager.battery_crit_percent").range(0.0, 100.0);
    return s;
}

inline ConfigSchema comms_schema() {
    auto s = common_schema();
    s.required_section("comms");
    s.optional<int>("comms.mavlink.heartbeat_rate_hz").range(1, 100);
    s.optional<int>("comms.mavlink.tx_rate_hz").range(1, 1000);
    s.optional<int>("comms.mavlink.rx_rate_hz").range(1, 1000);
    s.optional<int>("comms.mavlink.timeout_ms").range(1, 60000);
    s.optional<int>("comms.mavlink.baud_rate").range(1200, 4000000);
    s.optional<int>("comms.gcs.udp_port").range(1, 65535);
    s.optional<int>("comms.gcs.telemetry_rate_hz").range(1, 100);
    return s;
}

inline ConfigSchema payload_manager_schema() {
    auto s = common_schema();
    s.required_section("payload_manager");
    s.required<int>("payload_manager.update_rate_hz").range(1, 1000);
    s.optional<double>("payload_manager.gimbal.max_slew_rate_dps").range(0.1, 1000.0);
    s.optional<double>("payload_manager.gimbal.pitch_min_deg").range(-180.0, 0.0);
    s.optional<double>("payload_manager.gimbal.pitch_max_deg").range(0.0, 180.0);
    s.optional<double>("payload_manager.gimbal.yaw_min_deg").range(-360.0, 0.0);
    s.optional<double>("payload_manager.gimbal.yaw_max_deg").range(0.0, 360.0);
    s.optional<bool>("payload_manager.gimbal.auto_track.enabled");
    s.optional<double>("payload_manager.gimbal.auto_track.min_confidence").range(0.0, 1.0);
    return s;
}

inline ConfigSchema system_monitor_schema() {
    auto s = common_schema();
    s.required_section("system_monitor");
    s.optional<std::string>("system_monitor.platform").one_of({"linux", "jetson", "mock"});
    s.required<int>("system_monitor.update_rate_hz").range(1, 100);
    s.optional<int>("system_monitor.disk_check_interval_s").range(1, 3600);
    s.optional<double>("system_monitor.thresholds.cpu_warn_percent").range(0.0, 100.0);
    s.optional<double>("system_monitor.thresholds.mem_warn_percent").range(0.0, 100.0);
    s.optional<double>("system_monitor.thresholds.temp_warn_c").range(0.0, 200.0);
    s.optional<double>("system_monitor.thresholds.temp_crit_c").range(0.0, 200.0);
    s.optional<double>("system_monitor.thresholds.battery_warn_percent").range(0.0, 100.0);
    s.optional<double>("system_monitor.thresholds.battery_crit_percent").range(0.0, 100.0);
    s.optional<double>("system_monitor.thresholds.disk_crit_percent").range(0.0, 100.0);
    return s;
}

// ── validate_or_exit() — validate and log errors ───────────
// Returns 0 on success, 1 on validation failure (logs errors via spdlog).
// Catches exceptions from malformed configs (defense-in-depth).
inline int validate_or_exit(const Config& cfg, const ConfigSchema& schema) {
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
