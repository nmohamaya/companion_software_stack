// process4_mission_planner/include/planner/geofence.h
// Geofence — polygon + altitude ceiling/floor boundary checker.
//
// Evaluates whether a 3D position is within an allowed operating volume.
// The lateral boundary is a convex or concave polygon (ray-casting test),
// and the vertical boundary is a [floor, ceiling] altitude band.
//
// Usage:
//   Geofence fence;
//   fence.set_polygon({{0,0},{100,0},{100,100},{0,100}});
//   fence.set_altitude_limits(0.0f, 120.0f);
//   fence.enable();
//
//   auto result = fence.check(x, y, altitude);
//   if (result.violated) { /* take action */ }
//
// Thread safety: NOT thread-safe.  Call from the planning loop only.
// A future GCS upload path should swap the polygon atomically.
//
// Implements Epic #25 issue #29 (Geofencing).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::planner {

// ─────────────────────────────────────────────────────────────
// Types
// ─────────────────────────────────────────────────────────────

/// 2D vertex for the geofence polygon (NED or ENU local frame, metres).
struct GeoVertex {
    float x = 0.0f;
    float y = 0.0f;
};

/// Reason the geofence was violated.
enum class GeofenceViolation : uint8_t {
    NONE            = 0,
    OUTSIDE_POLYGON = 1,
    ABOVE_CEILING   = 2,
    BELOW_FLOOR     = 3,
};

inline const char* violation_name(GeofenceViolation v) {
    switch (v) {
        case GeofenceViolation::NONE: return "NONE";
        case GeofenceViolation::OUTSIDE_POLYGON: return "OUTSIDE_POLYGON";
        case GeofenceViolation::ABOVE_CEILING: return "ABOVE_CEILING";
        case GeofenceViolation::BELOW_FLOOR: return "BELOW_FLOOR";
        default: return "UNKNOWN";
    }
}

/// Result of a geofence check.
struct GeofenceResult {
    bool              violated = false;
    GeofenceViolation reason   = GeofenceViolation::NONE;
    float             margin_m = 0.0f;  // distance to nearest boundary (negative = inside)
    std::string       message;
};

// ─────────────────────────────────────────────────────────────
// Geofence
// ─────────────────────────────────────────────────────────────

class Geofence {
public:
    Geofence() = default;

    // ── Configuration ───────────────────────────────────────

    /// Set the lateral boundary polygon (minimum 3 vertices).
    /// Vertices should be ordered (CW or CCW); the polygon is closed
    /// automatically (last→first edge is implicit).
    void set_polygon(const std::vector<GeoVertex>& vertices) {
        polygon_ = vertices;
        if (polygon_.size() < 3) {
            spdlog::warn("[Geofence] Polygon has {} vertices (need ≥3) — disabled",
                         polygon_.size());
            enabled_ = false;
        }
    }

    /// Set the vertical operating band.
    /// @param floor_m   Minimum allowed altitude AGL (metres).  Typically 0.
    /// @param ceiling_m Maximum allowed altitude AGL (metres).
    void set_altitude_limits(float floor_m, float ceiling_m) {
        alt_floor_   = floor_m;
        alt_ceiling_ = ceiling_m;
    }

    /// Set the warning margin (metres).
    /// `check()` will populate `margin_m` — the caller can use it
    /// to issue a warning *before* the hard boundary is hit.
    void set_warning_margin(float margin_m) { warning_margin_ = std::max(0.0f, margin_m); }

    /// Enable/disable the geofence.
    void enable(bool on = true) {
        if (on && polygon_.size() < 3) {
            spdlog::error("[Geofence] Cannot enable — polygon has {} vertices", polygon_.size());
            return;
        }
        enabled_ = on;
        spdlog::info("[Geofence] {} ({} vertices, alt {:.0f}–{:.0f}m, margin {:.0f}m)",
                     enabled_ ? "ENABLED" : "DISABLED", polygon_.size(), alt_floor_, alt_ceiling_,
                     warning_margin_);
    }

    [[nodiscard]] bool is_enabled() const { return enabled_; }

    // ── Evaluation ──────────────────────────────────────────

    /// Check whether a 3D position is inside the geofence.
    /// @param x   East (or X) position in local frame (metres).
    /// @param y   North (or Y) position in local frame (metres).
    /// @param alt Altitude AGL (metres, positive up).
    /// @return    GeofenceResult with violation info.
    [[nodiscard]] GeofenceResult check(float x, float y, float alt) const {
        GeofenceResult result;
        if (!enabled_) return result;  // disabled → always OK

        // Guard against NaN/Inf inputs — treat as violation (defensive)
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(alt)) {
            result.violated = true;
            result.reason   = GeofenceViolation::OUTSIDE_POLYGON;
            result.margin_m = 0.0f;
            result.message  = "Non-finite position input — treating as violation";
            return result;
        }

        // ── Altitude check ──────────────────────────────────
        // Tolerance prevents false positives from sensor noise at
        // ground level (e.g. Gazebo reporting -0.3m before takeoff).
        constexpr float kAltTolerance = 0.5f;  // metres
        if (alt > alt_ceiling_ + kAltTolerance) {
            result.violated = true;
            result.reason   = GeofenceViolation::ABOVE_CEILING;
            result.margin_m = alt - alt_ceiling_;
            result.message  = "Above ceiling: " + std::to_string(static_cast<int>(alt)) + "m > " +
                             std::to_string(static_cast<int>(alt_ceiling_)) + "m";
            return result;
        }
        if (alt < alt_floor_ - kAltTolerance) {
            result.violated = true;
            result.reason   = GeofenceViolation::BELOW_FLOOR;
            result.margin_m = alt_floor_ - alt;
            result.message  = "Below floor: " + std::to_string(static_cast<int>(alt)) + "m < " +
                             std::to_string(static_cast<int>(alt_floor_)) + "m";
            return result;
        }

        // ── Lateral (polygon) check — ray-casting algorithm ─
        if (!point_in_polygon(x, y)) {
            result.violated = true;
            result.reason   = GeofenceViolation::OUTSIDE_POLYGON;
            result.margin_m = distance_to_polygon(x, y);
            result.message  = "Outside polygon boundary (" +
                             std::to_string(static_cast<int>(result.margin_m)) + "m from edge)";
            return result;
        }

        // Inside — compute margin to nearest edge (for warning).
        result.margin_m = -distance_to_polygon(x, y);  // negative = safely inside
        return result;
    }

    // ── Accessors ───────────────────────────────────────────

    [[nodiscard]] const std::vector<GeoVertex>& polygon() const { return polygon_; }
    [[nodiscard]] float                         alt_ceiling() const { return alt_ceiling_; }
    [[nodiscard]] float                         alt_floor() const { return alt_floor_; }
    [[nodiscard]] float                         warning_margin() const { return warning_margin_; }

private:
    std::vector<GeoVertex> polygon_;
    float                  alt_floor_      = 0.0f;
    float                  alt_ceiling_    = 120.0f;  // AGL metres
    float                  warning_margin_ = 10.0f;   // metres
    bool                   enabled_        = false;

    // ── Ray-casting point-in-polygon ────────────────────────
    // Classic even-odd rule.  Casts a ray along +X and counts
    // edge crossings.  Works for convex and concave polygons.
    [[nodiscard]] bool point_in_polygon(float px, float py) const {
        bool   inside = false;
        size_t n      = polygon_.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            float xi = polygon_[i].x, yi = polygon_[i].y;
            float xj = polygon_[j].x, yj = polygon_[j].y;

            bool intersect = ((yi > py) != (yj > py)) &&
                             (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    // ── Minimum distance to polygon boundary ────────────────
    // Used for margin computation (warning zone).
    [[nodiscard]] float distance_to_polygon(float px, float py) const {
        float  min_dist = std::numeric_limits<float>::max();
        size_t n        = polygon_.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            float d = point_to_segment_distance(px, py, polygon_[j].x, polygon_[j].y, polygon_[i].x,
                                                polygon_[i].y);
            min_dist = std::min(min_dist, d);
        }
        return min_dist;
    }

    /// Distance from point (px,py) to line segment (ax,ay)-(bx,by).
    static float point_to_segment_distance(float px, float py, float ax, float ay, float bx,
                                           float by) {
        float dx = bx - ax, dy = by - ay;
        float len2 = dx * dx + dy * dy;
        if (len2 < 1e-10f) {
            // Degenerate segment (point)
            float ex = px - ax, ey = py - ay;
            return std::sqrt(ex * ex + ey * ey);
        }
        // Parameter t of closest point on segment
        float t  = std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0f, 1.0f);
        float cx = ax + t * dx - px;
        float cy = ay + t * dy - py;
        return std::sqrt(cx * cx + cy * cy);
    }
};

}  // namespace drone::planner
