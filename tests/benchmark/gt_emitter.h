// tests/benchmark/gt_emitter.h
//
// Ground-truth emitter for the perception benchmark harness (Issue #594,
// Epic #523). Consumed by baseline capture (#573) to produce TP/FP/FN and
// MOTA/MOTP metrics against the pre-rewrite pipeline.
//
// Design summary (see docs/design/perception_v2_detailed_design.md § 13):
//   - Per-frame dynamic emission from simulator APIs (not static config).
//   - Stable `gt_track_id` across FoV exits — the simulator has omniscient
//     object identity, so re-entering objects emit the same track_id.
//   - Out-of-FoV / heavily occluded objects are NOT emitted that frame.
//   - Pluggable backend: Cosys (PR 1, this header) or Gazebo (PR 2, follow-up).
//
// Scope: pure C++17, header-only type declarations + small free functions.
// Backend implementations live in separate .cpp files gated behind
// `HAVE_COSYS_AIRSIM` / `HAVE_GAZEBO`.

#pragma once

#include "benchmark/perception_metrics.h"  // BBox2D

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace drone {
class Config;
}

namespace drone::benchmark {

// ────────────────────────────────────────────────────────────────────────────
// Data types — plug into perception_metrics::FrameData on the consumer side
// ────────────────────────────────────────────────────────────────────────────

/// 6-DoF camera pose captured alongside the frame. Stored in the GT record so
/// downstream consumers can correlate without a separate pose log.
struct GtCameraPose {
    float translation[3]{0.0F, 0.0F, 0.0F};       // world-frame, metres
    float quaternion[4]{1.0F, 0.0F, 0.0F, 0.0F};  // w, x, y, z
};

/// One ground-truth detection — what the simulator says was visible in the
/// frame, before the detector/tracker saw it.
struct GtDetection {
    uint32_t    class_id{0};       // COCO-style index; matches the detector's output
    std::string class_name{};      // human-readable, for the JSONL output
    BBox2D      bbox{};            // image-space (reuses perception_metrics::BBox2D)
    uint32_t    gt_track_id{0};    // stable simulator identity; same object → same id
    float       occlusion{0.0F};   // 0 = fully visible, 1 = fully occluded
    float       distance_m{0.0F};  // Euclidean distance from camera to object centre
};

/// One frame's ground truth. Writeable as one JSON object per line (JSONL) so
/// a long run appends cheaply.
struct FrameGroundTruth {
    uint64_t                 timestamp_ns{0};
    uint64_t                 frame_sequence{0};
    GtCameraPose             camera_pose{};
    std::vector<GtDetection> objects{};
};

// ────────────────────────────────────────────────────────────────────────────
// Class map — translates simulator object names → (class_id, class_name)
// ────────────────────────────────────────────────────────────────────────────

/// Loads a `gt_class_map` table from a scenario config and matches simulator
/// object names against its patterns. Supports trailing-`*` glob only — the
/// goal is human-auditable config, not a regex mini-language.
///
/// Example config entry (see docs/guides/CONFIG_GUIDE.md → "Scenario Config"):
///
///     "gt_class_map": {
///         "SK_Mannequin*":  { "class_id": 0, "class_name": "person" },
///         "SM_Car*":        { "class_id": 2, "class_name": "car" }
///     }
///
/// `lookup("SK_Mannequin_01")` returns `{ class_id: 0, class_name: "person" }`.
/// Objects whose name matches no pattern return `std::nullopt` — they're
/// dropped from GT (treated as "not a class the detector is expected to find").
class GtClassMap {
public:
    struct Entry {
        uint32_t    class_id{0};
        std::string class_name{};
    };

    /// Load from a scenario config's `gt_class_map` section. If the key is
    /// absent, the map is empty (every lookup returns nullopt — effectively
    /// disabling GT emission for that scenario).
    [[nodiscard]] static GtClassMap load(const drone::Config& scenario_cfg);

    /// Direct construction — used by tests and by factory helpers.
    struct PatternEntry {
        std::string pattern;  // trailing-`*` glob, or exact match if no `*`
        Entry       entry;
    };
    explicit GtClassMap(std::vector<PatternEntry> patterns);

    [[nodiscard]] bool        empty() const noexcept { return patterns_.empty(); }
    [[nodiscard]] std::size_t size() const noexcept { return patterns_.size(); }

    /// Match `object_name` against registered patterns. First match wins.
    /// Returns nullopt if no pattern matches.
    [[nodiscard]] std::optional<Entry> lookup(std::string_view object_name) const;

    /// Return just the pattern strings — used by backend emitters that pass
    /// the patterns to the simulator's detection-filter API (e.g. Cosys
    /// `simAddDetectionFilterMeshName`).
    [[nodiscard]] std::vector<std::string> patterns() const;

private:
    std::vector<PatternEntry> patterns_;
};

// ────────────────────────────────────────────────────────────────────────────
// Emitter interface
// ────────────────────────────────────────────────────────────────────────────

class IGroundTruthEmitter {
public:
    virtual ~IGroundTruthEmitter() = default;

    /// Produce one GT record for the frame at `timestamp_ns` / `frame_sequence`.
    /// Returns nullopt if the simulator is unreachable or returned no data —
    /// callers should treat this as "skip this frame" rather than an error.
    [[nodiscard]] virtual std::optional<FrameGroundTruth> emit(uint64_t timestamp_ns,
                                                               uint64_t frame_sequence) = 0;

    /// Human-readable backend name — goes into log lines ("cosys", "gazebo").
    [[nodiscard]] virtual std::string_view backend_name() const noexcept = 0;
};

// ────────────────────────────────────────────────────────────────────────────
// JSONL serialisation — one frame per line, appendable
// ────────────────────────────────────────────────────────────────────────────

/// Serialise a frame to a single JSON line (no trailing newline). Callers
/// that stream to a file should append a `\n` themselves.
[[nodiscard]] std::string to_json_line(const FrameGroundTruth& frame);

// ────────────────────────────────────────────────────────────────────────────
// Backend factories — gated at compile time
// ────────────────────────────────────────────────────────────────────────────

#ifdef HAVE_COSYS_AIRSIM
/// Build a Cosys-AirSim GT emitter. Reads host/port/camera/vehicle from the
/// `cosys_airsim` section of `full_cfg` and `gt_class_map` from the scenario
/// config (which is typically merged into `full_cfg` by the scenario loader).
/// Returns nullptr if the config or the class map is empty (no GT to emit).
[[nodiscard]] std::unique_ptr<IGroundTruthEmitter> create_cosys_gt_emitter(
    const drone::Config& full_cfg);
#endif

}  // namespace drone::benchmark
