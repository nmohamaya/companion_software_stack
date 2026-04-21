// tests/benchmark/baseline_capture.h
//
// Baseline capture for the perception benchmark harness (Issue #573,
// Epic #523). Accumulates per-scenario detection + tracking metrics and
// latency summaries, then serialises to benchmarks/baseline.json.
//
// Usage (inside a scenario runner):
//
//   BaselineCapture capture;
//   ScenarioBaseline& s = capture.add_scenario("cosys_perception");
//
//   // Per-frame: feed GT + predictions via FrameData
//   s.frames.push_back(frame_data);
//
//   // Per-frame: record latency via the profiler
//   // (LatencyProfiler is separate — snapshot it at the end)
//
//   // End of scenario: finalise metrics
//   s.latency_json = profiler.to_json();
//   capture.finalise("cosys_perception", iou_threshold, num_classes);
//
//   // After all scenarios: serialise
//   capture.write_json("benchmarks/baseline.json");

#pragma once

#include "benchmark/perception_metrics.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace drone::benchmark {

struct PerClassBaseline {
    uint32_t    class_id{0};
    std::string class_name{};
    uint32_t    tp{0};
    uint32_t    fp{0};
    uint32_t    fn{0};
    double      precision{0.0};
    double      recall{0.0};
    double      f1{0.0};
    double      ap{0.0};
    uint32_t    detection_count{0};
};

struct ScenarioBaseline {
    std::string scenario_name{};

    // Raw frame data accumulated during the run — consumed by finalise().
    std::vector<FrameData> frames{};

    // Detection metrics (filled by finalise).
    uint32_t total_tp{0};
    uint32_t total_fp{0};
    uint32_t total_fn{0};
    double   micro_precision{0.0};
    double   micro_recall{0.0};
    double   mean_ap{0.0};
    float    iou_threshold{0.5F};
    uint32_t num_classes{0};

    std::vector<PerClassBaseline> per_class{};

    // Tracking metrics (filled by finalise).
    double   mota{0.0};
    double   motp{0.0};
    uint32_t id_switches{0};
    uint32_t fragmentations{0};

    // Latency JSON snapshot from LatencyProfiler::to_json().
    // Stored as an opaque string — parsed back into the baseline JSON as-is.
    std::string latency_json{};

    uint32_t frame_count{0};
};

// Accumulates baselines for multiple scenarios and serialises to JSON.
class BaselineCapture {
public:
    BaselineCapture()                                  = default;
    ~BaselineCapture()                                 = default;
    BaselineCapture(const BaselineCapture&)            = delete;
    BaselineCapture& operator=(const BaselineCapture&) = delete;
    BaselineCapture(BaselineCapture&&)                 = default;
    BaselineCapture& operator=(BaselineCapture&&)      = default;

    // Add a new scenario or return the existing one. Returns a mutable reference
    // for frame accumulation. Duplicate names return the existing entry (idempotent).
    ScenarioBaseline& add_scenario(const std::string& name);

    // Compute detection + tracking metrics from accumulated frames.
    // Must be called after all frames are added for a scenario.
    // Returns false if the scenario name was not previously added.
    [[nodiscard]] bool finalise(const std::string& scenario_name, float iou_threshold,
                                uint32_t num_classes);

    // Finalise with a class-name map so per-class entries get human-readable names.
    // class_names: class_id -> class_name.
    // Returns false if the scenario name was not previously added.
    [[nodiscard]] bool finalise(const std::string& scenario_name, float iou_threshold,
                                uint32_t                               num_classes,
                                const std::map<uint32_t, std::string>& class_names);

    // Access a scenario baseline (returns nullptr if not found).
    [[nodiscard]] const ScenarioBaseline* scenario(const std::string& name) const;

    // All scenario names in insertion order.
    [[nodiscard]] const std::vector<std::string>& scenario_names() const;

    // Serialise the full baseline to a JSON string (pretty-printed, stable order).
    [[nodiscard]] std::string to_json() const;

    // Write to a file. Returns true on success. On failure, the output file may
    // be partially written or truncated.
    [[nodiscard]] bool write_json(const std::string& path) const;

    // Load an existing baseline from JSON. Returns true on success.
    // Loaded baselines have empty `frames` vectors (raw data is not stored in
    // the JSON — only the computed metrics). On failure, existing state is
    // preserved (parse uses temporaries before committing).
    [[nodiscard]] bool load_json(const std::string& path);

    [[nodiscard]] std::size_t size() const { return order_.size(); }

private:
    // Insertion-ordered scenario map.
    std::vector<std::string>                order_{};
    std::map<std::string, ScenarioBaseline> scenarios_{};
};

}  // namespace drone::benchmark
