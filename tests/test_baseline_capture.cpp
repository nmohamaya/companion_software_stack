// tests/test_baseline_capture.cpp
//
// Unit tests for BaselineCapture (Issue #573, Epic #523).
// Verifies metric accumulation, JSON round-trip, and per-class breakdown.

#include "benchmark/baseline_capture.h"
#include "benchmark/perception_metrics.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using namespace drone::benchmark;

namespace {

// Helper: build a FrameData with one GT and one matching prediction.
FrameData make_matched_frame(uint32_t class_id, float confidence, float iou_overlap) {
    FrameData f;
    f.timestamp_ns = 1000;

    GroundTruthDetection gt;
    gt.class_id    = class_id;
    gt.bbox        = {100.0F, 100.0F, 50.0F, 50.0F};
    gt.gt_track_id = 1;
    f.ground_truth.push_back(gt);

    // Shift prediction box to achieve approximate target IoU.
    // For 50x50 boxes, shifting by `offset` pixels along x reduces IoU.
    // IoU = (50 - offset)^2 / (2*50^2 - (50-offset)^2)
    // For iou_overlap ~0.7, offset ~11 pixels.
    float offset = 0.0F;
    if (iou_overlap < 0.99F) {
        offset = 50.0F * (1.0F - iou_overlap) / (1.0F + iou_overlap) * 2.0F;
    }

    PredictedDetection pred;
    pred.class_id      = class_id;
    pred.bbox          = {100.0F + offset, 100.0F, 50.0F, 50.0F};
    pred.confidence    = confidence;
    pred.pred_track_id = 1;
    f.predictions.push_back(pred);

    return f;
}

// Helper: build a FrameData with a GT but no matching prediction (FN).
FrameData make_fn_frame(uint32_t class_id) {
    FrameData f;
    f.timestamp_ns = 2000;

    GroundTruthDetection gt;
    gt.class_id    = class_id;
    gt.bbox        = {200.0F, 200.0F, 40.0F, 40.0F};
    gt.gt_track_id = 2;
    f.ground_truth.push_back(gt);

    return f;
}

// Helper: build a FrameData with a prediction but no GT (FP).
FrameData make_fp_frame(uint32_t class_id) {
    FrameData f;
    f.timestamp_ns = 3000;

    PredictedDetection pred;
    pred.class_id      = class_id;
    pred.bbox          = {300.0F, 300.0F, 30.0F, 30.0F};
    pred.confidence    = 0.6F;
    pred.pred_track_id = 3;
    f.predictions.push_back(pred);

    return f;
}

}  // namespace

// ── Basic capture: single scenario, single class, perfect detection ─────

TEST(BaselineCaptureTest, SingleScenarioPerfectDetection) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("test_scenario");

    for (int i = 0; i < 10; ++i) {
        s.frames.push_back(make_matched_frame(0, 0.9F, 0.95F));
    }

    capture.finalise("test_scenario", 0.5F, 1);

    const auto* result = capture.scenario("test_scenario");
    ASSERT_NE(result, nullptr);

    EXPECT_EQ(result->frame_count, 10U);
    EXPECT_EQ(result->total_tp, 10U);
    EXPECT_EQ(result->total_fp, 0U);
    EXPECT_EQ(result->total_fn, 0U);
    EXPECT_DOUBLE_EQ(result->micro_precision, 1.0);
    EXPECT_DOUBLE_EQ(result->micro_recall, 1.0);
    EXPECT_GT(result->mean_ap, 0.9);
}

// ── Mixed TP/FP/FN across frames ───────────────────────────────────────

TEST(BaselineCaptureTest, MixedDetections) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("mixed");

    s.frames.push_back(make_matched_frame(0, 0.9F, 0.8F));  // TP
    s.frames.push_back(make_fn_frame(0));                   // FN
    s.frames.push_back(make_fp_frame(0));                   // FP
    s.frames.push_back(make_matched_frame(0, 0.7F, 0.8F));  // TP

    capture.finalise("mixed", 0.5F, 1);

    const auto* r = capture.scenario("mixed");
    ASSERT_NE(r, nullptr);
    EXPECT_EQ(r->total_tp, 2U);
    EXPECT_EQ(r->total_fp, 1U);
    EXPECT_EQ(r->total_fn, 1U);
    EXPECT_EQ(r->frame_count, 4U);
}

// ── Per-class breakdown with class names ────────────────────────────────

TEST(BaselineCaptureTest, PerClassBreakdown) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("multi_class");

    // Class 0: 3 TP
    for (int i = 0; i < 3; ++i) {
        s.frames.push_back(make_matched_frame(0, 0.9F, 0.8F));
    }
    // Class 1: 1 FN
    s.frames.push_back(make_fn_frame(1));

    std::map<uint32_t, std::string> names = {{0, "person"}, {1, "car"}};
    capture.finalise("multi_class", 0.5F, 2, names);

    const auto* r = capture.scenario("multi_class");
    ASSERT_NE(r, nullptr);
    ASSERT_EQ(r->per_class.size(), 2U);

    // Find class 0.
    const PerClassBaseline* cls0 = nullptr;
    const PerClassBaseline* cls1 = nullptr;
    for (const auto& pc : r->per_class) {
        if (pc.class_id == 0) cls0 = &pc;
        if (pc.class_id == 1) cls1 = &pc;
    }

    ASSERT_NE(cls0, nullptr);
    EXPECT_EQ(cls0->tp, 3U);
    EXPECT_EQ(cls0->class_name, "person");

    ASSERT_NE(cls1, nullptr);
    EXPECT_EQ(cls1->fn, 1U);
    EXPECT_EQ(cls1->class_name, "car");
}

// ── Multiple scenarios in insertion order ────────────────────────────────

TEST(BaselineCaptureTest, MultipleScenarios) {
    BaselineCapture capture;
    capture.add_scenario("scenario_b");
    capture.add_scenario("scenario_a");
    capture.add_scenario("scenario_c");

    const auto names = capture.scenario_names();
    ASSERT_EQ(names.size(), 3U);
    EXPECT_EQ(names[0], "scenario_b");
    EXPECT_EQ(names[1], "scenario_a");
    EXPECT_EQ(names[2], "scenario_c");
}

// ── JSON round-trip: write then load preserves metrics ──────────────────

TEST(BaselineCaptureTest, JsonRoundTrip) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("roundtrip");

    for (int i = 0; i < 5; ++i) {
        s.frames.push_back(make_matched_frame(0, 0.9F, 0.85F));
    }
    s.frames.push_back(make_fp_frame(0));
    s.frames.push_back(make_fn_frame(0));

    std::map<uint32_t, std::string> names = {{0, "person"}};
    capture.finalise("roundtrip", 0.5F, 1, names);

    const std::string json = capture.to_json();

    // Parse and verify structure.
    auto parsed = nlohmann::json::parse(json);
    EXPECT_TRUE(parsed.contains("version"));
    EXPECT_TRUE(parsed.contains("scenarios"));
    EXPECT_TRUE(parsed["scenarios"].contains("roundtrip"));

    const auto& sj = parsed["scenarios"]["roundtrip"];
    EXPECT_EQ(sj["frame_count"].get<uint32_t>(), 7U);
    EXPECT_EQ(sj["detection"]["total_tp"].get<uint32_t>(), 5U);
    EXPECT_EQ(sj["detection"]["total_fp"].get<uint32_t>(), 1U);
    EXPECT_EQ(sj["detection"]["total_fn"].get<uint32_t>(), 1U);
    EXPECT_TRUE(sj.contains("per_class"));
    EXPECT_TRUE(sj.contains("tracking"));

    // Write to temp file and load back.
    const std::string tmp_path = "/tmp/test_baseline_roundtrip.json";
    ASSERT_TRUE(capture.write_json(tmp_path));

    BaselineCapture loaded;
    ASSERT_TRUE(loaded.load_json(tmp_path));

    const auto* lr = loaded.scenario("roundtrip");
    ASSERT_NE(lr, nullptr);
    EXPECT_EQ(lr->total_tp, capture.scenario("roundtrip")->total_tp);
    EXPECT_EQ(lr->total_fp, capture.scenario("roundtrip")->total_fp);
    EXPECT_EQ(lr->total_fn, capture.scenario("roundtrip")->total_fn);
    EXPECT_DOUBLE_EQ(lr->micro_precision, capture.scenario("roundtrip")->micro_precision);

    // Cleanup.
    std::filesystem::remove(tmp_path);
}

// ── Latency JSON passthrough ────────────────────────────────────────────

TEST(BaselineCaptureTest, LatencyJsonPassthrough) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("with_latency");
    s.frames.push_back(make_matched_frame(0, 0.9F, 0.8F));
    s.latency_json = R"({"stages":{"detector":{"count":100,"p95_ns":5000000}},"traces":[]})";
    capture.finalise("with_latency", 0.5F, 1);

    const std::string json   = capture.to_json();
    auto              parsed = nlohmann::json::parse(json);

    EXPECT_TRUE(parsed["scenarios"]["with_latency"].contains("latency"));
    EXPECT_TRUE(parsed["scenarios"]["with_latency"]["latency"].contains("stages"));

    // Round-trip the latency.
    const std::string tmp_path = "/tmp/test_baseline_latency.json";
    ASSERT_TRUE(capture.write_json(tmp_path));

    BaselineCapture loaded;
    ASSERT_TRUE(loaded.load_json(tmp_path));
    EXPECT_FALSE(loaded.scenario("with_latency")->latency_json.empty());

    std::filesystem::remove(tmp_path);
}

// ── Tracking metrics populated ──────────────────────────────────────────

TEST(BaselineCaptureTest, TrackingMetrics) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("tracking");

    // 5-frame perfect track — same gt_track_id and pred_track_id.
    for (int i = 0; i < 5; ++i) {
        s.frames.push_back(make_matched_frame(0, 0.9F, 0.9F));
    }
    capture.finalise("tracking", 0.5F, 1);

    const auto* r = capture.scenario("tracking");
    ASSERT_NE(r, nullptr);
    EXPECT_GT(r->motp, 0.0);
    EXPECT_EQ(r->id_switches, 0U);
    EXPECT_EQ(r->fragmentations, 0U);
}

// ── Empty scenario ──────────────────────────────────────────────────────

TEST(BaselineCaptureTest, EmptyScenario) {
    BaselineCapture capture;
    capture.add_scenario("empty");
    capture.finalise("empty", 0.5F, 1);

    const auto* r = capture.scenario("empty");
    ASSERT_NE(r, nullptr);
    EXPECT_EQ(r->frame_count, 0U);
    EXPECT_EQ(r->total_tp, 0U);
    EXPECT_EQ(r->total_fp, 0U);
    EXPECT_EQ(r->total_fn, 0U);
}

// ── Load from invalid JSON returns false ────────────────────────────────

TEST(BaselineCaptureTest, LoadInvalidJson) {
    BaselineCapture capture;
    EXPECT_FALSE(capture.load_json("/nonexistent/path.json"));
}

// ── Nonexistent scenario returns nullptr ────────────────────────────────

TEST(BaselineCaptureTest, NonexistentScenario) {
    BaselineCapture capture;
    EXPECT_EQ(capture.scenario("nope"), nullptr);
}

// ── Duplicate scenario name overwrites ──────────────────────────────────

TEST(BaselineCaptureTest, DuplicateScenarioReuses) {
    BaselineCapture   capture;
    ScenarioBaseline& s1 = capture.add_scenario("dup");
    s1.frames.push_back(make_matched_frame(0, 0.9F, 0.8F));

    ScenarioBaseline& s2 = capture.add_scenario("dup");
    EXPECT_EQ(s2.frames.size(), 1U);  // same object, not overwritten
    EXPECT_EQ(capture.size(), 1U);
}
