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
#include <unistd.h>

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
    // For 50x50 boxes shifted along x only:
    // IoU = (50 - offset) / (50 + offset)
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

// Helper: build a FrameData with configurable track IDs for tracking tests.
FrameData make_tracked_frame(uint32_t class_id, uint32_t gt_track_id, uint32_t pred_track_id,
                             float confidence, float iou_overlap) {
    FrameData f;
    f.timestamp_ns = gt_track_id * 1000;

    GroundTruthDetection gt;
    gt.class_id    = class_id;
    gt.bbox        = {100.0F, 100.0F, 50.0F, 50.0F};
    gt.gt_track_id = gt_track_id;
    f.ground_truth.push_back(gt);

    float offset = 0.0F;
    if (iou_overlap < 0.99F) {
        offset = 50.0F * (1.0F - iou_overlap) / (1.0F + iou_overlap) * 2.0F;
    }

    PredictedDetection pred;
    pred.class_id      = class_id;
    pred.bbox          = {100.0F + offset, 100.0F, 50.0F, 50.0F};
    pred.confidence    = confidence;
    pred.pred_track_id = pred_track_id;
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

// Helper: FN frame with a specific GT track ID for tracking tests.
FrameData make_fn_frame_with_track(uint32_t class_id, uint32_t gt_track_id) {
    FrameData f;
    f.timestamp_ns = gt_track_id * 1000 + 500;

    GroundTruthDetection gt;
    gt.class_id    = class_id;
    gt.bbox        = {100.0F, 100.0F, 50.0F, 50.0F};
    gt.gt_track_id = gt_track_id;
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

std::string tmp_path(const std::string& base) {
    return "/tmp/" + base + "_" + std::to_string(getpid()) + ".json";
}

}  // namespace

// -- Basic capture: single scenario, single class, perfect detection ----------

TEST(BaselineCaptureTest, SingleScenarioPerfectDetection) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("test_scenario");

    for (int i = 0; i < 10; ++i) {
        s.frames.push_back(make_matched_frame(0, 0.9F, 0.95F));
    }

    ASSERT_TRUE(capture.finalise("test_scenario", 0.5F, 1));

    const auto* result = capture.scenario("test_scenario");
    ASSERT_NE(result, nullptr);

    EXPECT_EQ(result->frame_count, 10U);
    EXPECT_EQ(result->total_tp, 10U);
    EXPECT_EQ(result->total_fp, 0U);
    EXPECT_EQ(result->total_fn, 0U);
    EXPECT_DOUBLE_EQ(result->micro_precision, 1.0);
    EXPECT_DOUBLE_EQ(result->micro_recall, 1.0);
    EXPECT_NEAR(result->mean_ap, 1.0, 1e-10);
}

// -- Mixed TP/FP/FN across frames --------------------------------------------

TEST(BaselineCaptureTest, MixedDetections) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("mixed");

    s.frames.push_back(make_matched_frame(0, 0.9F, 0.8F));  // TP
    s.frames.push_back(make_fn_frame(0));                   // FN
    s.frames.push_back(make_fp_frame(0));                   // FP
    s.frames.push_back(make_matched_frame(0, 0.7F, 0.8F));  // TP

    ASSERT_TRUE(capture.finalise("mixed", 0.5F, 1));

    const auto* r = capture.scenario("mixed");
    ASSERT_NE(r, nullptr);
    EXPECT_EQ(r->total_tp, 2U);
    EXPECT_EQ(r->total_fp, 1U);
    EXPECT_EQ(r->total_fn, 1U);
    EXPECT_EQ(r->frame_count, 4U);
}

// -- Per-class breakdown with class names ------------------------------------

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
    ASSERT_TRUE(capture.finalise("multi_class", 0.5F, 2, names));

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

// -- Multiple scenarios in insertion order -----------------------------------

TEST(BaselineCaptureTest, MultipleScenarios) {
    BaselineCapture capture;
    capture.add_scenario("scenario_b");
    capture.add_scenario("scenario_a");
    capture.add_scenario("scenario_c");

    const auto& names = capture.scenario_names();
    ASSERT_EQ(names.size(), 3U);
    EXPECT_EQ(names[0], "scenario_b");
    EXPECT_EQ(names[1], "scenario_a");
    EXPECT_EQ(names[2], "scenario_c");
}

// -- JSON round-trip: write then load preserves metrics ----------------------

TEST(BaselineCaptureTest, JsonRoundTrip) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("roundtrip");

    for (int i = 0; i < 5; ++i) {
        s.frames.push_back(make_matched_frame(0, 0.9F, 0.85F));
    }
    s.frames.push_back(make_fp_frame(0));
    s.frames.push_back(make_fn_frame(0));

    std::map<uint32_t, std::string> names = {{0, "person"}};
    ASSERT_TRUE(capture.finalise("roundtrip", 0.5F, 1, names));

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
    const std::string path = tmp_path("test_baseline_roundtrip");
    ASSERT_TRUE(capture.write_json(path));

    BaselineCapture loaded;
    ASSERT_TRUE(loaded.load_json(path));

    const auto* orig = capture.scenario("roundtrip");
    const auto* lr   = loaded.scenario("roundtrip");
    ASSERT_NE(lr, nullptr);

    // Detection metrics.
    EXPECT_EQ(lr->total_tp, orig->total_tp);
    EXPECT_EQ(lr->total_fp, orig->total_fp);
    EXPECT_EQ(lr->total_fn, orig->total_fn);
    EXPECT_DOUBLE_EQ(lr->micro_precision, orig->micro_precision);
    EXPECT_DOUBLE_EQ(lr->micro_recall, orig->micro_recall);
    EXPECT_NEAR(lr->mean_ap, orig->mean_ap, 1e-10);

    // Tracking metrics.
    EXPECT_NEAR(lr->mota, orig->mota, 1e-10);
    EXPECT_NEAR(lr->motp, orig->motp, 1e-10);
    EXPECT_EQ(lr->id_switches, orig->id_switches);
    EXPECT_EQ(lr->fragmentations, orig->fragmentations);

    // Per-class round-trip.
    ASSERT_EQ(lr->per_class.size(), 1U);
    EXPECT_EQ(lr->per_class[0].class_name, "person");
    EXPECT_EQ(lr->per_class[0].tp, orig->per_class[0].tp);
    EXPECT_NEAR(lr->per_class[0].ap, orig->per_class[0].ap, 1e-10);

    // Cleanup.
    std::filesystem::remove(path);
}

// -- Latency JSON passthrough ------------------------------------------------

TEST(BaselineCaptureTest, LatencyJsonPassthrough) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("with_latency");
    s.frames.push_back(make_matched_frame(0, 0.9F, 0.8F));
    s.latency_json = R"({"stages":{"detector":{"count":100,"p95_ns":5000000}},"traces":[]})";
    ASSERT_TRUE(capture.finalise("with_latency", 0.5F, 1));

    const std::string json   = capture.to_json();
    auto              parsed = nlohmann::json::parse(json);

    EXPECT_TRUE(parsed["scenarios"]["with_latency"].contains("latency"));
    EXPECT_TRUE(parsed["scenarios"]["with_latency"]["latency"].contains("stages"));

    // Round-trip the latency and verify content fidelity.
    const std::string path = tmp_path("test_baseline_latency");
    ASSERT_TRUE(capture.write_json(path));

    BaselineCapture loaded;
    ASSERT_TRUE(loaded.load_json(path));

    auto loaded_latency   = nlohmann::json::parse(loaded.scenario("with_latency")->latency_json);
    auto original_latency = nlohmann::json::parse(capture.scenario("with_latency")->latency_json);
    EXPECT_EQ(loaded_latency, original_latency);

    std::filesystem::remove(path);
}

// -- Tracking metrics populated -----------------------------------------------

TEST(BaselineCaptureTest, TrackingMetrics) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("tracking");

    // 5-frame perfect track — same gt_track_id and pred_track_id.
    for (int i = 0; i < 5; ++i) {
        s.frames.push_back(make_matched_frame(0, 0.9F, 0.9F));
    }
    ASSERT_TRUE(capture.finalise("tracking", 0.5F, 1));

    const auto* r = capture.scenario("tracking");
    ASSERT_NE(r, nullptr);
    // iou_overlap=0.9 → actual IoU ≈ (3*0.9 - 1)/(3 - 0.9) ≈ 0.81
    EXPECT_NEAR(r->motp, 0.81, 0.1);
    EXPECT_EQ(r->id_switches, 0U);
    EXPECT_EQ(r->fragmentations, 0U);
}

// -- Tracking: ID switch detection -------------------------------------------

TEST(BaselineCaptureTest, TrackingIdSwitch) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("id_switch");

    // Same GT track, different pred track in frame 2 → ID switch.
    s.frames.push_back(make_tracked_frame(0, 1, 10, 0.9F, 0.9F));
    s.frames.push_back(make_tracked_frame(0, 1, 20, 0.9F, 0.9F));
    s.frames.push_back(make_tracked_frame(0, 1, 20, 0.9F, 0.9F));

    ASSERT_TRUE(capture.finalise("id_switch", 0.5F, 1));

    const auto* r = capture.scenario("id_switch");
    ASSERT_NE(r, nullptr);
    EXPECT_GE(r->id_switches, 1U);
}

// -- Tracking: fragmentation detection ----------------------------------------

TEST(BaselineCaptureTest, TrackingFragmentation) {
    BaselineCapture   capture;
    ScenarioBaseline& s = capture.add_scenario("fragmentation");

    // Track appears, disappears (FN), reappears → fragmentation.
    s.frames.push_back(make_tracked_frame(0, 1, 10, 0.9F, 0.9F));
    s.frames.push_back(make_fn_frame_with_track(0, 1));
    s.frames.push_back(make_tracked_frame(0, 1, 10, 0.9F, 0.9F));

    ASSERT_TRUE(capture.finalise("fragmentation", 0.5F, 1));

    const auto* r = capture.scenario("fragmentation");
    ASSERT_NE(r, nullptr);
    EXPECT_GE(r->fragmentations, 1U);
}

// -- Empty scenario ----------------------------------------------------------

TEST(BaselineCaptureTest, EmptyScenario) {
    BaselineCapture capture;
    capture.add_scenario("empty");
    ASSERT_TRUE(capture.finalise("empty", 0.5F, 1));

    const auto* r = capture.scenario("empty");
    ASSERT_NE(r, nullptr);
    EXPECT_EQ(r->frame_count, 0U);
    EXPECT_EQ(r->total_tp, 0U);
    EXPECT_EQ(r->total_fp, 0U);
    EXPECT_EQ(r->total_fn, 0U);
}

// -- Finalise with unknown scenario returns false ----------------------------

TEST(BaselineCaptureTest, FinaliseUnknownScenario) {
    BaselineCapture capture;
    EXPECT_FALSE(capture.finalise("nonexistent", 0.5F, 1));
}

// -- Load from invalid path returns false ------------------------------------

TEST(BaselineCaptureTest, LoadInvalidPath) {
    BaselineCapture capture;
    EXPECT_FALSE(capture.load_json("/nonexistent/path.json"));
}

// -- Load from malformed JSON returns false -----------------------------------

TEST(BaselineCaptureTest, LoadMalformedJson) {
    const std::string path = tmp_path("test_baseline_malformed");
    {
        std::ofstream out(path);
        out << "{this is not valid json}";
    }
    BaselineCapture capture;
    EXPECT_FALSE(capture.load_json(path));
    std::filesystem::remove(path);
}

// -- Load from wrong-schema JSON returns false --------------------------------

TEST(BaselineCaptureTest, LoadWrongSchemaJson) {
    const std::string path = tmp_path("test_baseline_wrong_schema");
    {
        std::ofstream out(path);
        out << R"({"version": 1})";
    }
    BaselineCapture capture;
    EXPECT_FALSE(capture.load_json(path));
    std::filesystem::remove(path);
}

// -- Load preserves existing state on failure --------------------------------

TEST(BaselineCaptureTest, LoadPreservesStateOnFailure) {
    BaselineCapture capture;
    capture.add_scenario("existing");
    ASSERT_TRUE(capture.finalise("existing", 0.5F, 1));
    ASSERT_EQ(capture.size(), 1U);

    EXPECT_FALSE(capture.load_json("/nonexistent/path.json"));
    EXPECT_EQ(capture.size(), 1U);
    EXPECT_NE(capture.scenario("existing"), nullptr);
}

// -- Nonexistent scenario returns nullptr ------------------------------------

TEST(BaselineCaptureTest, NonexistentScenario) {
    BaselineCapture capture;
    EXPECT_EQ(capture.scenario("nope"), nullptr);
}

// -- Duplicate scenario name reuses existing ---------------------------------

TEST(BaselineCaptureTest, DuplicateScenarioReuses) {
    BaselineCapture   capture;
    ScenarioBaseline& s1 = capture.add_scenario("dup");
    s1.frames.push_back(make_matched_frame(0, 0.9F, 0.8F));

    ScenarioBaseline& s2 = capture.add_scenario("dup");
    EXPECT_EQ(s2.frames.size(), 1U);  // same object, not overwritten
    EXPECT_EQ(capture.size(), 1U);
}
