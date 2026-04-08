// tests/test_bytetrack_tracker.cpp
// Unit tests for ByteTrackTracker: IoU, cost matrix, two-stage association,
// track lifecycle, occlusion recovery, config/factory.
// Issue #163.
#include "perception/bytetrack_tracker.h"
#include "perception/itracker.h"
#include "perception/kalman_tracker.h"
#include "util/config.h"

#include <cmath>

#include <gtest/gtest.h>

using namespace drone::perception;

// ═══════════════════════════════════════════════════════════
// Helper: make a detection at (x,y) with given w,h,confidence
// ═══════════════════════════════════════════════════════════
static Detection2D make_det(float x, float y, float w, float h, float conf,
                            ObjectClass cls = ObjectClass::PERSON) {
    return {x, y, w, h, conf, cls, 0, 0};
}

static Detection2DList make_det_list(std::initializer_list<Detection2D> dets) {
    Detection2DList dl;
    dl.detections = dets;
    return dl;
}

// ═══════════════════════════════════════════════════════════
// IoU computation (3 tests)
// ═══════════════════════════════════════════════════════════

TEST(ByteTrackIoU, PerfectOverlap) {
    auto a = make_det(0, 0, 100, 100, 0.9f);
    auto b = make_det(0, 0, 100, 100, 0.9f);
    EXPECT_DOUBLE_EQ(ByteTrackTracker::compute_iou(a, b), 1.0);
}

TEST(ByteTrackIoU, NoOverlap) {
    auto a = make_det(0, 0, 50, 50, 0.9f);
    auto b = make_det(200, 200, 50, 50, 0.9f);
    EXPECT_DOUBLE_EQ(ByteTrackTracker::compute_iou(a, b), 0.0);
}

TEST(ByteTrackIoU, PartialOverlap) {
    // Two 100×100 boxes offset by 50px in both x and y.
    // Intersection: 50×50 = 2500.  Union: 10000 + 10000 - 2500 = 17500.
    // IoU = 2500/17500 = 1/7 ≈ 0.142857
    auto a = make_det(0, 0, 100, 100, 0.9f);
    auto b = make_det(50, 50, 100, 100, 0.9f);
    EXPECT_NEAR(ByteTrackTracker::compute_iou(a, b), 1.0 / 7.0, 1e-4);
}

// ═══════════════════════════════════════════════════════════
// IoU cost matrix (2 tests)
// ═══════════════════════════════════════════════════════════

TEST(ByteTrackCostMatrix, SingleTrackSingleDet) {
    ByteTrackTracker tracker;
    // Feed one high-conf detection to create a track
    auto dl = make_det_list({make_det(100, 100, 50, 50, 0.9f)});
    (void)tracker.update(dl);

    // Now compute cost matrix: 1 track vs 1 nearby detection
    std::vector<size_t>      indices{0};
    std::vector<Detection2D> dets{make_det(100, 100, 50, 50, 0.9f)};
    auto                     cost = tracker.compute_iou_cost_matrix(indices, dets);

    ASSERT_EQ(cost.size(), 1u);
    ASSERT_EQ(cost[0].size(), 1u);
    // Should be close to 0 (high IoU → low cost)
    EXPECT_LT(cost[0][0], 0.5);
}

TEST(ByteTrackCostMatrix, MultipleTracksAndDets) {
    ByteTrackTracker tracker;
    // Create 2 tracks with distinct positions
    auto dl = make_det_list({make_det(0, 0, 50, 50, 0.9f), make_det(200, 200, 50, 50, 0.9f)});
    (void)tracker.update(dl);

    // 2 tracks vs 3 detections
    std::vector<size_t>      indices{0, 1};
    std::vector<Detection2D> dets{make_det(0, 0, 50, 50, 0.9f), make_det(200, 200, 50, 50, 0.9f),
                                  make_det(500, 500, 50, 50, 0.9f)};
    auto                     cost = tracker.compute_iou_cost_matrix(indices, dets);

    ASSERT_EQ(cost.size(), 2u);
    ASSERT_EQ(cost[0].size(), 3u);
    // Track 0 should match det 0 well (low cost), det 2 poorly (high cost)
    EXPECT_LT(cost[0][0], 0.5);
    EXPECT_GT(cost[0][2], 0.9);
    // Track 1 should match det 1 well
    EXPECT_LT(cost[1][1], 0.5);
}

// ═══════════════════════════════════════════════════════════
// Two-stage association (4 tests)
// ═══════════════════════════════════════════════════════════

TEST(ByteTrackAssociation, HighConfMatchedFirst) {
    ByteTrackTracker::Params params;
    params.high_conf_threshold = 0.5f;
    params.min_hits            = 1;
    ByteTrackTracker tracker(params);

    // Frame 1: high-conf creates track
    auto dl1 = make_det_list({make_det(100, 100, 50, 50, 0.9f)});
    auto out = tracker.update(dl1);
    ASSERT_EQ(out.objects.size(), 1u);
    uint32_t id = out.objects[0].track_id;

    // Frame 2: same position high-conf → should match existing track
    auto dl2 = make_det_list({make_det(102, 102, 50, 50, 0.8f)});
    out      = tracker.update(dl2);
    ASSERT_EQ(out.objects.size(), 1u);
    EXPECT_EQ(out.objects[0].track_id, id);  // Same ID
}

TEST(ByteTrackAssociation, LowConfRecoversUnmatchedTrack) {
    ByteTrackTracker::Params params;
    params.high_conf_threshold = 0.5f;
    params.low_conf_threshold  = 0.1f;
    params.min_hits            = 1;
    ByteTrackTracker tracker(params);

    // Frame 1: two high-conf detections → two tracks
    auto dl1 = make_det_list({make_det(100, 100, 50, 50, 0.9f), make_det(300, 300, 50, 50, 0.9f)});
    auto out = tracker.update(dl1);
    ASSERT_EQ(out.objects.size(), 2u);
    uint32_t id_a = out.objects[0].track_id;
    uint32_t id_b = out.objects[1].track_id;

    // Frame 2: one high-conf matches track A, one low-conf matches track B
    auto dl2 = make_det_list({make_det(102, 102, 50, 50, 0.8f), make_det(302, 302, 50, 50, 0.2f)});
    out      = tracker.update(dl2);
    ASSERT_EQ(out.objects.size(), 2u);

    // Both original track IDs should be present
    bool found_a = false, found_b = false;
    for (const auto& obj : out.objects) {
        if (obj.track_id == id_a) found_a = true;
        if (obj.track_id == id_b) found_b = true;
    }
    EXPECT_TRUE(found_a);
    EXPECT_TRUE(found_b);
}

TEST(ByteTrackAssociation, LowConfDoesNotCreateNewTrack) {
    ByteTrackTracker::Params params;
    params.high_conf_threshold = 0.5f;
    params.low_conf_threshold  = 0.1f;
    params.min_hits            = 1;
    ByteTrackTracker tracker(params);

    // Only low-conf detections on empty tracker → no tracks
    auto dl  = make_det_list({make_det(100, 100, 50, 50, 0.2f), make_det(300, 300, 50, 50, 0.3f)});
    auto out = tracker.update(dl);
    EXPECT_EQ(out.objects.size(), 0u);
}

TEST(ByteTrackAssociation, OnlyHighConfCreatesNewTracks) {
    ByteTrackTracker::Params params;
    params.high_conf_threshold = 0.5f;
    params.low_conf_threshold  = 0.1f;
    params.min_hits            = 1;
    ByteTrackTracker tracker(params);

    // Mixed: one high-conf, one low-conf — only the high-conf creates a track
    auto dl  = make_det_list({make_det(100, 100, 50, 50, 0.9f), make_det(300, 300, 50, 50, 0.2f)});
    auto out = tracker.update(dl);
    EXPECT_EQ(out.objects.size(), 1u);
}

// ═══════════════════════════════════════════════════════════
// Track lifecycle (3 tests)
// ═══════════════════════════════════════════════════════════

TEST(ByteTrackLifecycle, EmptyDetections) {
    ByteTrackTracker tracker;
    auto             dl  = make_det_list({});
    auto             out = tracker.update(dl);
    EXPECT_EQ(out.objects.size(), 0u);
}

TEST(ByteTrackLifecycle, SingleDetectionBecomesTrack) {
    ByteTrackTracker tracker;  // min_hits = 3 default
    auto             det = make_det(100, 100, 50, 50, 0.9f);

    // Frame 1: tentative (hits=1)
    auto out = tracker.update(make_det_list({det}));
    EXPECT_EQ(out.objects.size(), 0u);  // Not confirmed yet

    // Frame 2: hits=2
    out = tracker.update(make_det_list({make_det(102, 102, 50, 50, 0.9f)}));
    EXPECT_EQ(out.objects.size(), 0u);

    // Frame 3: hits=3 → confirmed
    out = tracker.update(make_det_list({make_det(104, 104, 50, 50, 0.9f)}));
    EXPECT_EQ(out.objects.size(), 1u);
}

TEST(ByteTrackLifecycle, StaleTracksRemoved) {
    ByteTrackTracker::Params params;
    params.max_age  = 5;
    params.min_hits = 1;
    ByteTrackTracker tracker(params);

    // Create a track
    auto out = tracker.update(make_det_list({make_det(100, 100, 50, 50, 0.9f)}));
    ASSERT_EQ(out.objects.size(), 1u);

    // Send empty frames — track should be pruned after max_age+1 misses
    for (int i = 0; i < 7; ++i) {
        out = tracker.update(make_det_list({}));
    }
    EXPECT_EQ(out.objects.size(), 0u);
}

// ═══════════════════════════════════════════════════════════
// Occlusion recovery (2 tests)
// ═══════════════════════════════════════════════════════════

TEST(ByteTrackOcclusion, OcclusionRecovery) {
    ByteTrackTracker::Params params;
    params.high_conf_threshold = 0.5f;
    params.low_conf_threshold  = 0.1f;
    params.max_age             = 10;
    params.min_hits            = 1;
    ByteTrackTracker tracker(params);

    // Phase 1: 3 frames high-conf → confirmed track
    auto det = make_det(100, 100, 50, 50, 0.9f);
    (void)tracker.update(make_det_list({det}));
    (void)tracker.update(make_det_list({make_det(102, 102, 50, 50, 0.9f)}));
    auto out = tracker.update(make_det_list({make_det(104, 104, 50, 50, 0.9f)}));
    ASSERT_EQ(out.objects.size(), 1u);
    uint32_t original_id = out.objects[0].track_id;

    // Phase 2: 3 frames low-conf (simulating partial occlusion)
    // ByteTrack Stage 2 should recover the track each frame
    for (int i = 0; i < 3; ++i) {
        out =
            tracker.update(make_det_list({make_det(106.0f + i * 2, 106.0f + i * 2, 50, 50, 0.2f)}));
        ASSERT_EQ(out.objects.size(), 1u);
        EXPECT_EQ(out.objects[0].track_id, original_id) << "Lost ID at low-conf frame " << i;
    }

    // Phase 3: high-conf returns → still same track ID
    out = tracker.update(make_det_list({make_det(114, 114, 50, 50, 0.9f)}));
    ASSERT_EQ(out.objects.size(), 1u);
    EXPECT_EQ(out.objects[0].track_id, original_id);
}

// ═══════════════════════════════════════════════════════════
// Config & factory (5 tests)
// ═══════════════════════════════════════════════════════════

TEST(ByteTrackConfig, DefaultParams) {
    ByteTrackTracker::Params params;
    EXPECT_FLOAT_EQ(params.high_conf_threshold, 0.5f);
    EXPECT_FLOAT_EQ(params.low_conf_threshold, 0.1f);
    EXPECT_DOUBLE_EQ(params.max_iou_cost, 0.7);
    EXPECT_EQ(params.max_age, 10u);
    EXPECT_EQ(params.min_hits, 3u);
}

TEST(ByteTrackConfig, FactoryReturnsResult) {
    // Factory returns Result<> — verify Ok path
    auto result = create_tracker("bytetrack");
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value()->name(), "bytetrack");
}

TEST(ByteTrackConfig, FactoryWithNullConfig) {
    // Factory with nullptr config uses all defaults — should still work
    auto result = create_tracker("bytetrack", nullptr);
    ASSERT_TRUE(result.is_ok());
    auto tracker = std::move(result).value();
    EXPECT_EQ(tracker->name(), "bytetrack");

    // Verify it produces output with default params (min_hits=3)
    auto det = make_det(100, 100, 50, 50, 0.9f);
    (void)tracker->update(make_det_list({det}));
    (void)tracker->update(make_det_list({make_det(102, 102, 50, 50, 0.9f)}));
    auto out = tracker->update(make_det_list({make_det(104, 104, 50, 50, 0.9f)}));
    EXPECT_EQ(out.objects.size(), 1u);  // Confirmed after 3 hits
}

TEST(ByteTrackConfig, NameReturnsBytetrack) {
    ByteTrackTracker tracker;
    EXPECT_EQ(tracker.name(), "bytetrack");
}

TEST(ByteTrackConfig, FactoryCreatesBytetrack) {
    auto result = create_tracker("bytetrack");
    ASSERT_TRUE(result.is_ok());
    auto tracker = std::move(result).value();
    EXPECT_EQ(tracker->name(), "bytetrack");

    // Verify it works
    auto out = tracker->update(make_det_list({make_det(100, 100, 50, 50, 0.9f)}));
    // First frame, min_hits=3 default → no confirmed tracks yet
    EXPECT_EQ(out.objects.size(), 0u);
}

TEST(ByteTrackConfig, UnknownBackendFallsBackToByteTrack) {
    // Unknown backend should warn and fall back to ByteTrack, not throw
    auto result = create_tracker("nonexistent", nullptr);
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value()->name(), "bytetrack");
}
