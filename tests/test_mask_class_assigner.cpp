// tests/test_mask_class_assigner.cpp
// Unit tests for MaskClassAssigner — IoU-based SAM mask → detector class assignment.
#include "perception/mask_class_assigner.h"

#include <gtest/gtest.h>

using namespace drone::perception;
using namespace drone::hal;

// ── Helpers ──

static InferenceDetection make_mask(float x, float y, float w, float h, float conf = 0.9f) {
    InferenceDetection det;
    det.bbox       = {x, y, w, h};
    det.class_id   = -1;
    det.confidence = conf;
    return det;
}

static InferenceDetection make_det(float x, float y, float w, float h, int class_id,
                                   float conf = 0.8f) {
    InferenceDetection det;
    det.bbox       = {x, y, w, h};
    det.class_id   = class_id;
    det.confidence = conf;
    return det;
}

// ── Tests ──

TEST(MaskClassAssigner, PerfectOverlapAssignsClass) {
    MaskClassAssigner               assigner(0.5f);
    std::vector<InferenceDetection> masks = {make_mask(10, 10, 100, 100)};
    std::vector<InferenceDetection> dets  = {make_det(10, 10, 100, 100, 0)};  // COCO person

    auto result = assigner.assign(masks, dets);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].assigned_class, ObjectClass::PERSON);
    EXPECT_FLOAT_EQ(result[0].assignment_iou, 1.0f);
    EXPECT_EQ(result[0].detector_class_id, 0);
}

TEST(MaskClassAssigner, NoOverlapKeepsGeometric) {
    MaskClassAssigner               assigner(0.5f);
    std::vector<InferenceDetection> masks = {make_mask(10, 10, 50, 50)};
    std::vector<InferenceDetection> dets  = {make_det(200, 200, 50, 50, 0)};

    auto result = assigner.assign(masks, dets);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].assigned_class, ObjectClass::GEOMETRIC_OBSTACLE);
    EXPECT_FLOAT_EQ(result[0].assignment_iou, 0.0f);
}

TEST(MaskClassAssigner, PartialIoUAboveThreshold) {
    MaskClassAssigner assigner(0.3f);
    // 50% horizontal overlap: mask [0,0,100,100], det [50,0,100,100]
    // Intersection: [50,0] to [100,100] = 50*100 = 5000
    // Union: 10000 + 10000 - 5000 = 15000 → IoU = 5000/15000 ≈ 0.333
    std::vector<InferenceDetection> masks = {make_mask(0, 0, 100, 100)};
    std::vector<InferenceDetection> dets  = {make_det(50, 0, 100, 100, 7)};  // COCO truck

    auto result = assigner.assign(masks, dets);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].assigned_class, ObjectClass::VEHICLE_TRUCK);
    EXPECT_GT(result[0].assignment_iou, 0.3f);
}

TEST(MaskClassAssigner, PartialIoUBelowThreshold) {
    MaskClassAssigner assigner(0.5f);
    // Same overlap as above but threshold is 0.5 → IoU ≈ 0.333 < 0.5
    std::vector<InferenceDetection> masks = {make_mask(0, 0, 100, 100)};
    std::vector<InferenceDetection> dets  = {make_det(50, 0, 100, 100, 7)};

    auto result = assigner.assign(masks, dets);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].assigned_class, ObjectClass::GEOMETRIC_OBSTACLE);
}

TEST(MaskClassAssigner, MultiMaskMultiDetector) {
    MaskClassAssigner               assigner(0.5f);
    std::vector<InferenceDetection> masks = {
        make_mask(0, 0, 100, 100),
        make_mask(200, 0, 100, 100),
        make_mask(400, 0, 100, 100),
    };
    std::vector<InferenceDetection> dets = {
        make_det(0, 0, 100, 100, 0),    // person, matches mask 0
        make_det(200, 0, 100, 100, 2),  // COCO car, matches mask 1
    };

    auto result = assigner.assign(masks, dets);
    ASSERT_EQ(result.size(), 3u);
    EXPECT_EQ(result[0].assigned_class, ObjectClass::PERSON);
    EXPECT_EQ(result[1].assigned_class, ObjectClass::VEHICLE_CAR);
    EXPECT_EQ(result[2].assigned_class, ObjectClass::GEOMETRIC_OBSTACLE);
}

TEST(MaskClassAssigner, GreedyHighestConfidence) {
    // Two detectors overlap the same mask. Higher confidence wins.
    MaskClassAssigner               assigner(0.5f);
    std::vector<InferenceDetection> masks = {make_mask(0, 0, 100, 100)};
    std::vector<InferenceDetection> dets  = {
        make_det(0, 0, 100, 100, 7, 0.6f),  // truck, lower confidence
        make_det(0, 0, 100, 100, 0, 0.9f),  // person, higher confidence
    };

    auto result = assigner.assign(masks, dets);
    ASSERT_EQ(result.size(), 1u);
    // Higher-confidence detector (person, 0.9) gets first pick
    EXPECT_EQ(result[0].assigned_class, ObjectClass::PERSON);
}

TEST(MaskClassAssigner, EmptyMasks) {
    MaskClassAssigner               assigner(0.5f);
    std::vector<InferenceDetection> masks;
    std::vector<InferenceDetection> dets = {make_det(0, 0, 100, 100, 0)};

    auto result = assigner.assign(masks, dets);
    EXPECT_TRUE(result.empty());
}

TEST(MaskClassAssigner, EmptyDetectors) {
    MaskClassAssigner               assigner(0.5f);
    std::vector<InferenceDetection> masks = {make_mask(0, 0, 100, 100)};
    std::vector<InferenceDetection> dets;

    auto result = assigner.assign(masks, dets);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].assigned_class, ObjectClass::GEOMETRIC_OBSTACLE);
}

TEST(MaskClassAssigner, AllGeometricWhenNoDetectors) {
    MaskClassAssigner               assigner(0.5f);
    std::vector<InferenceDetection> masks = {
        make_mask(0, 0, 50, 50),
        make_mask(100, 0, 50, 50),
        make_mask(200, 0, 50, 50),
    };
    std::vector<InferenceDetection> dets;

    auto result = assigner.assign(masks, dets);
    ASSERT_EQ(result.size(), 3u);
    for (const auto& a : result) {
        EXPECT_EQ(a.assigned_class, ObjectClass::GEOMETRIC_OBSTACLE);
    }
}

TEST(MaskClassAssigner, BboxIoUComputation) {
    BoundingBox2D a{0, 0, 100, 100};
    BoundingBox2D b{50, 50, 100, 100};
    // Intersection: [50,50] to [100,100] = 50*50 = 2500
    // Union: 10000 + 10000 - 2500 = 17500
    float iou = MaskClassAssigner::compute_bbox_iou(a, b);
    EXPECT_NEAR(iou, 2500.0f / 17500.0f, 1e-5f);
}

TEST(MaskClassAssigner, BboxIoUNoOverlap) {
    BoundingBox2D a{0, 0, 50, 50};
    BoundingBox2D b{100, 100, 50, 50};
    EXPECT_FLOAT_EQ(MaskClassAssigner::compute_bbox_iou(a, b), 0.0f);
}
