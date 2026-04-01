// tests/test_obstacle_prediction.cpp
// Unit tests for dynamic obstacle prediction via velocity vectors (Issue #256).
#include "planner/occupancy_grid_3d.h"

#include <cmath>

#include <gtest/gtest.h>

using namespace drone::planner;

// ═════════════════════════════════════════════════════════════
// Helper: create a single-object DetectedObjectList
// ═════════════════════════════════════════════════════════════

static drone::ipc::DetectedObjectList make_single_object(float px, float py, float pz, float vx,
                                                         float vy, float vz, float conf = 0.9f) {
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects                   = 1;
    objects.objects[0].track_id           = 1;
    objects.objects[0].class_id           = drone::ipc::ObjectClass::UNKNOWN;
    objects.objects[0].confidence         = conf;
    objects.objects[0].position_x         = px;
    objects.objects[0].position_y         = py;
    objects.objects[0].position_z         = pz;
    objects.objects[0].velocity_x         = vx;
    objects.objects[0].velocity_y         = vy;
    objects.objects[0].velocity_z         = vz;
    objects.objects[0].heading            = 0.0f;
    objects.objects[0].bbox_x             = 0.0f;
    objects.objects[0].bbox_y             = 0.0f;
    objects.objects[0].bbox_w             = 0.0f;
    objects.objects[0].bbox_h             = 0.0f;
    objects.objects[0].has_camera         = true;
    objects.objects[0].has_radar          = false;
    objects.objects[0].estimated_radius_m = 0.0f;
    objects.objects[0].estimated_height_m = 0.0f;
    objects.objects[0].radar_update_count = 0;
    return objects;
}

// ═════════════════════════════════════════════════════════════
// Static object: no prediction cells inflated
// ═════════════════════════════════════════════════════════════

TEST(ObstaclePredictionTest, StaticObjectNoPredictionCells) {
    // Grid: 1m resolution, 20m extent, 1-cell inflation, prediction enabled
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, /*ttl=*/3.0f, /*min_conf=*/0.3f,
                         /*promotion_hits=*/0, /*radar_promo=*/3,
                         /*prediction_enabled=*/true, /*prediction_dt_s=*/2.0f);

    // Static object at (5,5,5) with zero velocity
    auto             objects = make_single_object(5.0f, 5.0f, 5.0f, 0.0f, 0.0f, 0.0f);
    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    // No prediction should have been applied
    EXPECT_EQ(grid.total_predictions_applied(), 0);

    // Cells near position should be occupied (current-position inflation)
    EXPECT_TRUE(grid.is_occupied({5, 5, 5}));

    // Cell far along any axis should NOT be occupied (no prediction sweep)
    EXPECT_FALSE(grid.is_occupied({10, 5, 5}));
    EXPECT_FALSE(grid.is_occupied({5, 10, 5}));
}

// ═════════════════════════════════════════════════════════════
// Zero velocity: no prediction cells
// ═════════════════════════════════════════════════════════════

TEST(ObstaclePredictionTest, ZeroVelocityNoPrediction) {
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, /*ttl=*/3.0f, /*min_conf=*/0.3f,
                         /*promotion_hits=*/0, /*radar_promo=*/3,
                         /*prediction_enabled=*/true, /*prediction_dt_s=*/2.0f);

    // Exactly zero velocity
    auto             objects = make_single_object(5.0f, 5.0f, 5.0f, 0.0f, 0.0f, 0.0f);
    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    EXPECT_EQ(grid.total_predictions_applied(), 0);
    EXPECT_TRUE(grid.prediction_enabled());
}

// ═════════════════════════════════════════════════════════════
// Very slow velocity (below threshold): no prediction
// ═════════════════════════════════════════════════════════════

TEST(ObstaclePredictionTest, SlowVelocityBelowThresholdNoPrediction) {
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, /*ttl=*/3.0f, /*min_conf=*/0.3f,
                         /*promotion_hits=*/0, /*radar_promo=*/3,
                         /*prediction_enabled=*/true, /*prediction_dt_s=*/2.0f);

    // 0.3 m/s — below 0.5 m/s threshold
    auto             objects = make_single_object(5.0f, 5.0f, 5.0f, 0.3f, 0.0f, 0.0f);
    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    EXPECT_EQ(grid.total_predictions_applied(), 0);
}

// ═════════════════════════════════════════════════════════════
// Moving object: cells inflated along velocity vector
// ═════════════════════════════════════════════════════════════

TEST(ObstaclePredictionTest, MovingObjectInflatesPredictionCells) {
    // 1m resolution, prediction dt=2s
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, /*ttl=*/3.0f, /*min_conf=*/0.3f,
                         /*promotion_hits=*/0, /*radar_promo=*/3,
                         /*prediction_enabled=*/true, /*prediction_dt_s=*/2.0f);

    // Object at (5,5,5) moving at 2 m/s in +X direction
    // Predicted position at t+2s: (9,5,5)
    auto             objects = make_single_object(5.0f, 5.0f, 5.0f, 2.0f, 0.0f, 0.0f);
    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    // Prediction should have been applied
    EXPECT_EQ(grid.total_predictions_applied(), 1);

    // Current position occupied
    EXPECT_TRUE(grid.is_occupied({5, 5, 5}));

    // Cells along the velocity vector should also be occupied
    EXPECT_TRUE(grid.is_occupied({7, 5, 5}));
    EXPECT_TRUE(grid.is_occupied({8, 5, 5}));
    EXPECT_TRUE(grid.is_occupied({9, 5, 5}));  // predicted position

    // Cell beyond prediction should NOT be occupied
    EXPECT_FALSE(grid.is_occupied({15, 5, 5}));
}

// ═════════════════════════════════════════════════════════════
// Moving object in diagonal direction
// ═════════════════════════════════════════════════════════════

TEST(ObstaclePredictionTest, DiagonalVelocityInflatesDiagonalCells) {
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, /*ttl=*/3.0f, /*min_conf=*/0.3f,
                         /*promotion_hits=*/0, /*radar_promo=*/3,
                         /*prediction_enabled=*/true, /*prediction_dt_s=*/2.0f);

    // Object at (5,5,5) moving at (1,1,0) m/s
    // Predicted position at t+2s: (7,7,5)
    auto             objects = make_single_object(5.0f, 5.0f, 5.0f, 1.0f, 1.0f, 0.0f);
    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    EXPECT_EQ(grid.total_predictions_applied(), 1);

    // Cells along diagonal should be occupied
    EXPECT_TRUE(grid.is_occupied({6, 6, 5}));
    EXPECT_TRUE(grid.is_occupied({7, 7, 5}));  // predicted position
}

// ═════════════════════════════════════════════════════════════
// Config toggle disables prediction
// ═════════════════════════════════════════════════════════════

TEST(ObstaclePredictionTest, PredictionDisabledNoCells) {
    // Prediction explicitly disabled
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, /*ttl=*/3.0f, /*min_conf=*/0.3f,
                         /*promotion_hits=*/0, /*radar_promo=*/3,
                         /*prediction_enabled=*/false, /*prediction_dt_s=*/2.0f);

    // Fast-moving object
    auto             objects = make_single_object(5.0f, 5.0f, 5.0f, 3.0f, 0.0f, 0.0f);
    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    EXPECT_FALSE(grid.prediction_enabled());
    EXPECT_EQ(grid.total_predictions_applied(), 0);

    // Current position occupied (regular inflation still works)
    EXPECT_TRUE(grid.is_occupied({5, 5, 5}));

    // Predicted position at (11,5,5) should NOT be occupied
    EXPECT_FALSE(grid.is_occupied({11, 5, 5}));
    // Intermediate cell should NOT be occupied either
    EXPECT_FALSE(grid.is_occupied({8, 5, 5}));
}

// ═════════════════════════════════════════════════════════════
// Prediction with zero dt_s: no prediction cells
// ═════════════════════════════════════════════════════════════

TEST(ObstaclePredictionTest, ZeroDtNoPrediction) {
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, /*ttl=*/3.0f, /*min_conf=*/0.3f,
                         /*promotion_hits=*/0, /*radar_promo=*/3,
                         /*prediction_enabled=*/true, /*prediction_dt_s=*/0.0f);

    auto             objects = make_single_object(5.0f, 5.0f, 5.0f, 3.0f, 0.0f, 0.0f);
    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    EXPECT_EQ(grid.total_predictions_applied(), 0);
}

// ═════════════════════════════════════════════════════════════
// Prediction cells respect grid bounds
// ═════════════════════════════════════════════════════════════

TEST(ObstaclePredictionTest, PredictionRespectsGridBounds) {
    // Small grid: 5m extent, 1m resolution → half_extent = 5 cells
    OccupancyGrid3D grid(1.0f, 5.0f, 1.0f, /*ttl=*/3.0f, /*min_conf=*/0.3f,
                         /*promotion_hits=*/0, /*radar_promo=*/3,
                         /*prediction_enabled=*/true, /*prediction_dt_s=*/5.0f);

    // Object at (3,0,0) moving fast in +X → predicted at (28,0,0) which is out of bounds
    auto             objects = make_single_object(3.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f);
    drone::ipc::Pose pose{};
    pose.translation[0] = -3.0;  // drone far away
    pose.translation[1] = -3.0;
    pose.translation[2] = 0.0;
    grid.update_from_objects(objects, pose);

    // Should still have prediction count (object had velocity)
    EXPECT_EQ(grid.total_predictions_applied(), 1);

    // Cells at grid boundary should be occupied
    EXPECT_TRUE(grid.is_occupied({4, 0, 0}));

    // But out-of-bounds cells should not crash or be occupied
    EXPECT_FALSE(grid.is_occupied({20, 0, 0}));
}

// ═════════════════════════════════════════════════════════════
// Prediction inflation uses same radius as current-position
// ═════════════════════════════════════════════════════════════

TEST(ObstaclePredictionTest, PredictionInflationRadiusMatchesCurrent) {
    // 2-cell inflation radius
    OccupancyGrid3D grid(1.0f, 20.0f, 2.0f, /*ttl=*/3.0f, /*min_conf=*/0.3f,
                         /*promotion_hits=*/0, /*radar_promo=*/3,
                         /*prediction_enabled=*/true, /*prediction_dt_s=*/2.0f);

    // Object at (5,5,5) moving at 3 m/s in +X
    // Predicted position at t+2s: (11,5,5)
    auto             objects = make_single_object(5.0f, 5.0f, 5.0f, 3.0f, 0.0f, 0.0f);
    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    // At the predicted position, cells 2 cells away in Y should be occupied
    // (2-cell inflation radius)
    EXPECT_TRUE(grid.is_occupied({11, 5, 5}));   // center of predicted pos
    EXPECT_TRUE(grid.is_occupied({11, 6, 5}));   // 1 cell offset
    EXPECT_TRUE(grid.is_occupied({11, 7, 5}));   // 2 cell offset (on boundary)
    EXPECT_FALSE(grid.is_occupied({11, 8, 5}));  // 3 cell offset (beyond inflation)
}

// ═════════════════════════════════════════════════════════════
// Multiple objects: only moving ones get prediction
// ═════════════════════════════════════════════════════════════

TEST(ObstaclePredictionTest, MultipleObjectsOnlyMovingGetPrediction) {
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, /*ttl=*/3.0f, /*min_conf=*/0.3f,
                         /*promotion_hits=*/0, /*radar_promo=*/3,
                         /*prediction_enabled=*/true, /*prediction_dt_s=*/2.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects = 2;

    // Object 0: static
    objects.objects[0].track_id           = 1;
    objects.objects[0].class_id           = drone::ipc::ObjectClass::UNKNOWN;
    objects.objects[0].confidence         = 0.9f;
    objects.objects[0].position_x         = 5.0f;
    objects.objects[0].position_y         = 5.0f;
    objects.objects[0].position_z         = 5.0f;
    objects.objects[0].velocity_x         = 0.0f;
    objects.objects[0].velocity_y         = 0.0f;
    objects.objects[0].velocity_z         = 0.0f;
    objects.objects[0].heading            = 0.0f;
    objects.objects[0].bbox_x             = 0.0f;
    objects.objects[0].bbox_y             = 0.0f;
    objects.objects[0].bbox_w             = 0.0f;
    objects.objects[0].bbox_h             = 0.0f;
    objects.objects[0].has_camera         = true;
    objects.objects[0].has_radar          = false;
    objects.objects[0].estimated_radius_m = 0.0f;
    objects.objects[0].estimated_height_m = 0.0f;
    objects.objects[0].radar_update_count = 0;

    // Object 1: moving at 2 m/s in +Y
    objects.objects[1].track_id           = 2;
    objects.objects[1].class_id           = drone::ipc::ObjectClass::UNKNOWN;
    objects.objects[1].confidence         = 0.9f;
    objects.objects[1].position_x         = -5.0f;
    objects.objects[1].position_y         = -5.0f;
    objects.objects[1].position_z         = 5.0f;
    objects.objects[1].velocity_x         = 0.0f;
    objects.objects[1].velocity_y         = 2.0f;
    objects.objects[1].velocity_z         = 0.0f;
    objects.objects[1].heading            = 0.0f;
    objects.objects[1].bbox_x             = 0.0f;
    objects.objects[1].bbox_y             = 0.0f;
    objects.objects[1].bbox_w             = 0.0f;
    objects.objects[1].bbox_h             = 0.0f;
    objects.objects[1].has_camera         = true;
    objects.objects[1].has_radar          = false;
    objects.objects[1].estimated_radius_m = 0.0f;
    objects.objects[1].estimated_height_m = 0.0f;
    objects.objects[1].radar_update_count = 0;

    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    // Only one object should trigger prediction
    EXPECT_EQ(grid.total_predictions_applied(), 1);

    // Object 1 predicted position: (-5, -5 + 2*2, 5) = (-5, -1, 5)
    EXPECT_TRUE(grid.is_occupied({-5, -1, 5}));
}
