// tests/test_color_contour_detector.cpp
// Unit tests for ColorContourDetector: HSV segmentation + connected-component labeling.
#include "perception/color_contour_detector.h"
#include "perception/detector_interface.h"
#include "util/config.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::perception;

// ═══════════════════════════════════════════════════════════
// Helpers: synthetic image creation
// ═══════════════════════════════════════════════════════════

/// Create a solid-color RGB image (3 channels).
static std::vector<uint8_t> make_solid_image(uint32_t w, uint32_t h, uint8_t r, uint8_t g,
                                             uint8_t b) {
    std::vector<uint8_t> img(w * h * 3);
    for (uint32_t i = 0; i < w * h; ++i) {
        img[i * 3 + 0] = r;
        img[i * 3 + 1] = g;
        img[i * 3 + 2] = b;
    }
    return img;
}

/// Fill a rectangle in an RGB image with the given color.
static void fill_rect(std::vector<uint8_t>& img, uint32_t img_w, uint32_t rx, uint32_t ry,
                      uint32_t rw, uint32_t rh, uint8_t r, uint8_t g, uint8_t b) {
    for (uint32_t y = ry; y < ry + rh; ++y) {
        for (uint32_t x = rx; x < rx + rw; ++x) {
            uint32_t idx = (y * img_w + x) * 3;
            img[idx + 0] = r;
            img[idx + 1] = g;
            img[idx + 2] = b;
        }
    }
}

/// Create a temp config file and return its path.
static std::vector<std::string> g_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    std::string path = "/tmp/test_ccd_" + std::to_string(getpid()) + "_" +
                       std::to_string(g_temp_files.size()) + ".json";
    std::ofstream ofs(path);
    ofs << json_content;
    ofs.close();
    g_temp_files.push_back(path);
    return path;
}

struct TempFileCleanup {
    ~TempFileCleanup() {
        for (const auto& f : g_temp_files) {
            std::remove(f.c_str());
        }
    }
};
static TempFileCleanup g_cleanup;

// ═══════════════════════════════════════════════════════════
// rgb_to_hsv tests
// ═══════════════════════════════════════════════════════════

TEST(RgbToHsvTest, PureRed) {
    auto hsv = rgb_to_hsv(255, 0, 0);
    EXPECT_NEAR(hsv.h, 0.0f, 1.0f);   // Red → hue ≈ 0
    EXPECT_NEAR(hsv.s, 1.0f, 0.01f);  // Fully saturated
    EXPECT_NEAR(hsv.v, 1.0f, 0.01f);  // Fully bright
}

TEST(RgbToHsvTest, PureGreen) {
    auto hsv = rgb_to_hsv(0, 255, 0);
    EXPECT_NEAR(hsv.h, 120.0f, 1.0f);
    EXPECT_NEAR(hsv.s, 1.0f, 0.01f);
    EXPECT_NEAR(hsv.v, 1.0f, 0.01f);
}

TEST(RgbToHsvTest, PureBlue) {
    auto hsv = rgb_to_hsv(0, 0, 255);
    EXPECT_NEAR(hsv.h, 240.0f, 1.0f);
    EXPECT_NEAR(hsv.s, 1.0f, 0.01f);
    EXPECT_NEAR(hsv.v, 1.0f, 0.01f);
}

TEST(RgbToHsvTest, White) {
    auto hsv = rgb_to_hsv(255, 255, 255);
    EXPECT_NEAR(hsv.s, 0.0f, 0.01f);  // White = no saturation
    EXPECT_NEAR(hsv.v, 1.0f, 0.01f);
}

TEST(RgbToHsvTest, Black) {
    auto hsv = rgb_to_hsv(0, 0, 0);
    EXPECT_NEAR(hsv.s, 0.0f, 0.01f);
    EXPECT_NEAR(hsv.v, 0.0f, 0.01f);
}

TEST(RgbToHsvTest, Yellow) {
    auto hsv = rgb_to_hsv(255, 255, 0);
    EXPECT_NEAR(hsv.h, 60.0f, 1.0f);
    EXPECT_NEAR(hsv.s, 1.0f, 0.01f);
    EXPECT_NEAR(hsv.v, 1.0f, 0.01f);
}

TEST(RgbToHsvTest, Grey50Percent) {
    auto hsv = rgb_to_hsv(128, 128, 128);
    EXPECT_NEAR(hsv.s, 0.0f, 0.01f);
    EXPECT_NEAR(hsv.v, 128.0f / 255.0f, 0.01f);
}

// ═══════════════════════════════════════════════════════════
// HsvRange tests
// ═══════════════════════════════════════════════════════════

TEST(HsvRangeTest, NormalRange) {
    HsvRange range{200.0f, 260.0f, 0.3f, 1.0f, 0.2f, 1.0f, ObjectClass::VEHICLE_CAR, "blue"};
    EXPECT_TRUE(range.contains(230.0f, 0.5f, 0.5f));
    EXPECT_FALSE(range.contains(100.0f, 0.5f, 0.5f));  // Wrong hue
    EXPECT_FALSE(range.contains(230.0f, 0.1f, 0.5f));  // Low saturation
    EXPECT_FALSE(range.contains(230.0f, 0.5f, 0.1f));  // Low value
}

TEST(HsvRangeTest, WrapAroundHue) {
    // Red wraps from 340 → 20
    HsvRange range{340.0f, 20.0f, 0.3f, 1.0f, 0.3f, 1.0f, ObjectClass::PERSON, "red"};
    EXPECT_TRUE(range.contains(350.0f, 0.5f, 0.5f));   // In [340, 360)
    EXPECT_TRUE(range.contains(5.0f, 0.5f, 0.5f));     // In [0, 20]
    EXPECT_FALSE(range.contains(180.0f, 0.5f, 0.5f));  // Green hue — out
}

TEST(HsvRangeTest, BoundaryValues) {
    HsvRange range{100.0f, 200.0f, 0.3f, 0.8f, 0.3f, 0.8f, ObjectClass::UNKNOWN, "test"};
    EXPECT_TRUE(range.contains(100.0f, 0.3f, 0.3f));   // Exact min
    EXPECT_TRUE(range.contains(200.0f, 0.8f, 0.8f));   // Exact max
    EXPECT_FALSE(range.contains(99.0f, 0.5f, 0.5f));   // Just below hue_min
    EXPECT_FALSE(range.contains(201.0f, 0.5f, 0.5f));  // Just above hue_max
}

// ═══════════════════════════════════════════════════════════
// UnionFind tests
// ═══════════════════════════════════════════════════════════

TEST(UnionFindTest, InitiallyDisjoint) {
    UnionFind uf(5);
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(uf.find(i), i);
    }
}

TEST(UnionFindTest, UniteAndFind) {
    UnionFind uf(5);
    uf.unite(0, 1);
    uf.unite(2, 3);
    EXPECT_EQ(uf.find(0), uf.find(1));
    EXPECT_EQ(uf.find(2), uf.find(3));
    EXPECT_NE(uf.find(0), uf.find(2));
}

TEST(UnionFindTest, TransitiveUnion) {
    UnionFind uf(5);
    uf.unite(0, 1);
    uf.unite(1, 2);
    uf.unite(2, 3);
    // All of 0-3 should share a root
    int root = uf.find(0);
    EXPECT_EQ(uf.find(1), root);
    EXPECT_EQ(uf.find(2), root);
    EXPECT_EQ(uf.find(3), root);
    // 4 is still separate
    EXPECT_NE(uf.find(4), root);
}

TEST(UnionFindTest, SelfUnite) {
    UnionFind uf(3);
    uf.unite(1, 1);  // No-op
    EXPECT_EQ(uf.find(1), 1);
}

// ═══════════════════════════════════════════════════════════
// ComponentBBox tests
// ═══════════════════════════════════════════════════════════

TEST(ComponentBBoxTest, Dimensions) {
    ComponentBBox bb{10, 20, 50, 60, 500};
    EXPECT_EQ(bb.width(), 41);   // 50 - 10 + 1
    EXPECT_EQ(bb.height(), 41);  // 60 - 20 + 1
    EXPECT_EQ(bb.area(), 500);
}

// ═══════════════════════════════════════════════════════════
// ColorContourDetector tests
// ═══════════════════════════════════════════════════════════

TEST(ColorContourDetectorTest, DefaultConstructorHasSixColors) {
    ColorContourDetector det;
    EXPECT_EQ(det.color_ranges().size(), 6u);
    EXPECT_EQ(det.name(), "ColorContourDetector");
}

TEST(ColorContourDetectorTest, NullFrameReturnsEmpty) {
    ColorContourDetector det;
    auto                 result = det.detect(nullptr, 640, 480, 3);
    EXPECT_TRUE(result.empty());
}

TEST(ColorContourDetectorTest, ZeroDimensionsReturnsEmpty) {
    ColorContourDetector det;
    std::vector<uint8_t> data(100, 0);
    EXPECT_TRUE(det.detect(data.data(), 0, 480, 3).empty());
    EXPECT_TRUE(det.detect(data.data(), 640, 0, 3).empty());
}

TEST(ColorContourDetectorTest, BlackImageNoDetections) {
    ColorContourDetector det;
    auto                 img    = make_solid_image(100, 100, 0, 0, 0);
    auto                 result = det.detect(img.data(), 100, 100, 3);
    EXPECT_TRUE(result.empty());
}

TEST(ColorContourDetectorTest, GreyImageNoDetections) {
    // Grey has zero saturation → should NOT match any color range (min_s ≥ 0.3)
    ColorContourDetector det;
    auto                 img    = make_solid_image(100, 100, 128, 128, 128);
    auto                 result = det.detect(img.data(), 100, 100, 3);
    EXPECT_TRUE(result.empty());
}

TEST(ColorContourDetectorTest, WhiteImageNoDetections) {
    // White has zero saturation
    ColorContourDetector det;
    auto                 img    = make_solid_image(100, 100, 255, 255, 255);
    auto                 result = det.detect(img.data(), 100, 100, 3);
    EXPECT_TRUE(result.empty());
}

TEST(ColorContourDetectorTest, SolidRedImageDetectsAsRed) {
    ColorContourDetector det;
    auto                 img    = make_solid_image(100, 100, 255, 0, 0);
    auto                 result = det.detect(img.data(), 100, 100, 3);
    ASSERT_FALSE(result.empty());
    // Should detect as PERSON (red → PERSON in defaults)
    EXPECT_EQ(result[0].class_id, ObjectClass::PERSON);
    // Bounding box should cover entire image
    EXPECT_NEAR(result[0].x, 0.0f, 1.0f);
    EXPECT_NEAR(result[0].y, 0.0f, 1.0f);
    EXPECT_NEAR(result[0].w, 100.0f, 1.0f);
    EXPECT_NEAR(result[0].h, 100.0f, 1.0f);
}

TEST(ColorContourDetectorTest, SolidBlueImageDetectsAsCar) {
    ColorContourDetector det;
    auto                 img    = make_solid_image(100, 100, 0, 0, 255);
    auto                 result = det.detect(img.data(), 100, 100, 3);
    ASSERT_FALSE(result.empty());
    // Blue hue (240°) should fall in VEHICLE_CAR range (200-260)
    bool found_car = false;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::VEHICLE_CAR) {
            found_car = true;
            break;
        }
    }
    EXPECT_TRUE(found_car);
}

TEST(ColorContourDetectorTest, SolidGreenImageDetectsAsDrone) {
    ColorContourDetector det;
    auto                 img    = make_solid_image(100, 100, 0, 255, 0);
    auto                 result = det.detect(img.data(), 100, 100, 3);
    ASSERT_FALSE(result.empty());
    bool found_drone = false;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::DRONE) {
            found_drone = true;
            break;
        }
    }
    EXPECT_TRUE(found_drone);
}

TEST(ColorContourDetectorTest, SmallRectOnBlackBackground) {
    // 200×200 black image with a 20×20 red rectangle at (50, 50)
    const uint32_t W = 200, H = 200;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    fill_rect(img, W, 50, 50, 20, 20, 255, 0, 0);

    ColorContourDetector det;
    auto                 result = det.detect(img.data(), W, H, 3);
    ASSERT_FALSE(result.empty());

    // Find the PERSON detection (red)
    const Detection2D* red_det = nullptr;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::PERSON) {
            red_det = &d;
            break;
        }
    }
    ASSERT_NE(red_det, nullptr);

    // Bounding box should tightly bound the rectangle
    EXPECT_NEAR(red_det->x, 50.0f, 1.0f);
    EXPECT_NEAR(red_det->y, 50.0f, 1.0f);
    EXPECT_NEAR(red_det->w, 20.0f, 1.0f);
    EXPECT_NEAR(red_det->h, 20.0f, 1.0f);
}

TEST(ColorContourDetectorTest, MultipleColorRectsDetected) {
    // 300×300 black image with:
    //   Red rectangle at (10, 10) size 30×30
    //   Blue rectangle at (200, 200) size 40×40
    const uint32_t W = 300, H = 300;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    fill_rect(img, W, 10, 10, 30, 30, 255, 0, 0);    // Red
    fill_rect(img, W, 200, 200, 40, 40, 0, 0, 255);  // Blue

    ColorContourDetector det;
    auto                 result = det.detect(img.data(), W, H, 3);

    // Should have at least 2 detections (one red, one blue)
    ASSERT_GE(result.size(), 2u);

    bool found_person = false, found_car = false;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::PERSON) found_person = true;
        if (d.class_id == ObjectClass::VEHICLE_CAR) found_car = true;
    }
    EXPECT_TRUE(found_person);
    EXPECT_TRUE(found_car);
}

TEST(ColorContourDetectorTest, MinAreaFilteringRemovesSmallBlobs) {
    // 200×200 black image with a tiny 3×3 red rectangle (9 pixels, below default min_area=80)
    const uint32_t W = 200, H = 200;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    fill_rect(img, W, 50, 50, 3, 3, 255, 0, 0);

    ColorContourDetector det;
    auto                 result = det.detect(img.data(), W, H, 3);

    // 9 pixels < min_contour_area(80) → should be filtered out
    bool found_person = false;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::PERSON) found_person = true;
    }
    EXPECT_FALSE(found_person);
}

TEST(ColorContourDetectorTest, ConfidenceSorted) {
    // Two rectangles: a big one and a small one — big one should have higher confidence
    const uint32_t W = 400, H = 400;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    fill_rect(img, W, 10, 10, 100, 100, 255, 0, 0);  // Big red
    fill_rect(img, W, 300, 300, 15, 15, 0, 0, 255);  // Small blue (225 > 80)

    ColorContourDetector det;
    auto                 result = det.detect(img.data(), W, H, 3);

    ASSERT_GE(result.size(), 2u);
    // Results should be sorted by confidence (descending)
    for (size_t i = 1; i < result.size(); ++i) {
        EXPECT_GE(result[i - 1].confidence, result[i].confidence);
    }
}

TEST(ColorContourDetectorTest, ConfidenceRange) {
    ColorContourDetector det;
    auto                 img    = make_solid_image(100, 100, 255, 0, 0);
    auto                 result = det.detect(img.data(), 100, 100, 3);
    ASSERT_FALSE(result.empty());
    for (const auto& d : result) {
        EXPECT_GE(d.confidence, 0.0f);
        EXPECT_LE(d.confidence, 1.0f);
    }
}

TEST(ColorContourDetectorTest, TimestampNonZero) {
    ColorContourDetector det;
    auto                 img    = make_solid_image(100, 100, 255, 0, 0);
    auto                 result = det.detect(img.data(), 100, 100, 3);
    ASSERT_FALSE(result.empty());
    EXPECT_GT(result[0].timestamp_ns, 0u);
}

TEST(ColorContourDetectorTest, TwoDisjointSameColorRects) {
    // Two separate red rectangles should yield two PERSON detections
    const uint32_t W = 300, H = 100;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    fill_rect(img, W, 10, 10, 30, 30, 255, 0, 0);   // Red #1
    fill_rect(img, W, 200, 10, 30, 30, 255, 0, 0);  // Red #2 (well separated)

    ColorContourDetector det;
    auto                 result = det.detect(img.data(), W, H, 3);

    int person_count = 0;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::PERSON) ++person_count;
    }
    EXPECT_EQ(person_count, 2);
}

// ═══════════════════════════════════════════════════════════
// Config-based construction tests
// ═══════════════════════════════════════════════════════════

TEST(ColorContourDetectorTest, ConfigMinAreaOverride) {
    // Set min_contour_area = 500 — small rects should be filtered
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "backend": "color_contour",
                "min_contour_area": 500,
                "max_detections": 10
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    ColorContourDetector det(cfg);

    // 200×200 black + 15×15 red rect = 225 pixels < 500 → filtered
    const uint32_t W = 200, H = 200;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    fill_rect(img, W, 50, 50, 15, 15, 255, 0, 0);
    auto result = det.detect(img.data(), W, H, 3);

    bool found_person = false;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::PERSON) found_person = true;
    }
    EXPECT_FALSE(found_person);
}

TEST(ColorContourDetectorTest, ConfigMaxDetections) {
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "backend": "color_contour",
                "min_contour_area": 5,
                "max_detections": 2
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    ColorContourDetector det(cfg);

    // Create image with 4 distinct colored rectangles
    const uint32_t W = 400, H = 200;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    fill_rect(img, W, 10, 10, 30, 30, 255, 0, 0);     // Red
    fill_rect(img, W, 100, 10, 30, 30, 0, 0, 255);    // Blue
    fill_rect(img, W, 200, 10, 30, 30, 0, 255, 0);    // Green
    fill_rect(img, W, 300, 10, 30, 30, 255, 255, 0);  // Yellow

    auto result = det.detect(img.data(), W, H, 3);
    EXPECT_LE(static_cast<int>(result.size()), 2);
}

TEST(ColorContourDetectorTest, ConfigFallsBackToDefaults) {
    // Config with no color ranges → should use defaults
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "backend": "color_contour"
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    ColorContourDetector det(cfg);
    EXPECT_EQ(det.color_ranges().size(), 6u);
}

// ═══════════════════════════════════════════════════════════
// Factory tests
// ═══════════════════════════════════════════════════════════

TEST(DetectorFactoryTest, CreateSimulated) {
    auto det = create_detector("simulated");
    ASSERT_NE(det, nullptr);
    EXPECT_EQ(det->name(), "SimulatedDetector");
}

TEST(DetectorFactoryTest, CreateColorContour) {
    auto det = create_detector("color_contour");
    ASSERT_NE(det, nullptr);
    EXPECT_EQ(det->name(), "ColorContourDetector");
}

TEST(DetectorFactoryTest, CreateColorContourWithConfig) {
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "min_contour_area": 200
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto det = create_detector("color_contour", &cfg);
    ASSERT_NE(det, nullptr);
    EXPECT_EQ(det->name(), "ColorContourDetector");
}

TEST(DetectorFactoryTest, EmptyStringCreatesSimulated) {
    auto det = create_detector("");
    ASSERT_NE(det, nullptr);
    EXPECT_EQ(det->name(), "SimulatedDetector");
}

TEST(DetectorFactoryTest, UnknownBackendThrows) {
    EXPECT_THROW(create_detector("tensorflow"), std::runtime_error);
}

// ═══════════════════════════════════════════════════════════
// Edge cases
// ═══════════════════════════════════════════════════════════

TEST(ColorContourDetectorTest, SinglePixelBelowMinArea) {
    const uint32_t W = 100, H = 100;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    // Single red pixel
    img[(50 * W + 50) * 3 + 0] = 255;
    img[(50 * W + 50) * 3 + 1] = 0;
    img[(50 * W + 50) * 3 + 2] = 0;

    ColorContourDetector det;
    auto                 result = det.detect(img.data(), W, H, 3);
    // 1 pixel < 80 min_area → empty
    EXPECT_TRUE(result.empty());
}

TEST(ColorContourDetectorTest, FourChannelImage) {
    // RGBA image (4 channels) — detector should still work (uses first 3)
    const uint32_t       W = 100, H = 100;
    std::vector<uint8_t> img(W * H * 4, 0);
    // Fill a 20×20 red block starting at (10,10)
    for (uint32_t y = 10; y < 30; ++y) {
        for (uint32_t x = 10; x < 30; ++x) {
            uint32_t idx = (y * W + x) * 4;
            img[idx + 0] = 255;
            img[idx + 1] = 0;
            img[idx + 2] = 0;
            img[idx + 3] = 255;  // Alpha
        }
    }

    ColorContourDetector det;
    auto                 result = det.detect(img.data(), W, H, 4);
    ASSERT_FALSE(result.empty());
    bool found_person = false;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::PERSON) found_person = true;
    }
    EXPECT_TRUE(found_person);
}

TEST(ColorContourDetectorTest, LessThanThreeChannelsReturnsEmpty) {
    ColorContourDetector det;
    std::vector<uint8_t> data(100, 128);
    EXPECT_TRUE(det.detect(data.data(), 10, 10, 2).empty());
    EXPECT_TRUE(det.detect(data.data(), 10, 10, 1).empty());
    EXPECT_TRUE(det.detect(data.data(), 10, 10, 0).empty());
}

// ═══════════════════════════════════════════════════════════
// Optimisation tests (Issue #128) — single-pass + subsampling + max_fps
// ═══════════════════════════════════════════════════════════

TEST(ColorContourDetectorTest, DefaultSubsampleIsTwo) {
    ColorContourDetector det;
    EXPECT_EQ(det.subsample(), 2);
}

TEST(ColorContourDetectorTest, SubsampleConfigKeyAccepted) {
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "backend": "color_contour",
                "subsample": 1
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    ColorContourDetector det(cfg);
    EXPECT_EQ(det.subsample(), 1);
}

TEST(ColorContourDetectorTest, SubsampleTwoConfigKey) {
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "backend": "color_contour",
                "subsample": 2
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    ColorContourDetector det(cfg);
    EXPECT_EQ(det.subsample(), 2);
}

TEST(ColorContourDetectorTest, MaxFpsConfigAccepted) {
    // Verify max_fps config key is read without error and detect() still works.
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "backend": "color_contour",
                "max_fps": 30,
                "subsample": 1
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    ColorContourDetector det(cfg);
    auto                 img    = make_solid_image(100, 100, 255, 0, 0);
    auto                 result = det.detect(img.data(), 100, 100, 3);
    EXPECT_FALSE(result.empty());
}

TEST(ColorContourDetectorTest, SubsampleZeroClampedToOne) {
    // subsample <= 0 must be clamped to 1 (no invalid stride).
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "backend": "color_contour",
                "subsample": 0
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    ColorContourDetector det(cfg);
    EXPECT_EQ(det.subsample(), 1);
}

TEST(ColorContourDetectorTest, Subsample1FullResDetectsSmallRect) {
    // With subsample=1 (no subsampling), a 10×10 rect (100 px > min_area=80) is detected.
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "backend": "color_contour",
                "subsample": 1,
                "min_contour_area": 80,
                "min_bbox_height_px": 0,
                "max_aspect_ratio": 0
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    ColorContourDetector det(cfg);

    const uint32_t W = 200, H = 200;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    fill_rect(img, W, 50, 50, 10, 10, 255, 0, 0);  // 100 px > 80

    auto result = det.detect(img.data(), W, H, 3);
    bool found  = false;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::PERSON) found = true;
    }
    EXPECT_TRUE(found);
}

TEST(ColorContourDetectorTest, Subsample2DetectsMediumRect) {
    // With subsample=2 (default), a 30×30 rect is still detected; bbox within ±subsample
    // of true position.
    ColorContourDetector det;  // default subsample=2
    ASSERT_EQ(det.subsample(), 2);

    const uint32_t W = 300, H = 300;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    fill_rect(img, W, 60, 60, 30, 30, 255, 0, 0);  // Red 30×30 = 900 px

    auto               result = det.detect(img.data(), W, H, 3);
    const Detection2D* found  = nullptr;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::PERSON) {
            found = &d;
            break;
        }
    }
    ASSERT_NE(found, nullptr);
    // Bbox should be approximately correct (within subsample pixels).
    EXPECT_NEAR(found->x, 60.0f, 2.0f);
    EXPECT_NEAR(found->y, 60.0f, 2.0f);
    EXPECT_NEAR(found->w, 30.0f, 2.0f);
    EXPECT_NEAR(found->h, 30.0f, 2.0f);
}

TEST(ColorContourDetectorTest, SinglePassDetectsAllSixColors) {
    // Six distinct color rectangles in one frame — all six must be detected
    // in a single detect() call (validates single-pass classification).
    const uint32_t W = 700, H = 100;
    auto           img = make_solid_image(W, H, 0, 0, 0);
    // Colors matching each of the 6 default ranges:
    fill_rect(img, W, 10, 10, 30, 30, 255, 0, 0);     // Red   → PERSON
    fill_rect(img, W, 110, 10, 30, 30, 0, 0, 255);    // Blue  → VEHICLE_CAR
    fill_rect(img, W, 210, 10, 30, 30, 255, 255, 0);  // Yellow → VEHICLE_TRUCK
    fill_rect(img, W, 310, 10, 30, 30, 0, 200, 0);    // Green → DRONE
    fill_rect(img, W, 410, 10, 30, 30, 255, 128, 0);  // Orange → ANIMAL
    fill_rect(img, W, 510, 10, 30, 30, 200, 0, 200);  // Magenta → BUILDING

    ColorContourDetector det;
    auto                 result = det.detect(img.data(), W, H, 3);

    bool found_person = false, found_car = false, found_truck = false;
    bool found_drone = false, found_animal = false, found_building = false;
    for (const auto& d : result) {
        switch (d.class_id) {
            case ObjectClass::PERSON: found_person = true; break;
            case ObjectClass::VEHICLE_CAR: found_car = true; break;
            case ObjectClass::VEHICLE_TRUCK: found_truck = true; break;
            case ObjectClass::DRONE: found_drone = true; break;
            case ObjectClass::ANIMAL: found_animal = true; break;
            case ObjectClass::BUILDING: found_building = true; break;
            default: break;
        }
    }
    EXPECT_TRUE(found_person);
    EXPECT_TRUE(found_car);
    EXPECT_TRUE(found_truck);
    EXPECT_TRUE(found_drone);
    EXPECT_TRUE(found_animal);
    EXPECT_TRUE(found_building);
}

TEST(ColorContourDetectorTest, Subsample2SmallBlobFilteredByArea) {
    // A 4×4 rect (16 full-res pixels) is below min_area=80, even after
    // subsampling scales up the count. Should be filtered.
    ColorContourDetector det;  // default subsample=2, min_area=80
    const uint32_t       W = 200, H = 200;
    auto                 img = make_solid_image(W, H, 0, 0, 0);
    fill_rect(img, W, 50, 50, 4, 4, 255, 0, 0);  // 16 px < 80

    auto result = det.detect(img.data(), W, H, 3);
    bool found  = false;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::PERSON) found = true;
    }
    EXPECT_FALSE(found);
}

TEST(ColorContourDetectorTest, kNoColorConstantIsCorrectValue) {
    EXPECT_EQ(ColorContourDetector::kNoColor, static_cast<uint8_t>(0xFF));
}

// Regression: odd frame dimensions (not multiples of subsample) must not produce
// bbox coordinates that exceed the image bounds after the scale-back from subsampled
// space to full-resolution space.
TEST(ColorContourDetectorTest, Subsample2OddDimsBboxWithinBounds) {
    ColorContourDetector det;  // default subsample=2
    ASSERT_EQ(det.subsample(), 2);

    // 301×299 — neither dimension is divisible by 2.
    const uint32_t W   = 301;
    const uint32_t H   = 299;
    auto           img = make_solid_image(W, H, 0, 0, 0);

    // Red rectangle that intentionally extends to the very right/bottom edge so the
    // last partial subsampled cell (the boundary cell) is exercised.
    fill_rect(img, W, 240, 240, 61, 59, 255, 0, 0);  // rect reaches pixel (300,298)

    auto               result = det.detect(img.data(), W, H, 3);
    const Detection2D* found  = nullptr;
    for (const auto& d : result) {
        if (d.class_id == ObjectClass::PERSON) {
            found = &d;
            break;
        }
    }
    ASSERT_NE(found, nullptr) << "Expected red blob to be detected";

    // Coordinates must be non-negative.
    EXPECT_GE(found->x, 0.0f);
    EXPECT_GE(found->y, 0.0f);
    EXPECT_GE(found->w, 0.0f);
    EXPECT_GE(found->h, 0.0f);

    // Right / bottom edge of bbox must not exceed the image dimensions.
    EXPECT_LE(found->x + found->w, static_cast<float>(W) + 1.0f);
    EXPECT_LE(found->y + found->h, static_cast<float>(H) + 1.0f);
}

// ═══════════════════════════════════════════════════════════
// Bbox ground-feature filters (Issue #345)
// ═══════════════════════════════════════════════════════════

TEST(ColorContourDetector, MinBboxHeightRejectsShortDetections) {
    // Default detector has min_bbox_height_px=15.
    // A short, wide red rectangle (100×8 px) should be rejected.
    // A tall rectangle (20×40 px) should pass.
    constexpr uint32_t   W = 640, H = 480;
    std::vector<uint8_t> img(W * H * 3, 128);
    fill_rect(img, W, 10, 10, 100, 8, 255, 0, 0);    // wide & short → rejected
    fill_rect(img, W, 300, 100, 20, 40, 255, 0, 0);  // narrow & tall → accepted

    ColorContourDetector det;
    auto                 dets = det.detect(img.data(), W, H, 3);
    // Only the tall rectangle should survive (short one rejected by height
    // filter AND aspect ratio filter)
    ASSERT_EQ(dets.size(), 1u);
    EXPECT_GE(dets[0].h, 15.0f);
}

TEST(ColorContourDetector, MaxAspectRatioRejectsFlatDetections) {
    // Default detector has max_aspect_ratio=3.0.
    // A flat red rectangle (120×20 px, aspect 6:1) should be rejected.
    // A square (30×30 px) should pass.
    constexpr uint32_t   W = 640, H = 480;
    std::vector<uint8_t> img(W * H * 3, 128);
    fill_rect(img, W, 10, 10, 120, 20, 255, 0, 0);   // flat → rejected (6:1 > 3.0)
    fill_rect(img, W, 300, 100, 30, 30, 255, 0, 0);  // square → accepted (1:1)

    ColorContourDetector det;
    auto                 dets = det.detect(img.data(), W, H, 3);
    ASSERT_EQ(dets.size(), 1u);
    // The surviving detection should be roughly square
    EXPECT_LE(dets[0].w / dets[0].h, 3.0f);
}

TEST(ColorContourDetector, TallNarrowDetectionPassesBothFilters) {
    // A tall, narrow detection (15×60 px) should pass both filters:
    // height=60 ≥ 15, aspect=15/60=0.25 ≤ 3.0.
    constexpr uint32_t   W = 640, H = 480;
    std::vector<uint8_t> img(W * H * 3, 128);
    fill_rect(img, W, 100, 50, 15, 60, 255, 0, 0);  // tall & narrow

    ColorContourDetector det;
    auto                 dets = det.detect(img.data(), W, H, 3);
    ASSERT_EQ(dets.size(), 1u);
    EXPECT_GE(dets[0].h, 15.0f);
}
