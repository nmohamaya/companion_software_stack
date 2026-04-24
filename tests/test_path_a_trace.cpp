// tests/test_path_a_trace.cpp
// Tests for the PathATrace diagnostic JSONL writer (Issue #612, PR #614).
//
// Covers:
//   - disabled path is a no-op (no file created, no allocations, no crashes)
//   - enabled writes one JSONL line per record_batch() call
//   - path confinement rejects absolute paths and ../-escapes
//   - R_body_from_cam extrinsic constants are self-consistent (Fix #55)

#include "util/path_a_trace.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace {

std::filesystem::path temp_path(const std::string& name) {
    auto p = std::filesystem::temp_directory_path() /
             ("path_a_trace_test_" + std::to_string(::getpid()) + "_" + name);
    std::filesystem::remove_all(p);
    return p;
}

void fill_batch(std::vector<drone::hal::InferenceDetection>& dets,
                std::vector<drone::hal::InferenceDetection>& masks,
                std::vector<drone::hal::VoxelUpdate>&        voxels) {
    drone::hal::InferenceDetection d{};
    d.bbox.x     = 10;
    d.bbox.y     = 20;
    d.bbox.w     = 30;
    d.bbox.h     = 40;
    d.class_id   = 1;
    d.confidence = 0.9f;
    dets.push_back(d);
    masks.push_back(d);

    drone::hal::VoxelUpdate v{};
    v.position_m     = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    v.occupancy      = 0.8f;
    v.confidence     = 0.7f;
    v.semantic_label = 42;
    voxels.push_back(v);
}

}  // namespace

TEST(PathATraceTest, DisabledPathIsNoop) {
    const auto p = temp_path("disabled");
    // Even with an invalid absolute path, `enabled=false` should short-circuit.
    drone::util::PathATrace trace(false, "/nonexistent/path/that/would/fail.jsonl");
    EXPECT_FALSE(trace.enabled());
    // No file should have been created at the path passed in.
    EXPECT_FALSE(std::filesystem::exists("/nonexistent/path/that/would/fail.jsonl"));
    // Calling record_batch on a disabled trace must not crash.
    std::vector<drone::hal::InferenceDetection> dets, masks;
    std::vector<drone::hal::VoxelUpdate>        voxels;
    fill_batch(dets, masks, voxels);
    drone::ipc::Pose pose{};
    trace.record_batch(/*t_ns=*/1, /*seq=*/1, pose, dets, masks, voxels);
    SUCCEED();
}

TEST(PathATraceTest, EnabledWritesOneJsonlLinePerBatch) {
    const auto        p   = temp_path("write.jsonl");
    const std::string rel = std::string("drone_logs/") +
                            p.filename().string();  // relative → confined
    const auto abs = std::filesystem::current_path() / rel;
    std::filesystem::create_directories(abs.parent_path());
    std::filesystem::remove(abs);

    {
        drone::util::PathATrace trace(true, rel);
        ASSERT_TRUE(trace.enabled());

        std::vector<drone::hal::InferenceDetection> dets, masks;
        std::vector<drone::hal::VoxelUpdate>        voxels;
        fill_batch(dets, masks, voxels);
        drone::ipc::Pose pose{};
        pose.translation[0] = 1.0;
        pose.translation[1] = 2.0;
        pose.translation[2] = 3.0;
        pose.quaternion[0]  = 1.0;  // identity

        trace.record_batch(100, 7, pose, dets, masks, voxels);
        trace.record_batch(200, 8, pose, dets, masks, voxels);
    }  // dtor flushes + closes

    std::ifstream in(abs);
    ASSERT_TRUE(in.is_open()) << "expected trace file at " << abs;
    std::string              line;
    std::vector<std::string> lines;
    while (std::getline(in, line)) lines.push_back(line);
    ASSERT_EQ(lines.size(), 2u);
    // Each line must be valid JSON with the documented top-level keys.
    for (const auto& ln : lines) {
        auto j = nlohmann::json::parse(ln);
        EXPECT_TRUE(j.contains("t_ns"));
        EXPECT_TRUE(j.contains("seq"));
        EXPECT_TRUE(j.contains("pose_t"));
        EXPECT_TRUE(j.contains("pose_q"));
        EXPECT_TRUE(j.contains("voxels"));
        EXPECT_EQ(j["voxels"].size(), 1u);
    }
    std::filesystem::remove(abs);
}

TEST(PathATraceTest, PathConfinementRejectsAbsoluteWithoutDroneLogs) {
    // Absolute path with no `drone_logs` segment must be rejected — force
    // the writer inert.
    drone::util::PathATrace trace(true, "/tmp/absolute_escape.jsonl");
    EXPECT_FALSE(trace.enabled());
    EXPECT_FALSE(std::filesystem::exists("/tmp/absolute_escape.jsonl"));
}

TEST(PathATraceTest, PathConfinementAcceptsAbsoluteUnderDroneLogs) {
    // Scenario runners rewrite trace_path to an absolute path under the
    // run's drone_logs/ subtree.  Verify that such paths are accepted —
    // this was broken before the confinement relaxation (2026-04-24)
    // when every scenario-33 run was rejecting its own trace file.
    auto abs_path = std::filesystem::temp_directory_path() /
                    ("drone_logs_test_" + std::to_string(::getpid())) / "drone_logs" /
                    "path_a_voxel_trace.jsonl";
    std::filesystem::create_directories(abs_path.parent_path());
    drone::util::PathATrace trace(true, abs_path.string());
    EXPECT_TRUE(trace.enabled()) << "absolute path with drone_logs segment should be accepted: "
                                 << abs_path;
    std::filesystem::remove_all(abs_path.parent_path().parent_path());
}

TEST(PathATraceTest, PathConfinementRejectsDotDotEscape) {
    // Even a relative path that lexically normalises to escape the project
    // root must be rejected.  `../../etc/foo.jsonl` normalises to start
    // with `..`.
    drone::util::PathATrace trace(true, "../../etc/evil_trace.jsonl");
    EXPECT_FALSE(trace.enabled());
}

TEST(PathATraceTest, PathConfinementStaticHelper) {
    // Exhaust the is_path_confined() truth table in one place.
    using drone::util::PathATrace;
    EXPECT_TRUE(PathATrace::is_path_confined("drone_logs/foo.jsonl"));
    EXPECT_TRUE(PathATrace::is_path_confined("foo.jsonl"));
    EXPECT_TRUE(PathATrace::is_path_confined("build/test/a.jsonl"));
    EXPECT_FALSE(PathATrace::is_path_confined(""));
    EXPECT_FALSE(PathATrace::is_path_confined("/etc/passwd"));
    EXPECT_FALSE(PathATrace::is_path_confined("/tmp/something.jsonl"));
    EXPECT_FALSE(PathATrace::is_path_confined("../foo.jsonl"));
    EXPECT_FALSE(PathATrace::is_path_confined("../../etc/foo"));
    // Paths that contain "..", but normalise to staying within the tree,
    // are fine (e.g. "foo/../bar.jsonl" normalises to "bar.jsonl").
    EXPECT_TRUE(PathATrace::is_path_confined("foo/../bar.jsonl"));
}

// ── R_body_from_cam frame-convention invariants (Fix #55) ───────────────
// The camera→body rotation applied in mask_projection_thread must map:
//   cam Z-axis (forward)   → body X-axis (forward)
//   cam X-axis (right)     → body -Y-axis (right-of-body is negative Y)
//   cam Y-axis (down)      → body -Z-axis (up-of-body is negative Y-cam)
// This test locks in those invariants so a future refactor that flips a
// sign (the exact class of bug Fix #55 addressed) fails loudly.
TEST(PathATraceTest, RBodyFromCamFrameConventionInvariants) {
    // Rebuild the constant matching process2_perception/src/main.cpp.
    const Eigen::Matrix3f R = (Eigen::Matrix3f() << 0, 0, 1,  //
                               -1, 0, 0,                      //
                               0, -1, 0                       //
                               )
                                  .finished();
    // Unit vectors in the camera optical frame.
    const Eigen::Vector3f cam_x(1, 0, 0);  // right
    const Eigen::Vector3f cam_y(0, 1, 0);  // down
    const Eigen::Vector3f cam_z(0, 0, 1);  // forward
    // After applying R, they must land on the expected body axes.
    EXPECT_TRUE((R * cam_z).isApprox(Eigen::Vector3f(1, 0, 0), 1e-6f))
        << "cam +Z (forward) must map to body +X (forward)";
    EXPECT_TRUE((R * cam_x).isApprox(Eigen::Vector3f(0, -1, 0), 1e-6f))
        << "cam +X (right) must map to body -Y (body Y is left-positive)";
    EXPECT_TRUE((R * cam_y).isApprox(Eigen::Vector3f(0, 0, -1), 1e-6f))
        << "cam +Y (down) must map to body -Z (body Z is up-positive)";
    // Should be a proper rotation — det(R) = +1, R^T R = I.
    EXPECT_NEAR(R.determinant(), 1.0f, 1e-6f);
    EXPECT_TRUE((R.transpose() * R).isApprox(Eigen::Matrix3f::Identity(), 1e-6f));
}
