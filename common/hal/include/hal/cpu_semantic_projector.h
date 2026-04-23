// common/hal/include/hal/cpu_semantic_projector.h
// CPU reference implementation of ISemanticProjector.
// Back-projects detections through a pinhole model using depth map samples.
#pragma once

#include "hal/isemantic_projector.h"

#include <cmath>
#include <string>
#include <vector>

namespace drone::hal {

class CpuSemanticProjector : public ISemanticProjector {
public:
    [[nodiscard]] bool init(const CameraIntrinsics& intrinsics) override {
        if (intrinsics.width == 0 || intrinsics.height == 0) return false;
        if (intrinsics.fx <= 0.0f || intrinsics.fy <= 0.0f) return false;
        intrinsics_  = intrinsics;
        initialized_ = true;
        return true;
    }

    [[nodiscard]] drone::util::Result<std::vector<VoxelUpdate>, std::string> project(
        const std::vector<InferenceDetection>& detections, const DepthMap& depth,
        const Eigen::Affine3f& camera_pose) override {
        if (!initialized_) {
            return drone::util::Result<std::vector<VoxelUpdate>, std::string>::err(
                "CpuSemanticProjector not initialized");
        }
        if (depth.data.empty() || depth.width == 0 || depth.height == 0) {
            return drone::util::Result<std::vector<VoxelUpdate>, std::string>::err(
                "Invalid depth map");
        }

        std::vector<VoxelUpdate> updates;
        updates.reserve(detections.size());

        // Scale factors from image coords to depth map coords
        const float depth_src_w = depth.source_width > 0 ? static_cast<float>(depth.source_width)
                                                         : static_cast<float>(depth.width);
        const float depth_src_h = depth.source_height > 0 ? static_cast<float>(depth.source_height)
                                                          : static_cast<float>(depth.height);
        const float scale_x     = static_cast<float>(depth.width) / depth_src_w;
        const float scale_y     = static_cast<float>(depth.height) / depth_src_h;

        for (const auto& det : detections) {
            if (!det.mask.empty() && det.mask_width > 0 && det.mask_height > 0) {
                // Sparse grid sampling within mask
                project_masked(det, depth, camera_pose, scale_x, scale_y, updates);
            } else {
                // Single sample at bbox centre
                project_bbox_centre(det, depth, camera_pose, scale_x, scale_y, updates);
            }
        }

        return drone::util::Result<std::vector<VoxelUpdate>, std::string>::ok(std::move(updates));
    }

    [[nodiscard]] std::string name() const override { return "CpuSemanticProjector"; }

    /// Texture gate (Issue #616) — reject depth samples in low-gradient regions
    /// of the depth map.  Flat depth regions correspond to DA V2's weakest
    /// failure mode (cube faces, sky, textureless walls) where the network
    /// has no features to latch onto.  Threshold is depth-gradient magnitude
    /// (metres per pixel).  0.0 = disabled (backward-compatible default).
    /// Operators tune this against scenario 33's `check_voxel_on_target.py`
    /// output; 0.05-0.2 m/px is a reasonable starting range for DA V2 at
    /// 518x518 input.
    void set_texture_gate_threshold(float threshold) {
        texture_gate_threshold_ = std::max(0.0f, threshold);
    }

    [[nodiscard]] float texture_gate_threshold() const { return texture_gate_threshold_; }

private:
    CameraIntrinsics intrinsics_{};
    bool             initialized_{false};
    float            texture_gate_threshold_{0.0f};

    // Back-project pixel (u,v) at depth Z to a world-frame 3D point
    [[nodiscard]] Eigen::Vector3f backproject(float u, float v, float z,
                                              const Eigen::Affine3f& camera_pose) const {
        const float     x_cam = (u - intrinsics_.cx) * z / intrinsics_.fx;
        const float     y_cam = (v - intrinsics_.cy) * z / intrinsics_.fy;
        Eigen::Vector3f pt_cam(x_cam, y_cam, z);
        return camera_pose * pt_cam;
    }

    [[nodiscard]] float sample_depth(const DepthMap& depth, float u, float v, float scale_x,
                                     float scale_y) const {
        const auto dx = static_cast<uint32_t>(
            std::clamp(u * scale_x, 0.0f, static_cast<float>(depth.width - 1)));
        const auto dy = static_cast<uint32_t>(
            std::clamp(v * scale_y, 0.0f, static_cast<float>(depth.height - 1)));
        const float d = depth.data[dy * depth.width + dx] * depth.scale;
        if (!std::isfinite(d) || d <= 0.0f) return 0.0f;
        // Reject samples beyond kMaxObstacleDepth — distant horizon / sky /
        // scenery contours are not navigation-relevant and would otherwise
        // flood the downstream occupancy grid with ghost voxels.  20 m is a
        // conservative drone-scale cutoff; real obstacles the planner needs
        // to route around are well within this range at scenario cruise
        // speeds.  (#608 E5.INT — discovered when EdgeContourSAM masks
        // picked up every image edge and back-projected them into voxels at
        // 50-100 m depth, filling the grid far past `max_static_cells`.)
        constexpr float kMaxObstacleDepth = 20.0f;
        if (d > kMaxObstacleDepth) return 0.0f;
        // Texture gate (Issue #616) — reject samples where the local depth
        // gradient is below threshold.  Cube faces, sky, untextured walls
        // produce nearly-flat depth output from DA V2 (the network has no
        // features to latch onto); voxelising them pollutes the grid.
        // Computing the gradient on the *depth* map (not RGB) is a proxy
        // for DA V2 confidence — if the model couldn't see variation, we
        // shouldn't trust the metres it reports.  Disabled by default
        // (threshold=0 short-circuits at the top of the check).
        if (texture_gate_threshold_ > 0.0f &&
            depth_gradient_magnitude(depth, dx, dy) < texture_gate_threshold_) {
            return 0.0f;
        }
        return d;
    }

    /// Sobel-style gradient magnitude on the depth map at pixel (dx, dy).
    /// Returns the magnitude in metres-per-pixel (depth units).  Safely
    /// clamps the 3x3 stencil at the map boundary.  Used by the texture
    /// gate above.
    [[nodiscard]] float depth_gradient_magnitude(const DepthMap& depth, uint32_t dx,
                                                 uint32_t dy) const {
        if (depth.width < 3 || depth.height < 3) return 0.0f;
        const uint32_t x1 = dx == 0 ? 0 : dx - 1;
        const uint32_t x2 = std::min<uint32_t>(dx + 1, depth.width - 1);
        const uint32_t y1 = dy == 0 ? 0 : dy - 1;
        const uint32_t y2 = std::min<uint32_t>(dy + 1, depth.height - 1);

        const auto at = [&](uint32_t x, uint32_t y) {
            return depth.data[y * depth.width + x] * depth.scale;
        };

        // 3x3 Sobel stencil — standard kernels for Gx and Gy.  Non-finite
        // samples anywhere in the stencil poison the whole gradient (return
        // 0 so the texture gate fails closed — a safe default).
        const float p00 = at(x1, y1), p01 = at(dx, y1), p02 = at(x2, y1);
        const float p10 = at(x1, dy), p12 = at(x2, dy);
        const float p20 = at(x1, y2), p21 = at(dx, y2), p22 = at(x2, y2);
        if (!std::isfinite(p00) || !std::isfinite(p01) || !std::isfinite(p02) ||
            !std::isfinite(p10) || !std::isfinite(p12) || !std::isfinite(p20) ||
            !std::isfinite(p21) || !std::isfinite(p22)) {
            return 0.0f;
        }
        const float gx = (p02 + 2.0f * p12 + p22) - (p00 + 2.0f * p10 + p20);
        const float gy = (p20 + 2.0f * p21 + p22) - (p00 + 2.0f * p01 + p02);
        return std::sqrt(gx * gx + gy * gy);
    }

    void project_bbox_centre(const InferenceDetection& det, const DepthMap& depth,
                             const Eigen::Affine3f& camera_pose, float scale_x, float scale_y,
                             std::vector<VoxelUpdate>& updates) const {
        const float u = det.bbox.x + det.bbox.w * 0.5f;
        const float v = det.bbox.y + det.bbox.h * 0.5f;
        const float z = sample_depth(depth, u, v, scale_x, scale_y);
        if (z <= 0.0f) return;

        VoxelUpdate vu;
        vu.position_m     = backproject(u, v, z, camera_pose);
        vu.semantic_label = static_cast<uint8_t>(std::max(det.class_id, 0));
        vu.confidence     = det.confidence;
        vu.occupancy      = 1.0f;
        updates.push_back(vu);
    }

    void project_masked(const InferenceDetection& det, const DepthMap& depth,
                        const Eigen::Affine3f& camera_pose, float scale_x, float scale_y,
                        std::vector<VoxelUpdate>& updates) const {
        // Sparse 4x4 grid within the mask bounding box
        constexpr int GRID   = 4;
        const float   step_x = det.bbox.w / static_cast<float>(GRID);
        const float   step_y = det.bbox.h / static_cast<float>(GRID);

        // Backends in the codebase use two mask conventions:
        //   1. bbox-local      — mask_width ≈ bbox.w (e.g. YOLOv8-seg tiled masks)
        //   2. full-image      — mask_width ≫ bbox.w, pixel indexed by (u, v)
        //                        directly (EdgeContourSAMBackend, SimulatedSAMBackend).
        // Auto-detect from geometry to accept both.  Issue #608 E5.INT: the
        // original code assumed only convention (1), silently filtered almost
        // every sample when convention (2) backends were used (~99% probe
        // rejection), producing 1-2 voxels/frame instead of ~10-50.
        const bool mask_is_full_image = static_cast<float>(det.mask_width) >= det.bbox.w * 1.5f &&
                                        static_cast<float>(det.mask_height) >= det.bbox.h * 1.5f;

        for (int gy = 0; gy < GRID; ++gy) {
            for (int gx = 0; gx < GRID; ++gx) {
                const float u = det.bbox.x + (static_cast<float>(gx) + 0.5f) * step_x;
                const float v = det.bbox.y + (static_cast<float>(gy) + 0.5f) * step_y;

                uint32_t mx = 0;
                uint32_t my = 0;
                if (mask_is_full_image) {
                    mx = static_cast<uint32_t>(
                        std::clamp(u, 0.0f, static_cast<float>(det.mask_width - 1)));
                    my = static_cast<uint32_t>(
                        std::clamp(v, 0.0f, static_cast<float>(det.mask_height - 1)));
                } else {
                    mx = static_cast<uint32_t>(std::clamp(
                        (u - det.bbox.x) / det.bbox.w * static_cast<float>(det.mask_width), 0.0f,
                        static_cast<float>(det.mask_width - 1)));
                    my = static_cast<uint32_t>(std::clamp(
                        (v - det.bbox.y) / det.bbox.h * static_cast<float>(det.mask_height), 0.0f,
                        static_cast<float>(det.mask_height - 1)));
                }
                if (det.mask[my * det.mask_width + mx] < 128) continue;

                const float z = sample_depth(depth, u, v, scale_x, scale_y);
                if (z <= 0.0f) continue;

                VoxelUpdate vu;
                vu.position_m     = backproject(u, v, z, camera_pose);
                vu.semantic_label = static_cast<uint8_t>(std::max(det.class_id, 0));
                vu.confidence     = det.confidence;
                vu.occupancy      = 1.0f;
                updates.push_back(vu);
            }
        }
    }
};

}  // namespace drone::hal
