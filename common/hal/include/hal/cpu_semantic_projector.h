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

private:
    CameraIntrinsics intrinsics_{};
    bool             initialized_{false};

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
        return d;
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

        for (int gy = 0; gy < GRID; ++gy) {
            for (int gx = 0; gx < GRID; ++gx) {
                const float u = det.bbox.x + (static_cast<float>(gx) + 0.5f) * step_x;
                const float v = det.bbox.y + (static_cast<float>(gy) + 0.5f) * step_y;

                // Check mask at this sample point
                const auto mx = static_cast<uint32_t>(
                    std::clamp((u - det.bbox.x) / det.bbox.w * static_cast<float>(det.mask_width),
                               0.0f, static_cast<float>(det.mask_width - 1)));
                const auto my = static_cast<uint32_t>(
                    std::clamp((v - det.bbox.y) / det.bbox.h * static_cast<float>(det.mask_height),
                               0.0f, static_cast<float>(det.mask_height - 1)));
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
