// process2_perception/include/perception/mask_depth_projector.h
// PATH A orchestrator: SAM masks + detector classes + depth → 3D voxel updates.
// Chains MaskClassAssigner (IoU-based class assignment) with ISemanticProjector
// (pinhole back-projection) to produce VoxelUpdate[] for the occupancy grid.
// See docs/design/perception_design.md for PATH A architecture.
#pragma once

#include "hal/isemantic_projector.h"
#include "perception/mask_class_assigner.h"
#include "perception/types.h"
#include "util/ilogger.h"

#include <algorithm>
#include <vector>

namespace drone::perception {

class MaskDepthProjector {
public:
    /// Issue #645 — altitude filter applied to back-projected voxels.
    /// Diagnostic on run 2026-04-30_174147 showed 22% of voxels above 6 m
    /// (sky misprojections from SAM masks of distant background) and 10%
    /// below 0.3 m (ground patches projected with bias).  Filtering at
    /// the projector boundary stops these reaching the publish stage.
    /// Both thresholds are world-frame (NED-derived from SLAM pose).
    /// `max_z_m == 0` disables the upper bound; `min_z_m == 0` disables
    /// the lower bound.  Default: bypass (legacy behaviour).
    struct AltitudeFilter {
        float min_z_m{0.0f};
        float max_z_m{0.0f};
    };

    MaskDepthProjector(const MaskDepthProjector&)                = delete;
    MaskDepthProjector& operator=(const MaskDepthProjector&)     = delete;
    MaskDepthProjector(MaskDepthProjector&&) noexcept            = default;
    MaskDepthProjector& operator=(MaskDepthProjector&&) noexcept = default;

    // projector must outlive this object (non-owning reference).
    explicit MaskDepthProjector(hal::ISemanticProjector& projector, float iou_threshold = 0.5f,
                                AltitudeFilter altitude = AltitudeFilter{0.0f, 0.0f})
        : projector_(projector)
        , assigner_(std::clamp(iou_threshold, 0.01f, 1.0f))
        , altitude_(altitude) {
        if (iou_threshold < 0.01f || iou_threshold > 1.0f) {
            DRONE_LOG_WARN("[MaskDepthProjector] iou_threshold {:.3f} clamped to [{:.2f}, {:.2f}]",
                           iou_threshold, 0.01f, 1.0f);
        }
        if (altitude_.min_z_m > 0.0f && altitude_.max_z_m > 0.0f &&
            altitude_.min_z_m >= altitude_.max_z_m) {
            DRONE_LOG_WARN("[MaskDepthProjector] altitude filter min_z_m={:.2f} >= "
                           "max_z_m={:.2f}; disabling.",
                           altitude_.min_z_m, altitude_.max_z_m);
            altitude_ = AltitudeFilter{};
        }
        if (altitude_.min_z_m > 0.0f || altitude_.max_z_m > 0.0f) {
            DRONE_LOG_INFO("[MaskDepthProjector] altitude filter active: "
                           "min_z_m={:.2f} max_z_m={:.2f}",
                           altitude_.min_z_m, altitude_.max_z_m);
        }
    }

    // Full PATH A pipeline: assign detector classes to SAM masks via bbox IoU,
    // then back-project the classified masks through depth into 3D voxel updates.
    // Unmatched masks receive ObjectClass::GEOMETRIC_OBSTACLE (via MaskClassAssigner default).
    [[nodiscard]] drone::util::Result<std::vector<hal::VoxelUpdate>, std::string> project(
        const std::vector<hal::InferenceDetection>& sam_masks,
        const std::vector<hal::InferenceDetection>& detector_outputs, const hal::DepthMap& depth,
        const Eigen::Affine3f& camera_pose) const {

        if (sam_masks.empty()) {
            return drone::util::Result<std::vector<hal::VoxelUpdate>, std::string>::ok({});
        }

        auto assignments = assigner_.assign(sam_masks, detector_outputs);

        // Repurpose class_id from COCO-id to ObjectClass ordinal so
        // CpuSemanticProjector writes it into VoxelUpdate::semantic_label.
        // Mask data is preserved for 4×4 sparse grid sampling.
        std::vector<hal::InferenceDetection> classified;
        classified.reserve(assignments.size());

        for (auto& a : assignments) {
            auto& det    = a.mask_detection;
            det.class_id = static_cast<int>(a.assigned_class);
            classified.push_back(std::move(det));
        }

        auto result = projector_.project(classified, depth, camera_pose);
        if (!result.is_ok()) {
            return result;
        }

        // Issue #645 — altitude filter (#658).  Drop voxels above max_z (sky
        // misprojections) and below min_z (ground patches with depth bias).
        // World-frame Z; thresholds 0 = disabled.
        if (altitude_.min_z_m > 0.0f || altitude_.max_z_m > 0.0f) {
            auto&        voxels = result.value();
            const size_t before = voxels.size();
            voxels.erase(std::remove_if(voxels.begin(), voxels.end(),
                                        [&](const hal::VoxelUpdate& v) {
                                            const float z = v.position_m.z();
                                            if (altitude_.max_z_m > 0.0f && z > altitude_.max_z_m) {
                                                return true;
                                            }
                                            if (altitude_.min_z_m > 0.0f && z < altitude_.min_z_m) {
                                                return true;
                                            }
                                            return false;
                                        }),
                         voxels.end());
            altitude_dropped_ += (before - voxels.size());
        }

        return result;
    }

    [[nodiscard]] float iou_threshold() const { return assigner_.iou_threshold(); }

    /// Issue #645 — number of voxels dropped by the altitude filter since
    /// construction.  Surfaced for diagnostic logging.
    [[nodiscard]] uint64_t altitude_dropped_count() const noexcept { return altitude_dropped_; }

private:
    hal::ISemanticProjector& projector_;
    MaskClassAssigner        assigner_;
    AltitudeFilter           altitude_;
    mutable uint64_t         altitude_dropped_{0};
};

}  // namespace drone::perception
