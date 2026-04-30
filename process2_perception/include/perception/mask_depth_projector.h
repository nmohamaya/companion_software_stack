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

    /// Issue #645 — SAM mask size filter applied BEFORE back-projection.
    /// Diagnostic on run 2026-04-30_174147 showed SAM produces 41-47
    /// masks per 1280×720 frame for scenes with ~7 spawned obstacles.
    /// The bulk are tiny texture features (60-80 px) or huge background
    /// regions (sky, ground) — both produce ghost voxels.  Drop masks
    /// with bbox area outside `[min_area_px, max_area_px]` before they
    /// hit the back-projection stage.  Real obstacles in scenario 33
    /// at typical detection distance produce masks of 1-3 k px (small
    /// pillar) up to ~40 k px (close wall); 100-200 k px masks are
    /// typically sky.  `0` disables either bound.  Default: bypass.
    struct MaskSizeFilter {
        float min_area_px{0.0f};
        float max_area_px{0.0f};
    };

    MaskDepthProjector(const MaskDepthProjector&)                = delete;
    MaskDepthProjector& operator=(const MaskDepthProjector&)     = delete;
    MaskDepthProjector(MaskDepthProjector&&) noexcept            = default;
    MaskDepthProjector& operator=(MaskDepthProjector&&) noexcept = default;

    // projector must outlive this object (non-owning reference).
    explicit MaskDepthProjector(hal::ISemanticProjector& projector, float iou_threshold = 0.5f,
                                AltitudeFilter altitude  = AltitudeFilter{0.0f, 0.0f},
                                MaskSizeFilter mask_size = MaskSizeFilter{0.0f, 0.0f})
        : projector_(projector)
        , assigner_(std::clamp(iou_threshold, 0.01f, 1.0f))
        , altitude_(altitude)
        , mask_size_(mask_size) {
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
        if (mask_size_.min_area_px > 0.0f && mask_size_.max_area_px > 0.0f &&
            mask_size_.min_area_px >= mask_size_.max_area_px) {
            DRONE_LOG_WARN("[MaskDepthProjector] mask size filter min_area_px={:.0f} >= "
                           "max_area_px={:.0f}; disabling.",
                           mask_size_.min_area_px, mask_size_.max_area_px);
            mask_size_ = MaskSizeFilter{0.0f, 0.0f};
        }
        if (mask_size_.min_area_px > 0.0f || mask_size_.max_area_px > 0.0f) {
            DRONE_LOG_INFO("[MaskDepthProjector] SAM mask size filter active: "
                           "min_area_px={:.0f} max_area_px={:.0f}",
                           mask_size_.min_area_px, mask_size_.max_area_px);
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

        // Issue #645 — SAM mask size filter (#659).  Drop masks whose bbox
        // area falls outside the configured band BEFORE back-projection.
        // Tiny masks are texture features that produce scattered ghost
        // voxels; huge masks are sky / ground that produce ghost walls.
        const std::vector<hal::InferenceDetection>* sam_masks_ptr = &sam_masks;
        std::vector<hal::InferenceDetection>        filtered_masks;
        if (mask_size_.min_area_px > 0.0f || mask_size_.max_area_px > 0.0f) {
            filtered_masks.reserve(sam_masks.size());
            for (const auto& m : sam_masks) {
                const float area = m.bbox.w * m.bbox.h;
                if (mask_size_.min_area_px > 0.0f && area < mask_size_.min_area_px) {
                    ++mask_size_dropped_min_;
                    continue;
                }
                if (mask_size_.max_area_px > 0.0f && area > mask_size_.max_area_px) {
                    ++mask_size_dropped_max_;
                    continue;
                }
                filtered_masks.push_back(m);
            }
            sam_masks_ptr = &filtered_masks;
            if (filtered_masks.empty()) {
                return drone::util::Result<std::vector<hal::VoxelUpdate>, std::string>::ok({});
            }
        }

        auto assignments = assigner_.assign(*sam_masks_ptr, detector_outputs);

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

    /// Issue #645 — counts of SAM masks dropped by the mask-size filter.
    /// `min` counter = below `min_area_px` (texture features).
    /// `max` counter = above `max_area_px` (sky / ground regions).
    [[nodiscard]] uint64_t mask_size_dropped_min_count() const noexcept {
        return mask_size_dropped_min_;
    }
    [[nodiscard]] uint64_t mask_size_dropped_max_count() const noexcept {
        return mask_size_dropped_max_;
    }

private:
    hal::ISemanticProjector& projector_;
    MaskClassAssigner        assigner_;
    AltitudeFilter           altitude_;
    MaskSizeFilter           mask_size_;
    mutable uint64_t         altitude_dropped_{0};
    mutable uint64_t         mask_size_dropped_min_{0};
    mutable uint64_t         mask_size_dropped_max_{0};
};

}  // namespace drone::perception
