// process2_perception/include/perception/mask_depth_projector.h
// PATH A orchestrator: SAM masks + detector classes + depth → 3D voxel updates.
// Chains MaskClassAssigner (IoU-based class assignment) with ISemanticProjector
// (pinhole back-projection) to produce VoxelUpdate[] for the occupancy grid.
#pragma once

#include "hal/isemantic_projector.h"
#include "perception/mask_class_assigner.h"
#include "perception/types.h"

#include <vector>

namespace drone::perception {

class MaskDepthProjector {
public:
    MaskDepthProjector(const MaskDepthProjector&)            = delete;
    MaskDepthProjector& operator=(const MaskDepthProjector&) = delete;
    MaskDepthProjector(MaskDepthProjector&&)                 = default;
    MaskDepthProjector& operator=(MaskDepthProjector&&)      = default;

    explicit MaskDepthProjector(hal::ISemanticProjector& projector, float iou_threshold = 0.5f)
        : projector_(projector), assigner_(iou_threshold), iou_threshold_(iou_threshold) {}

    // Full PATH A pipeline: assign detector classes to SAM masks via bbox IoU,
    // then back-project the classified masks through depth into 3D voxel updates.
    // Unmatched masks receive ObjectClass::GEOMETRIC_OBSTACLE.
    [[nodiscard]] drone::util::Result<std::vector<hal::VoxelUpdate>, std::string> project(
        const std::vector<hal::InferenceDetection>& sam_masks,
        const std::vector<hal::InferenceDetection>& detector_outputs, const hal::DepthMap& depth,
        const Eigen::Affine3f& camera_pose) const {

        if (sam_masks.empty()) {
            return drone::util::Result<std::vector<hal::VoxelUpdate>, std::string>::ok({});
        }

        auto assignments = assigner_.assign(sam_masks, detector_outputs);

        // Convert assignments to InferenceDetections with class_id set from
        // the assigned ObjectClass.  CpuSemanticProjector writes class_id into
        // VoxelUpdate::semantic_label and uses the mask for 4×4 sparse sampling.
        std::vector<hal::InferenceDetection> classified;
        classified.reserve(assignments.size());

        for (auto& a : assignments) {
            auto& det    = a.mask_detection;
            det.class_id = static_cast<int>(a.assigned_class);
            classified.push_back(std::move(det));
        }

        return projector_.project(classified, depth, camera_pose);
    }

    [[nodiscard]] float iou_threshold() const { return iou_threshold_; }

private:
    hal::ISemanticProjector& projector_;
    MaskClassAssigner        assigner_;
    float                    iou_threshold_;
};

}  // namespace drone::perception
