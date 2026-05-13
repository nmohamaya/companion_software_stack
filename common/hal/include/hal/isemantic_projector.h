// common/hal/include/hal/isemantic_projector.h
// HAL interface: projects 2D detections + depth into 3D voxel updates.
// Implementations: CpuSemanticProjector (pinhole back-projection).
#pragma once

#include "hal/idepth_estimator.h"
#include "hal/iinference_backend.h"
#include "hal/ivolumetric_map.h"
#include "util/result.h"

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone::hal {

struct CameraIntrinsics {
    float    fx{0.0f};
    float    fy{0.0f};
    float    cx{0.0f};
    float    cy{0.0f};
    uint32_t width{0};
    uint32_t height{0};
};

class ISemanticProjector {
public:
    virtual ~ISemanticProjector() = default;

    [[nodiscard]] virtual bool init(const CameraIntrinsics& intrinsics) = 0;

    [[nodiscard]] virtual drone::util::Result<std::vector<VoxelUpdate>, std::string> project(
        const std::vector<InferenceDetection>& detections, const DepthMap& depth,
        const Eigen::Affine3f& camera_pose) = 0;

    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::hal
