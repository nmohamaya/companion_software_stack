// common/hal/include/hal/ivolumetric_map.h
// HAL interface: Volumetric map abstraction for 3D occupancy + semantic labels.
// Implementations: SimulatedVolumetricMap (built-in, hash-map CPU).
// Future: UFOMap, nvblox (GPU), Vulkan-compute backends.
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace drone::hal {

struct VoxelKey {
    int32_t x{0};
    int32_t y{0};
    int32_t z{0};

    bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const {
        auto h1 = std::hash<int32_t>{}(k.x);
        auto h2 = std::hash<int32_t>{}(k.y);
        auto h3 = std::hash<int32_t>{}(k.z);
        return h1 ^ (h2 << 11) ^ (h3 << 22);
    }
};

struct VoxelData {
    float    occupancy{0.0f};
    uint8_t  semantic_label{0};
    float    confidence{0.0f};
    uint64_t last_updated_ns{0};
};

struct VoxelUpdate {
    Eigen::Vector3f position_m{Eigen::Vector3f::Zero()};
    float           occupancy{1.0f};
    uint8_t         semantic_label{0};
    float           confidence{0.5f};
    uint64_t        timestamp_ns{0};
};

/// Abstract volumetric map interface.
/// Stores 3D occupancy with optional semantic labels per voxel.
class IVolumetricMap {
public:
    virtual ~IVolumetricMap() = default;

    /// Initialize the map with the given voxel resolution (meters per voxel side).
    [[nodiscard]] virtual bool init(float resolution_m) = 0;

    /// Insert voxel updates into the map.
    [[nodiscard]] virtual bool insert(const std::vector<VoxelUpdate>& updates) = 0;

    /// Query the voxel at a world-frame position. Returns nullopt if no data.
    [[nodiscard]] virtual std::optional<VoxelData> query(
        const Eigen::Vector3f& position_m) const = 0;

    /// Number of occupied voxels in the map.
    [[nodiscard]] virtual size_t size() const = 0;

    /// Clear all voxels.
    virtual void clear() = 0;

    /// Voxel resolution in meters.
    [[nodiscard]] virtual float resolution() const = 0;

    /// Human-readable backend name.
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::hal
