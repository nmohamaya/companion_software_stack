// common/hal/include/hal/simulated_volumetric_map.h
// CPU reference volumetric map using std::unordered_map.
// No external dependencies. Used by unit tests and quick dev cycles.
#pragma once

#include "hal/ivolumetric_map.h"
#include "util/config.h"

#include <cmath>
#include <unordered_map>

namespace drone::hal {

class SimulatedVolumetricMap : public IVolumetricMap {
public:
    explicit SimulatedVolumetricMap(float resolution_m = 0.2f) : resolution_(resolution_m) {}

    SimulatedVolumetricMap(const drone::Config& cfg, const std::string& section)
        : resolution_(cfg.get<float>(section + ".resolution_m", 0.2f)) {}

    [[nodiscard]] bool init(float resolution_m) override {
        if (resolution_m <= 0.0f) return false;
        resolution_ = resolution_m;
        map_.clear();
        initialized_ = true;
        return true;
    }

    [[nodiscard]] bool insert(const std::vector<VoxelUpdate>& updates) override {
        for (const auto& u : updates) {
            auto  key             = to_key(u.position_m);
            auto& voxel           = map_[key];
            voxel.occupancy       = u.occupancy;
            voxel.semantic_label  = u.semantic_label;
            voxel.confidence      = u.confidence;
            voxel.last_updated_ns = u.timestamp_ns;
        }
        return true;
    }

    [[nodiscard]] std::optional<VoxelData> query(const Eigen::Vector3f& position_m) const override {
        auto key = to_key(position_m);
        auto it  = map_.find(key);
        if (it == map_.end()) return std::nullopt;
        return it->second;
    }

    [[nodiscard]] size_t size() const override { return map_.size(); }

    void clear() override { map_.clear(); }

    [[nodiscard]] float resolution() const override { return resolution_; }

    [[nodiscard]] std::string name() const override { return "SimulatedVolumetricMap"; }

private:
    VoxelKey to_key(const Eigen::Vector3f& pos) const {
        return {static_cast<int32_t>(std::floor(pos.x() / resolution_)),
                static_cast<int32_t>(std::floor(pos.y() / resolution_)),
                static_cast<int32_t>(std::floor(pos.z() / resolution_))};
    }

    float                                                 resolution_{0.2f};
    bool                                                  initialized_{false};
    std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> map_;
};

}  // namespace drone::hal
