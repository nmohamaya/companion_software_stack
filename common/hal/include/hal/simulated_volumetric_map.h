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
    // PR #602 P2 review: previously the value-only constructor and the
    // Config-driven constructor both bypassed init() validation,
    // leaving `initialized_` at its default false but happily accepting
    // insert()/query() calls.  Now both constructors set
    // `initialized_ = true` after validating + storing the resolution,
    // and the resolution itself is clamped to a positive minimum so a
    // tampered config can't trigger div-by-zero in to_key().
    explicit SimulatedVolumetricMap(float resolution_m = 0.2f)
        : resolution_(std::max(kMinResolutionM, resolution_m)), initialized_(true) {}

    SimulatedVolumetricMap(const drone::Config& cfg, const std::string& section)
        : resolution_(std::max(kMinResolutionM, cfg.get<float>(section + ".resolution_m", 0.2f)))
        , initialized_(true) {}

    [[nodiscard]] bool init(float resolution_m) override {
        if (resolution_m <= 0.0f) return false;
        resolution_ = resolution_m;
        map_.clear();
        initialized_ = true;
        return true;
    }

    [[nodiscard]] bool insert(const std::vector<VoxelUpdate>& updates) override {
        // PR #602 P1 review: previously `initialized_` was set in init()
        // but never checked here — dead write.  Now we honour the
        // contract: insert() refuses when not initialised.
        if (!initialized_) return false;
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
    // PR #602 P2 review: positive minimum resolution prevents div-by-zero
    // in to_key().  Value chosen well below any realistic obstacle-grid
    // resolution (~5 cm) so the floor never bites in normal operation
    // but a tampered/typo config can't take down the perception process.
    static constexpr float kMinResolutionM = 1e-4f;

    VoxelKey to_key(const Eigen::Vector3f& pos) const {
        // resolution_ is positive by construction (clamped at ctor +
        // init() validates > 0), but assert here for the catch-all case.
        const float r = std::max(kMinResolutionM, resolution_);
        return {static_cast<int32_t>(std::floor(pos.x() / r)),
                static_cast<int32_t>(std::floor(pos.y() / r)),
                static_cast<int32_t>(std::floor(pos.z() / r))};
    }

    float                                                 resolution_{0.2f};
    bool                                                  initialized_{false};
    std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> map_;
};

}  // namespace drone::hal
