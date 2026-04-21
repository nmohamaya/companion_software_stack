// tests/benchmark/cosys_gt_emitter.cpp
//
// Cosys-AirSim backend for IGroundTruthEmitter (Issue #594 PR 1).
// Gated by HAVE_COSYS_AIRSIM — compiles to an empty TU without the SDK.
//
// Uses AirSim's built-in detection API:
//   simAddDetectionFilterMeshName(camera, image_type, pattern)  — register a
//                                                                 class filter
//   simSetDetectionFilterRadius(camera, image_type, radius_cm)  — visibility
//   simGetDetections(camera, image_type)                         — per-frame pull
//
// AirSim returns `vector<DetectionInfo>` with name + 2D bbox + 3D bbox +
// relative_pose already computed — we only need class-map translation and
// stable identity assignment.

#ifdef HAVE_COSYS_AIRSIM

#include "benchmark/gt_emitter.h"
#include "hal/cosys_rpc_client.h"
#include "hal/hal_factory.h"  // detail::get_shared_cosys_client
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace drone::benchmark {

namespace {

// Cosys camera we attach detection filters to. Matches the key used by
// CosysCameraBackend (config/cosys_airsim.json → camera_name).
constexpr const char*                              kDefaultCameraName       = "mission_cam";
constexpr const char*                              kDefaultVehicleName      = "Drone0";
constexpr float                                    kDefaultDetectionRadiusM = 100.0F;  // metres
constexpr msr::airlib::ImageCaptureBase::ImageType kImageType =
    msr::airlib::ImageCaptureBase::ImageType::Scene;

// Hash a simulator object name into a stable uint32 object id. Same input →
// same output across processes / runs; different names → different ids with
// overwhelming probability. The GT consumer only needs stability across
// frames of a single run, which this easily provides.
//
// This is a GT-side object identifier (what the simulator KNOWS), distinct
// from perception-tracker track ids (ByteTrack/UKF) which are what the
// tracker GUESSES. The benchmark compares the two.
[[nodiscard]] uint32_t stable_object_id_from_name(std::string_view name) {
    const std::uint64_t h = std::hash<std::string_view>{}(name);
    // Fold 64 → 32 bits; lower 32 are fine because we don't need the id to
    // round-trip back to the name.
    return static_cast<std::uint32_t>(h) ^ static_cast<std::uint32_t>(h >> 32U);
}

class CosysGtEmitter : public IGroundTruthEmitter {
public:
    CosysGtEmitter(std::shared_ptr<drone::hal::CosysRpcClient> rpc, std::string camera_name,
                   std::string vehicle_name, float detection_radius_m, GtClassMap class_map)
        : rpc_(std::move(rpc))
        , camera_name_(std::move(camera_name))
        , vehicle_name_(std::move(vehicle_name))
        , detection_radius_m_(detection_radius_m)
        , class_map_(std::move(class_map)) {}

    /// Register AirSim detection filters for every pattern in the class map.
    /// Must be called after the RPC client connects. Returns true if the RPC
    /// was reachable (individual filter-add failures are logged at WARN but
    /// do not fail the whole operation — a partial filter set is still useful).
    [[nodiscard]] bool register_filters(const std::vector<std::string>& patterns) {
        if (!rpc_ || !rpc_->is_connected() || class_map_.empty()) {
            return false;
        }
        return rpc_->with_client([&](auto& rpc) {
            // Clear any pre-existing filter so repeated runs don't accumulate.
            try {
                rpc.simClearDetectionMeshNames(camera_name_, kImageType, vehicle_name_);
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[GT-Cosys] simClearDetectionMeshNames failed: {}", e.what());
            }
            try {
                // AirSim takes radius in centimetres.
                rpc.simSetDetectionFilterRadius(camera_name_, kImageType,
                                                detection_radius_m_ * 100.0F, vehicle_name_);
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[GT-Cosys] simSetDetectionFilterRadius failed: {}", e.what());
            }
            for (const auto& pattern : patterns) {
                try {
                    // AirSim accepts trailing-* wildcards in mesh_name (same
                    // syntax GtClassMap uses), so we register patterns verbatim.
                    rpc.simAddDetectionFilterMeshName(camera_name_, kImageType, pattern,
                                                      vehicle_name_);
                } catch (const std::exception& e) {
                    DRONE_LOG_WARN("[GT-Cosys] simAddDetectionFilterMeshName({}) failed: {}",
                                   pattern, e.what());
                }
            }
        });
    }

    std::optional<FrameGroundTruth> emit(uint64_t timestamp_ns, uint64_t frame_sequence) override {
        if (!rpc_ || !rpc_->is_connected() || class_map_.empty()) {
            return std::nullopt;
        }

        std::vector<msr::airlib::DetectionInfo> detections;
        msr::airlib::Pose                       vehicle_pose{};
        const bool                              ok = rpc_->with_client([&](auto& rpc) {
            try {
                detections = rpc.simGetDetections(camera_name_, kImageType, vehicle_name_);
                vehicle_pose = rpc.simGetVehiclePose(vehicle_name_);
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[GT-Cosys] RPC error during emit: {}", e.what());
                detections.clear();
            }
        });
        if (!ok) {
            return std::nullopt;
        }

        FrameGroundTruth frame;
        frame.timestamp_ns   = timestamp_ns;
        frame.frame_sequence = frame_sequence;
        frame.camera_pose    = to_camera_pose(vehicle_pose);
        frame.objects.reserve(detections.size());

        for (const auto& d : detections) {
            const auto cls = class_map_.lookup(d.name);
            if (!cls) {
                continue;  // Not in our tracked class set — drop.
            }

            GtDetection g;
            g.class_id     = cls->class_id;
            g.class_name   = cls->class_name;
            g.gt_object_id = stable_object_id_from_name(d.name);
            g.bbox         = to_bbox(d.box2D);
            // AirSim's detection API filters to visible objects already, so
            // everything we see is at least partly visible. A proper occlusion
            // ratio requires segmentation-mask accounting — tracked as a
            // follow-up (see IMPROVEMENTS.md entry for GT occlusion score).
            g.occlusion  = 0.0F;
            g.distance_m = distance_to_object(d.relative_pose);

            frame.objects.push_back(std::move(g));
        }
        return frame;
    }

    [[nodiscard]] std::string_view backend_name() const noexcept override { return "cosys"; }

private:
    static GtCameraPose to_camera_pose(const msr::airlib::Pose& p) {
        GtCameraPose out;
        out.translation[0] = p.position.x();
        out.translation[1] = p.position.y();
        out.translation[2] = p.position.z();
        out.quaternion[0]  = p.orientation.w();
        out.quaternion[1]  = p.orientation.x();
        out.quaternion[2]  = p.orientation.y();
        out.quaternion[3]  = p.orientation.z();
        return out;
    }

    static BBox2D to_bbox(const msr::airlib::Box2D& b) {
        BBox2D out;
        out.x = b.min.x();
        out.y = b.min.y();
        out.w = b.max.x() - b.min.x();
        out.h = b.max.y() - b.min.y();
        return out;
    }

    static float distance_to_object(const msr::airlib::Pose& relative_pose) {
        const float dx = relative_pose.position.x();
        const float dy = relative_pose.position.y();
        const float dz = relative_pose.position.z();
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    std::shared_ptr<drone::hal::CosysRpcClient> rpc_;
    std::string                                 camera_name_;
    std::string                                 vehicle_name_;
    float                                       detection_radius_m_;
    GtClassMap                                  class_map_;
};

}  // namespace

std::unique_ptr<IGroundTruthEmitter> create_cosys_gt_emitter(const drone::Config& full_cfg) {
    auto class_map = GtClassMap::load(full_cfg);
    if (class_map.empty()) {
        DRONE_LOG_INFO("[GT-Cosys] No `gt_class_map` in scenario config — emitter disabled");
        return nullptr;
    }

    // Use the HAL-side shared RPC client rather than constructing a second
    // connection to the same AirSim server. This keeps the benchmark harness
    // and the HAL backends on a single RPC session and matches whatever
    // connect-retry policy CosysRpcClient uses.
    auto rpc = drone::hal::detail::get_shared_cosys_client(full_cfg);
    if (!rpc || !rpc->is_connected()) {
        DRONE_LOG_WARN("[GT-Cosys] Shared RPC client not connected — emitter disabled");
        return nullptr;
    }

    const std::string camera = full_cfg.get<std::string>(drone::cfg_key::cosys_airsim::CAMERA_NAME,
                                                         kDefaultCameraName);
    const std::string vehicle =
        full_cfg.get<std::string>(drone::cfg_key::cosys_airsim::VEHICLE_NAME, kDefaultVehicleName);
    const float radius_m = full_cfg.get<float>(drone::cfg_key::benchmark::GT_DETECTION_RADIUS_M,
                                               kDefaultDetectionRadiusM);

    const auto patterns = class_map.patterns();
    auto       emitter  = std::make_unique<CosysGtEmitter>(rpc, camera, vehicle, radius_m,
                                                           std::move(class_map));
    if (!emitter->register_filters(patterns)) {
        DRONE_LOG_WARN(
            "[GT-Cosys] Filter registration failed — emitter constructed but may emit nothing");
    }
    DRONE_LOG_INFO("[GT-Cosys] Emitter ready — camera={} vehicle={} radius={}m", camera, vehicle,
                   radius_m);
    return emitter;
}

}  // namespace drone::benchmark

#endif  // HAVE_COSYS_AIRSIM
