// common/hal/include/hal/cosys_camera.h
// Cosys-AirSim camera backend — receives images via AirSim RPC API.
// Implements ICamera — connects to Cosys-AirSim simGetImages() endpoint.
// Guarded by HAVE_COSYS_AIRSIM (set by CMake when AirSim SDK is found).
//
// Thread-safety:
//   - AirSim RPC response is written to back-buffer
//   - capture() swaps buffers and returns the latest frame
//   - Synchronised via mutex + condition_variable
//
// Issue: #434
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_rpc_client.h"
#include "hal/icamera.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace drone::hal {

/// Cosys-AirSim camera backend using AirSim RPC API.
///
/// Connects to a Cosys-AirSim simulation and retrieves camera images via
/// simGetImages(). Uses a double-buffer scheme: the retrieval thread writes
/// to the back-buffer, and `capture()` swaps it to the front-buffer.
class CosysCameraBackend : public ICamera {
public:
    /// @param client   Shared RPC client (manages connection lifecycle)
    /// @param cfg      Loaded configuration
    /// @param section  Config path prefix (e.g. "video_capture.mission_cam")
    explicit CosysCameraBackend(std::shared_ptr<CosysRpcClient> client, const drone::Config& cfg,
                                const std::string& section)
        : client_(std::move(client))
        , camera_name_(cfg.get<std::string>(std::string(drone::cfg_key::cosys_airsim::CAMERA_NAME),
                                            "front_center"))
        , vehicle_name_(cfg.get<std::string>(
              std::string(drone::cfg_key::cosys_airsim::VEHICLE_NAME), "Drone0")) {
        (void)section;  // section reserved for future per-camera config
        DRONE_LOG_INFO("[CosysCamera] Created for {} camera='{}' vehicle='{}'", client_->endpoint(),
                       camera_name_, vehicle_name_);
    }

    ~CosysCameraBackend() override { close(); }

    // Non-copyable, non-movable (owns RPC connection state)
    CosysCameraBackend(const CosysCameraBackend&)            = delete;
    CosysCameraBackend& operator=(const CosysCameraBackend&) = delete;
    CosysCameraBackend(CosysCameraBackend&&)                 = delete;
    CosysCameraBackend& operator=(CosysCameraBackend&&)      = delete;

    // ── ICamera interface ──────────────────────────────────

    bool open(uint32_t width, uint32_t height, int fps) override {
        std::lock_guard<std::mutex> lock(mtx_);
        if (open_) {
            DRONE_LOG_WARN("[CosysCamera] Already open — call close() first");
            return false;
        }

        width_       = width;
        height_      = height;
        fps_         = fps;
        channels_    = 3;  // RGB
        sequence_    = 0;
        frame_ready_ = false;

        // Pre-allocate double buffers (RGB)
        const auto buf_size = static_cast<size_t>(height_) * width_ * channels_;
        back_buffer_.resize(buf_size, 0);
        front_buffer_.resize(buf_size, 0);

        // TODO(#462): Start image retrieval thread using simGetImages() API
        // via client_. Until SDK is integrated, open() succeeds but capture()
        // will timeout (no frames arrive). This allows pipeline startup without the SDK.

        open_ = true;
        DRONE_LOG_WARN("[CosysCamera] SDK not connected — capture() will timeout until "
                       "AirSim RPC integration is implemented");
        DRONE_LOG_INFO("[CosysCamera] Opened camera='{}' on {} ({}x{}@{}Hz)", camera_name_,
                       client_->endpoint(), width_, height_, fps_);
        return true;
    }

    void close() override {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!open_) return;

        // TODO(#462): Stop image retrieval thread.
        // Connection lifecycle is managed by the shared CosysRpcClient.

        open_        = false;
        frame_ready_ = false;

        // Wake any thread blocked in capture()
        cv_.notify_all();

        DRONE_LOG_INFO("[CosysCamera] Closed camera='{}'", camera_name_);
    }

    CapturedFrame capture() override {
        std::unique_lock<std::mutex> lock(mtx_);
        CapturedFrame                frame;
        if (!open_) return frame;

        // Wait for a frame (with timeout)
        constexpr auto kTimeout = std::chrono::seconds(2);
        if (!cv_.wait_for(lock, kTimeout, [this] { return frame_ready_ || !open_; })) {
            DRONE_LOG_WARN("[CosysCamera] capture() timed out on camera='{}'", camera_name_);
            return frame;
        }

        if (!open_) return frame;

        // Swap front/back buffers
        std::swap(front_buffer_, back_buffer_);
        frame_ready_ = false;

        // Build CapturedFrame from front buffer
        frame.timestamp_ns = last_timestamp_ns_;
        frame.sequence     = sequence_++;
        frame.width        = width_;
        frame.height       = height_;
        frame.channels     = channels_;
        frame.stride       = width_ * channels_;
        frame.data         = front_buffer_.data();
        frame.valid        = true;

        return frame;
    }

    bool is_open() const override {
        std::lock_guard<std::mutex> lock(mtx_);
        return open_;
    }

    std::string name() const override {
        return "CosysCamera(" + camera_name_ + "@" + client_->endpoint() + ")";
    }

private:
    // ── Shared RPC client (shared_ptr: shared across 4 HAL backends) ──
    std::shared_ptr<CosysRpcClient> client_;

    // ── Config ─────────────────────────────────────────────
    std::string camera_name_;   ///< AirSim camera name
    std::string vehicle_name_;  ///< AirSim vehicle name

    // ── Synchronisation ────────────────────────────────────
    mutable std::mutex      mtx_;  ///< Guards all mutable state
    std::condition_variable cv_;   ///< Signals new frame available

    bool     open_{false};
    uint32_t width_{0};
    uint32_t height_{0};
    int      fps_{0};
    uint32_t channels_{3};

    // Double-buffer scheme
    std::vector<uint8_t> back_buffer_;         ///< Written by retrieval thread
    std::vector<uint8_t> front_buffer_;        ///< Read by capture()
    bool                 frame_ready_{false};  ///< Back buffer has new data

    // Last received frame metadata
    uint64_t last_timestamp_ns_{0};
    uint64_t sequence_{0};
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
