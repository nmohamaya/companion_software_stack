// common/hal/include/hal/cosys_camera.h
// Cosys-AirSim camera backend — receives images via AirSim RPC API.
// Implements ICamera — connects to Cosys-AirSim simGetImages() endpoint.
// Guarded by HAVE_COSYS_AIRSIM (set by CMake when AirSim SDK is found).
//
// Thread-safety:
//   - Image retrieval thread (producer) writes frames via TripleBuffer::write()
//   - capture() (consumer) reads frames via TripleBuffer::read()
//   - Lock-free: no mutex on the hot path (frame handoff)
//   - Mutex used only for open/close state transitions (cold path)
//
// Issue: #434, #462
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_rpc_client.h"
#include "hal/icamera.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"
#include "util/triple_buffer.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

namespace drone::hal {

/// Cosys-AirSim camera backend using AirSim RPC API.
///
/// Connects to a Cosys-AirSim simulation and retrieves camera images via
/// simGetImages(). Uses a lock-free TripleBuffer for frame handoff between
/// the retrieval thread (producer) and capture() (consumer).
///
/// The retrieval thread runs at the configured FPS, calling simGetImages()
/// with ImageType::Scene to get uncompressed RGB frames.
class CosysCameraBackend : public ICamera {
public:
    /// Frame data passed through the lock-free TripleBuffer.
    /// Bundles pixel data + metadata so the consumer gets a consistent snapshot.
    struct FrameData {
        std::vector<uint8_t> pixels;           ///< RGB pixel data
        uint32_t             width{0};         ///< Frame width
        uint32_t             height{0};        ///< Frame height
        uint32_t             channels{3};      ///< Always 3 (RGB)
        uint64_t             timestamp_ns{0};  ///< Capture timestamp
    };

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

    // Non-copyable, non-movable (owns retrieval thread)
    CosysCameraBackend(const CosysCameraBackend&)            = delete;
    CosysCameraBackend& operator=(const CosysCameraBackend&) = delete;
    CosysCameraBackend(CosysCameraBackend&&)                 = delete;
    CosysCameraBackend& operator=(CosysCameraBackend&&)      = delete;

    // ── ICamera interface ──────────────────────────────────

    bool open(uint32_t width, uint32_t height, int fps) override {
        std::lock_guard<std::mutex> lock(state_mtx_);
        if (open_.load(std::memory_order_acquire)) {
            DRONE_LOG_WARN("[CosysCamera] Already open — call close() first");
            return false;
        }

        if (!client_->is_connected()) {
            DRONE_LOG_ERROR("[CosysCamera] RPC client not connected — cannot open camera");
            return false;
        }

        width_    = width;
        height_   = height;
        fps_      = fps;
        sequence_ = 0;

        open_.store(true, std::memory_order_release);

        // Start image retrieval thread — polls simGetImages() at target FPS
        retrieval_thread_ = std::thread(&CosysCameraBackend::retrieval_loop, this);

        DRONE_LOG_INFO("[CosysCamera] Opened camera='{}' on {} ({}x{}@{}Hz)", camera_name_,
                       client_->endpoint(), width_, height_, fps_);
        return true;
    }

    void close() override {
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            if (!open_.load(std::memory_order_acquire)) return;
            open_.store(false, std::memory_order_release);
        }

        // Join retrieval thread outside the lock to avoid deadlock
        if (retrieval_thread_.joinable()) {
            retrieval_thread_.join();
        }

        DRONE_LOG_INFO("[CosysCamera] Closed camera='{}'", camera_name_);
    }

    CapturedFrame capture() override {
        CapturedFrame frame;
        if (!open_.load(std::memory_order_acquire)) return frame;

        // Lock-free read from TripleBuffer — returns nullopt if no new frame
        auto latest = frame_buffer_.read();
        if (!latest) {
            // No new frame yet — return the last frame if we have one
            if (!last_frame_.pixels.empty()) {
                return build_frame(last_frame_);
            }
            return frame;  // No frame available at all
        }

        // Got a new frame — cache it and return
        last_frame_ = std::move(*latest);
        return build_frame(last_frame_);
    }

    bool is_open() const override { return open_.load(std::memory_order_acquire); }

    std::string name() const override {
        return "CosysCamera(" + camera_name_ + "@" + client_->endpoint() + ")";
    }

private:
    /// Build a CapturedFrame from FrameData. The returned frame points into
    /// last_frame_.pixels which is stable until the next capture() call.
    CapturedFrame build_frame(const FrameData& data) {
        CapturedFrame frame;
        frame.timestamp_ns = data.timestamp_ns;
        frame.sequence     = sequence_++;
        frame.width        = data.width;
        frame.height       = data.height;
        frame.channels     = data.channels;
        frame.stride       = data.width * data.channels;
        frame.data         = data.pixels.data();
        frame.valid        = true;
        return frame;
    }

    /// Image retrieval thread — polls AirSim simGetImages() at target FPS.
    /// Writes frames to TripleBuffer (lock-free, never blocks).
    void retrieval_loop() {
        using namespace msr::airlib;
        const auto frame_interval = std::chrono::milliseconds(fps_ > 0 ? 1000 / fps_
                                                                       : 33);  // default ~30 FPS

        DRONE_LOG_INFO("[CosysCamera] Retrieval thread started (interval={}ms)",
                       frame_interval.count());

        while (open_.load(std::memory_order_acquire)) {
            try {
                if (!client_->is_connected()) {
                    DRONE_LOG_WARN("[CosysCamera] RPC disconnected — skipping frame");
                    std::this_thread::sleep_for(frame_interval);
                    continue;
                }

                // Request uncompressed RGB Scene image from AirSim
                std::vector<ImageCaptureBase::ImageRequest> requests = {
                    ImageCaptureBase::ImageRequest(camera_name_, ImageCaptureBase::ImageType::Scene,
                                                   /* pixels_as_float */ false,
                                                   /* compress */ false)};

                auto responses = client_->rpc_client().simGetImages(requests, vehicle_name_);

                if (responses.empty() || responses[0].image_data_uint8.empty()) {
                    std::this_thread::sleep_for(frame_interval);
                    continue;
                }

                const auto& resp     = responses[0];
                const auto  channels = uint32_t{3};
                const auto  expected = static_cast<size_t>(resp.width) * resp.height * channels;

                if (resp.image_data_uint8.size() < expected || resp.width == 0 ||
                    resp.height == 0) {
                    DRONE_LOG_WARN("[CosysCamera] Unexpected image size: {}x{} ({} bytes, "
                                   "expected {})",
                                   resp.width, resp.height, resp.image_data_uint8.size(), expected);
                    std::this_thread::sleep_for(frame_interval);
                    continue;
                }

                // Build FrameData and write to TripleBuffer (lock-free, never blocks)
                FrameData fd;
                fd.pixels.assign(resp.image_data_uint8.begin(),
                                 resp.image_data_uint8.begin() + static_cast<ptrdiff_t>(expected));
                fd.width    = static_cast<uint32_t>(resp.width);
                fd.height   = static_cast<uint32_t>(resp.height);
                fd.channels = channels;
                fd.timestamp_ns =
                    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::steady_clock::now().time_since_epoch())
                                              .count());

                frame_buffer_.write(std::move(fd));

            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysCamera] RPC error in retrieval thread: {}", e.what());
            }

            std::this_thread::sleep_for(frame_interval);
        }

        DRONE_LOG_INFO("[CosysCamera] Retrieval thread exiting");
    }

    // ── Shared RPC client (shared_ptr: shared across 4 HAL backends) ──
    std::shared_ptr<CosysRpcClient> client_;

    // ── Config ─────────────────────────────────────────────
    std::string camera_name_;   ///< AirSim camera name
    std::string vehicle_name_;  ///< AirSim vehicle name

    // ── State (cold path — mutex for open/close only) ──────
    std::mutex        state_mtx_;    ///< Guards open/close transitions only
    std::atomic<bool> open_{false};  ///< Atomic for lock-free checks in hot path

    uint32_t width_{0};
    uint32_t height_{0};
    int      fps_{0};
    uint64_t sequence_{0};

    // ── Lock-free frame handoff (hot path) ─────────────────
    // Producer (retrieval thread) writes via frame_buffer_.write()
    // Consumer (capture()) reads via frame_buffer_.read()
    // No mutex, no cv — TripleBuffer handles synchronisation via atomics.
    drone::TripleBuffer<FrameData> frame_buffer_;
    FrameData                      last_frame_;  ///< Most recent frame for repeat reads

    // ── Image retrieval thread ─────────────────────────────
    std::thread retrieval_thread_;  ///< Polls simGetImages() at target FPS
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
