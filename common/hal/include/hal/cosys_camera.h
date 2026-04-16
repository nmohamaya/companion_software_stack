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

        width_  = width;
        height_ = height;
        fps_    = fps;
        sequence_.store(0, std::memory_order_relaxed);

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

    /// Read the latest frame from the TripleBuffer (lock-free).
    /// SINGLE-CONSUMER: this method must only be called from one thread.
    /// The returned CapturedFrame owns its pixel data (valid for frame lifetime).
    CapturedFrame capture() override {
        CapturedFrame frame;
        if (!open_.load(std::memory_order_acquire)) return frame;

        // Lock-free read from TripleBuffer — returns nullopt if no new frame
        auto latest = frame_buffer_.read();
        if (!latest) {
            // No new frame yet — return the last frame with same sequence
            // (don't increment — consumers use sequence to detect new frames)
            if (!last_frame_.pixels.empty()) {
                return build_frame(last_frame_, false);
            }
            return frame;  // No frame available at all
        }

        // Got a new frame — cache it and return with incremented sequence
        last_frame_ = std::move(*latest);
        return build_frame(last_frame_, true);
    }

    bool is_open() const override { return open_.load(std::memory_order_acquire); }

    std::string name() const override {
        return "CosysCamera(" + camera_name_ + "@" + client_->endpoint() + ")";
    }

private:
    /// Build a CapturedFrame from FrameData. The returned frame owns a copy of
    /// the pixel data, so it remains valid for its entire lifetime.
    /// @param new_frame  If true, increment sequence (new data). If false, reuse
    ///                   last sequence (duplicate frame, no new data from producer).
    CapturedFrame build_frame(const FrameData& data, bool new_frame) {
        CapturedFrame frame;
        frame.timestamp_ns = data.timestamp_ns;
        frame.sequence     = new_frame ? sequence_.fetch_add(1, std::memory_order_relaxed)
                                       : sequence_.load(std::memory_order_relaxed);
        frame.width        = data.width;
        frame.height       = data.height;
        frame.channels     = data.channels;
        frame.stride       = data.width * data.channels;
        frame.data         = data.pixels;  // copy from cached FrameData
        frame.valid        = true;
        return frame;
    }

    /// Image retrieval thread — polls AirSim simGetImages() at target FPS.
    /// Writes frames to TripleBuffer (lock-free, never blocks).
    void retrieval_loop() {
        using namespace msr::airlib;
        // Clamp to minimum 1ms to prevent hot-loop on misconfigured fps > 1000
        const auto frame_interval =
            std::chrono::milliseconds(std::max(1, fps_ > 0 ? 1000 / fps_ : 33));

        DRONE_LOG_INFO("[CosysCamera] Retrieval thread started (interval={}ms)",
                       frame_interval.count());

        constexpr uint32_t kRGBChannels = 3;

        // Hoist request vector outside the loop — same request every frame
        std::vector<ImageCaptureBase::ImageRequest> requests = {
            ImageCaptureBase::ImageRequest(camera_name_, ImageCaptureBase::ImageType::Scene,
                                           /* pixels_as_float */ false,
                                           /* compress */ false)};

        while (open_.load(std::memory_order_acquire)) {
            try {
                // Use with_client() to prevent TOCTOU race on disconnect
                bool got_frame = client_->with_client([&](auto& rpc) {
                    auto responses = rpc.simGetImages(requests, vehicle_name_);

                    if (responses.empty() || responses[0].image_data_uint8.empty()) {
                        return;  // no frame this tick
                    }

                    const auto& resp = responses[0];

                    // Guard signed→unsigned: check > 0 BEFORE casting
                    if (resp.width <= 0 || resp.height <= 0) {
                        DRONE_LOG_WARN("[CosysCamera] Invalid dimensions: {}x{}", resp.width,
                                       resp.height);
                        return;
                    }

                    const auto w        = static_cast<uint32_t>(resp.width);
                    const auto h        = static_cast<uint32_t>(resp.height);
                    const auto expected = static_cast<size_t>(w) * h * kRGBChannels;

                    if (resp.image_data_uint8.size() < expected) {
                        DRONE_LOG_WARN("[CosysCamera] Unexpected size: {}x{} ({} bytes, "
                                       "expected {})",
                                       w, h, resp.image_data_uint8.size(), expected);
                        return;
                    }

                    // Build FrameData and write to TripleBuffer (lock-free, never blocks)
                    FrameData fd;
                    fd.pixels.assign(resp.image_data_uint8.begin(),
                                     resp.image_data_uint8.begin() +
                                         static_cast<ptrdiff_t>(expected));
                    fd.width         = w;
                    fd.height        = h;
                    fd.channels      = kRGBChannels;
                    const auto ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           std::chrono::steady_clock::now().time_since_epoch())
                                           .count();
                    fd.timestamp_ns = static_cast<uint64_t>(std::max(decltype(ts_ns){0}, ts_ns));

                    frame_buffer_.write(std::move(fd));
                });

                if (!got_frame) {
                    // RPC client disconnected — skip this tick
                    DRONE_LOG_WARN("[CosysCamera] RPC disconnected — skipping frame");
                }

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

    uint32_t              width_{0};
    uint32_t              height_{0};
    int                   fps_{0};
    std::atomic<uint64_t> sequence_{
        0};  ///< Frame sequence (atomic: written by capture(), read after open())

    // ── Lock-free frame handoff (hot path) ─────────────────
    // Producer (retrieval thread) writes via frame_buffer_.write()
    // Consumer (capture()) reads via frame_buffer_.read()
    // No mutex, no cv — TripleBuffer handles synchronisation via atomics.
    drone::TripleBuffer<FrameData> frame_buffer_;
    /// Most recent frame for repeat reads. Only accessed from capture() which
    /// is single-consumer (ICamera contract). Not thread-safe for concurrent callers.
    FrameData last_frame_;

    // ── Image retrieval thread ─────────────────────────────
    std::thread retrieval_thread_;  ///< Polls simGetImages() at target FPS
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
