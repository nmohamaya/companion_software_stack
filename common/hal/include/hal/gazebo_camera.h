// common/hal/include/hal/gazebo_camera.h
// Gazebo camera backend — receives images from Gazebo via gz-transport.
// Implements ICamera — subscribes to a gz::msgs::Image topic.
// Guarded by HAVE_GAZEBO (set by CMake when gz-transport13 + gz-msgs10 are found).
//
// The Gazebo topic is set via constructor argument (read from config "gz_topic"):
//   GazeboCameraBackend("/camera")
//
// Thread-safety:
//   - gz-transport callback runs on its own thread, writes to back-buffer
//   - capture() swaps buffers and returns the latest frame
//   - Synchronised via mutex + condition_variable
//
// Issue: #9
#pragma once
#ifdef HAVE_GAZEBO

#include "hal/icamera.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <spdlog/spdlog.h>

namespace drone::hal {

/// Gazebo camera backend using gz-transport.
///
/// Subscribes to a Gazebo Image topic and presents frames via the ICamera
/// interface.  Uses a double-buffer scheme: the gz-transport callback writes
/// to the back-buffer, and `capture()` swaps it to the front-buffer.
class GazeboCameraBackend : public ICamera {
public:
    /// @param gz_topic  Gazebo transport topic to subscribe to
    ///                  (e.g. "/camera", "/stereo_left").
    explicit GazeboCameraBackend(std::string gz_topic) : gz_topic_(std::move(gz_topic)) {}

    ~GazeboCameraBackend() override { close(); }

    // Non-copyable, non-movable (owns gz::transport::Node)
    GazeboCameraBackend(const GazeboCameraBackend&)            = delete;
    GazeboCameraBackend& operator=(const GazeboCameraBackend&) = delete;
    GazeboCameraBackend(GazeboCameraBackend&&)                 = delete;
    GazeboCameraBackend& operator=(GazeboCameraBackend&&)      = delete;

    // ── ICamera interface ──────────────────────────────────

    bool open(uint32_t width, uint32_t height, int fps) override {
        std::lock_guard<std::mutex> lock(mtx_);
        if (open_) {
            spdlog::warn("[GazeboCameraBackend] Already open — call close() first");
            return false;
        }

        width_       = width;
        height_      = height;
        fps_         = fps;
        channels_    = 3;  // Default RGB; adjusted on first frame
        stride_      = width_ * channels_;
        sequence_    = 0;
        frame_ready_ = false;

        // Pre-allocate double buffers (RGB assumed)
        size_t buf_size = static_cast<size_t>(height_) * width_ * 4;  // max RGBA
        back_buffer_.resize(buf_size, 0);
        front_buffer_.resize(buf_size, 0);

        // Subscribe to the Gazebo image topic
        bool subscribed = node_.Subscribe(gz_topic_, &GazeboCameraBackend::on_image, this);

        if (!subscribed) {
            spdlog::error("[GazeboCameraBackend] Failed to subscribe to '{}'", gz_topic_);
            return false;
        }

        open_ = true;
        spdlog::info("[GazeboCameraBackend] Subscribed to '{}' (expecting {}x{}@{}Hz)", gz_topic_,
                     width_, height_, fps_);
        return true;
    }

    void close() override {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!open_) return;

        node_.Unsubscribe(gz_topic_);
        open_        = false;
        frame_ready_ = false;

        // Wake any thread blocked in capture()
        cv_.notify_all();

        spdlog::info("[GazeboCameraBackend] Closed (topic='{}')", gz_topic_);
    }

    CapturedFrame capture() override {
        std::unique_lock<std::mutex> lock(mtx_);
        CapturedFrame                frame;
        if (!open_) return frame;

        // Wait for a frame (with timeout)
        constexpr auto kTimeout = std::chrono::seconds(2);
        if (!cv_.wait_for(lock, kTimeout, [this] { return frame_ready_ || !open_; })) {
            spdlog::warn("[GazeboCameraBackend] capture() timed out on '{}'", gz_topic_);
            return frame;
        }

        if (!open_) return frame;

        // Swap front/back buffers
        std::swap(front_buffer_, back_buffer_);
        frame_ready_ = false;

        // Build CapturedFrame from front buffer
        frame.timestamp_ns = last_timestamp_ns_;
        frame.sequence     = sequence_++;
        frame.width        = last_width_;
        frame.height       = last_height_;
        frame.channels     = last_channels_;
        frame.stride       = last_width_ * last_channels_;
        frame.data         = front_buffer_.data();
        frame.valid        = true;

        return frame;
    }

    bool is_open() const override {
        std::lock_guard<std::mutex> lock(mtx_);
        return open_;
    }

    std::string name() const override { return "GazeboCameraBackend(" + gz_topic_ + ")"; }

private:
    // ── gz-transport callback ──────────────────────────────
    void on_image(const gz::msgs::Image& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!open_) return;

        uint32_t msg_w  = msg.width();
        uint32_t msg_h  = msg.height();
        auto     pf     = msg.pixel_format_type();
        uint32_t src_ch = pixel_format_channels(pf);
        if (src_ch == 0) {
            spdlog::warn("[GazeboCameraBackend] Unsupported pixel format {} on '{}'",
                         static_cast<int>(pf), gz_topic_);
            return;
        }

        // Determine output channels (match source, or convert RGBA/BGRA→RGB)
        uint32_t dst_ch = (src_ch == 4) ? 3 : src_ch;  // strip alpha

        // Verify data size
        size_t expected_size = static_cast<size_t>(msg_w) * msg_h * src_ch;
        if (msg.data().size() < expected_size) {
            spdlog::warn("[GazeboCameraBackend] Data size mismatch: got {} expected {} on '{}'",
                         msg.data().size(), expected_size, gz_topic_);
            return;
        }

        // Ensure back buffer is large enough
        size_t dst_size = static_cast<size_t>(msg_w) * msg_h * dst_ch;
        if (back_buffer_.size() < dst_size) {
            back_buffer_.resize(dst_size);
        }

        // Convert pixel data to output buffer
        const auto* src = reinterpret_cast<const uint8_t*>(msg.data().data());

        if (src_ch == dst_ch && !is_bgr_format(pf)) {
            // Direct copy: L_INT8 or RGB_INT8
            std::copy(src, src + dst_size, back_buffer_.data());
        } else if (is_bgr_format(pf) && src_ch == 3) {
            // BGR → RGB
            convert_bgr_to_rgb(src, back_buffer_.data(), msg_w * msg_h);
        } else if (pf == gz::msgs::RGBA_INT8) {
            // RGBA → RGB (drop alpha)
            convert_rgba_to_rgb(src, back_buffer_.data(), msg_w * msg_h);
        } else if (pf == gz::msgs::BGRA_INT8) {
            // BGRA → RGB (swap + drop alpha)
            convert_bgra_to_rgb(src, back_buffer_.data(), msg_w * msg_h);
        } else {
            // Fallback: just copy what fits
            const auto fallback_size = std::min(dst_size, msg.data().size());
            std::copy(src, src + fallback_size, back_buffer_.data());
        }

        last_width_    = msg_w;
        last_height_   = msg_h;
        last_channels_ = dst_ch;
        last_timestamp_ns_ =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());

        frame_ready_ = true;
        cv_.notify_one();
    }

    // ── Pixel format helpers ───────────────────────────────

    /// Return number of bytes per pixel for supported formats (0 = unsupported).
    static uint32_t pixel_format_channels(gz::msgs::PixelFormatType pf) {
        switch (pf) {
            case gz::msgs::L_INT8: return 1;
            case gz::msgs::RGB_INT8: return 3;
            case gz::msgs::BGR_INT8: return 3;
            case gz::msgs::RGBA_INT8: return 4;
            case gz::msgs::BGRA_INT8: return 4;
            default: return 0;  // 16/32-bit not supported
        }
    }

    static bool is_bgr_format(gz::msgs::PixelFormatType pf) { return pf == gz::msgs::BGR_INT8; }

    static void convert_bgr_to_rgb(const uint8_t* src, uint8_t* dst, size_t pixel_count) {
        for (size_t i = 0; i < pixel_count; ++i) {
            dst[i * 3 + 0] = src[i * 3 + 2];  // R ← B
            dst[i * 3 + 1] = src[i * 3 + 1];  // G ← G
            dst[i * 3 + 2] = src[i * 3 + 0];  // B ← R
        }
    }

    static void convert_rgba_to_rgb(const uint8_t* src, uint8_t* dst, size_t pixel_count) {
        for (size_t i = 0; i < pixel_count; ++i) {
            dst[i * 3 + 0] = src[i * 4 + 0];
            dst[i * 3 + 1] = src[i * 4 + 1];
            dst[i * 3 + 2] = src[i * 4 + 2];
        }
    }

    static void convert_bgra_to_rgb(const uint8_t* src, uint8_t* dst, size_t pixel_count) {
        for (size_t i = 0; i < pixel_count; ++i) {
            dst[i * 3 + 0] = src[i * 4 + 2];  // R ← B
            dst[i * 3 + 1] = src[i * 4 + 1];  // G ← G
            dst[i * 3 + 2] = src[i * 4 + 0];  // B ← R
        }
    }

    // ── Members ────────────────────────────────────────────
    std::string         gz_topic_;  ///< Gazebo transport topic name
    gz::transport::Node node_;      ///< Gazebo transport node

    mutable std::mutex      mtx_;  ///< Guards all mutable state
    std::condition_variable cv_;   ///< Signals new frame available

    bool     open_{false};
    uint32_t width_{0};
    uint32_t height_{0};
    int      fps_{0};
    uint32_t channels_{3};
    uint32_t stride_{0};

    // Double-buffer scheme
    std::vector<uint8_t> back_buffer_;         ///< Written by gz callback
    std::vector<uint8_t> front_buffer_;        ///< Read by capture()
    bool                 frame_ready_{false};  ///< Back buffer has new data

    // Last received frame metadata
    uint32_t last_width_{0};
    uint32_t last_height_{0};
    uint32_t last_channels_{0};
    uint64_t last_timestamp_ns_{0};
    uint64_t sequence_{0};
};

}  // namespace drone::hal

#endif  // HAVE_GAZEBO
