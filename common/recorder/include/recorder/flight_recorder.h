// common/recorder/include/recorder/flight_recorder.h
// Flight Data Recorder — captures IPC traffic to a binary log file for
// post-flight analysis, debugging, and replay.
//
// Binary log format (per record):
//   [WireHeader (32 bytes)] [topic_name_len (uint16_t)] [topic_name] [payload]
//
// The ring buffer discards oldest records when max_size_bytes is reached.
#pragma once

#include "ipc/ipc_types.h"
#include "ipc/wire_format.h"
#include "util/config_keys.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace drone::recorder {

/// File magic for flight recorder logs: "FLOG" in little-endian.
static constexpr uint32_t kFlightLogMagic = 0x474F4C46;  // "FLOG" LE

/// Flight log file header — written once at the start of every log file.
struct __attribute__((packed)) FlightLogFileHeader {
    uint32_t magic         = kFlightLogMagic;
    uint32_t version       = 1;
    uint64_t start_time_ns = 0;
    uint64_t reserved      = 0;
};

static_assert(sizeof(FlightLogFileHeader) == 24, "FlightLogFileHeader must be 24 bytes");
static_assert(std::is_trivially_copyable_v<FlightLogFileHeader>,
              "FlightLogFileHeader must be trivially copyable for binary I/O");

/// Per-record header in the log file.
struct __attribute__((packed)) RecordHeader {
    drone::ipc::WireHeader wire_header{};
    uint16_t               topic_name_len = 0;
};

static_assert(sizeof(RecordHeader) == 34, "RecordHeader must be 34 bytes");
static_assert(std::is_trivially_copyable_v<RecordHeader>,
              "RecordHeader must be trivially copyable for binary I/O");

/// A single recorded entry in the ring buffer.
struct RecordEntry {
    RecordHeader         header{};
    std::string          topic_name;
    std::vector<uint8_t> payload;

    /// Total serialized size of this entry on disk.
    [[nodiscard]] std::size_t serialized_size() const {
        return sizeof(RecordHeader) + topic_name.size() + payload.size();
    }
};

/// Ring buffer that holds recorded entries up to a configurable byte limit.
/// When full, oldest entries are discarded to make room.
/// Uses std::deque for O(1) front eviction.
class RecordRingBuffer {
public:
    explicit RecordRingBuffer(std::size_t max_bytes) : max_bytes_(max_bytes) {}

    /// Push a new entry. Discards oldest entries if necessary.
    void push(RecordEntry entry) {
        const auto entry_size = entry.serialized_size();
        if (entry_size > max_bytes_) return;  // single entry exceeds entire buffer

        while (current_bytes_ + entry_size > max_bytes_ && !entries_.empty()) {
            current_bytes_ -= entries_.front().serialized_size();
            entries_.pop_front();
        }

        current_bytes_ += entry_size;
        entries_.push_back(std::move(entry));
    }

    /// Access all entries (oldest first).
    [[nodiscard]] const std::deque<RecordEntry>& entries() const { return entries_; }

    /// Current total serialized size in bytes.
    [[nodiscard]] std::size_t current_bytes() const { return current_bytes_; }

    /// Maximum buffer size in bytes.
    [[nodiscard]] std::size_t max_bytes() const { return max_bytes_; }

    /// Number of entries in the buffer.
    [[nodiscard]] std::size_t size() const { return entries_.size(); }

    /// Clear all entries.
    void clear() {
        entries_.clear();
        current_bytes_ = 0;
    }

private:
    std::size_t             max_bytes_     = 0;
    std::size_t             current_bytes_ = 0;
    std::deque<RecordEntry> entries_;
};

/// Write a complete log file from a set of record entries.
/// Accepts any container of RecordEntry (vector, deque, etc.).
/// Returns true on success.
template<typename Container>
[[nodiscard]] inline bool write_log_file(const std::string& path, const Container& entries,
                                         uint64_t start_time_ns = 0) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) return false;

    FlightLogFileHeader file_header{};
    file_header.start_time_ns = start_time_ns;
    const auto* fh_bytes      = reinterpret_cast<const char*>(&file_header);
    out.write(fh_bytes, sizeof(file_header));

    // Maximum topic name length the reader accepts.
    constexpr std::size_t kMaxTopicNameLen = 256;

    for (const auto& entry : entries) {
        // Clamp topic name length to what the reader expects (max 256).
        const std::size_t clamped_len = std::min(entry.topic_name.size(), kMaxTopicNameLen);
        RecordHeader      hdr_copy    = entry.header;
        hdr_copy.topic_name_len       = static_cast<uint16_t>(clamped_len);
        const auto* rh_bytes          = reinterpret_cast<const char*>(&hdr_copy);
        out.write(rh_bytes, sizeof(RecordHeader));
        out.write(entry.topic_name.data(), static_cast<std::streamsize>(clamped_len));
        out.write(reinterpret_cast<const char*>(entry.payload.data()),
                  static_cast<std::streamsize>(entry.payload.size()));
    }

    return out.good();
}

/// Read all record entries from a log file.
/// Returns a ReadLogResult with success=true on complete read, or
/// truncated=true if the file ended mid-record.
struct ReadLogResult {
    FlightLogFileHeader      file_header{};
    std::vector<RecordEntry> entries;
    bool                     success = false;
    bool truncated = false;  ///< True if file ended mid-record or failed validation
};

[[nodiscard]] inline ReadLogResult read_log_file(const std::string& path) {
    ReadLogResult result;
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return result;

    // Read file header
    auto* fh_dst = reinterpret_cast<char*>(&result.file_header);
    in.read(fh_dst, sizeof(FlightLogFileHeader));
    if (!in.good() || result.file_header.magic != kFlightLogMagic) return result;

    // Read entries — track whether loop ends on clean EOF or corruption/truncation
    bool corrupt = false;
    while (in.good() && in.peek() != std::char_traits<char>::eof()) {
        RecordEntry entry;
        auto*       rh_dst = reinterpret_cast<char*>(&entry.header);
        in.read(rh_dst, sizeof(RecordHeader));
        if (!in.good()) {
            corrupt = true;
            break;
        }

        // Validate wire header
        if (entry.header.wire_header.magic != drone::ipc::kWireMagic) {
            corrupt = true;
            break;
        }

        // Read topic name
        const auto topic_len = entry.header.topic_name_len;
        if (topic_len > 256) {
            corrupt = true;
            break;
        }
        entry.topic_name.resize(topic_len);
        in.read(entry.topic_name.data(), topic_len);
        if (!in.good()) {
            corrupt = true;
            break;
        }

        // Read payload
        const auto payload_size = entry.header.wire_header.payload_size;
        if (payload_size > 64u * 1024u * 1024u) {
            corrupt = true;
            break;
        }
        entry.payload.resize(payload_size);
        in.read(reinterpret_cast<char*>(entry.payload.data()),
                static_cast<std::streamsize>(payload_size));
        if (!in.good() && !in.eof()) {
            corrupt = true;
            break;
        }
        // Trim to actual bytes read on partial read (truncated file)
        const auto actual = static_cast<std::size_t>(in.gcount());
        if (actual < payload_size) {
            entry.payload.resize(actual);
            entry.header.wire_header.payload_size = static_cast<uint32_t>(actual);
            corrupt                               = true;  // partial read means truncation
        }

        result.entries.push_back(std::move(entry));

        if (corrupt) break;
    }

    result.truncated = corrupt;
    result.success   = !corrupt;
    return result;
}

/// Flight data recorder — subscribes to IPC topics and records traffic
/// to an in-memory ring buffer, then flushes to disk.
///
/// Thread safety:
///   - start()/stop()/is_recording() use an atomic flag (lock-free).
///   - record() prepares the entry then acquires the mutex to push into the ring buffer.
///     Timestamp is captured inside the critical section to guarantee monotonic ordering.
///   - flush() copies entries under the mutex, releases it, then writes to disk.
///   - entry_count()/buffered_bytes() acquire the mutex for consistent reads.
///
/// Usage:
///   FlightRecorder recorder(cfg);
///   recorder.start();
///   // ... flight happens ...
///   recorder.flush();   // writes to disk
///   recorder.stop();
class FlightRecorder {
public:
    /// Construct with explicit parameters.
    FlightRecorder(std::size_t max_size_bytes, const std::string& output_dir)
        : ring_buffer_(max_size_bytes), output_dir_(output_dir) {}

    /// Construct from config object.
    /// Reads: recorder.max_size_mb (clamped to [1, 4096], default 64),
    ///        recorder.output_dir (default /tmp/flight_logs)
    template<typename ConfigT>
    explicit FlightRecorder(const ConfigT& cfg)
        : ring_buffer_(
              validated_max_bytes(cfg.template get<int>(drone::cfg_key::recorder::MAX_SIZE_MB, 64)))
        , output_dir_(cfg.template get<std::string>(drone::cfg_key::recorder::OUTPUT_DIR,
                                                    "/tmp/flight_logs")) {}

    /// Record a message from a given topic.
    /// @tparam T  Trivially copyable IPC struct.
    template<typename T>
    void record(const std::string& topic, drone::ipc::WireMessageType msg_type, const T& msg,
                uint32_t seq = 0) {
        static_assert(std::is_trivially_copyable_v<T>, "record() requires trivially copyable T");
        if (!recording_.load(std::memory_order_acquire)) return;

        // Prepare entry fields that don't need synchronization
        RecordEntry entry;
        entry.header.wire_header.magic        = drone::ipc::kWireMagic;
        entry.header.wire_header.version      = drone::ipc::kWireVersion;
        entry.header.wire_header.msg_type     = msg_type;
        entry.header.wire_header.payload_size = static_cast<uint32_t>(sizeof(T));
        entry.header.wire_header.sequence     = seq;
        entry.header.topic_name_len =
            static_cast<uint16_t>(std::min(topic.size(), std::size_t{256}));

        entry.topic_name = topic.substr(0, entry.header.topic_name_len);
        entry.payload.resize(sizeof(T));
        const auto* src = reinterpret_cast<const uint8_t*>(&msg);
        std::copy(src, src + sizeof(T), entry.payload.data());

        // Capture timestamp and push under the lock for monotonic ordering
        std::lock_guard<std::mutex> lock(mutex_);
        const auto                  now_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        // Enforce monotonic: never go backwards
        last_timestamp_ns_                    = std::max(last_timestamp_ns_ + 1, now_ns);
        entry.header.wire_header.timestamp_ns = last_timestamp_ns_;
        ring_buffer_.push(std::move(entry));
    }

    /// Start recording.
    void start() { recording_.store(true, std::memory_order_release); }

    /// Stop recording.
    void stop() { recording_.store(false, std::memory_order_release); }

    /// Whether recording is active.
    [[nodiscard]] bool is_recording() const { return recording_.load(std::memory_order_acquire); }

    /// Flush the ring buffer to a timestamped log file.
    /// Copies entries under the lock, releases it, then writes to disk so
    /// record() is not blocked during file I/O.
    /// Returns the path of the written file, or empty string on failure.
    [[nodiscard]] std::string flush() {
        // Move entries out under the lock — O(1) swap avoids duplicating payload memory.
        std::vector<RecordEntry> snapshot;
        uint64_t                 start_ns = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (ring_buffer_.size() == 0) return "";

            // Move entries from deque into vector for writing
            snapshot.reserve(ring_buffer_.size());
            for (auto& e : ring_buffer_.entries()) {
                snapshot.push_back(std::move(e));
            }
            start_ns = snapshot.front().header.wire_header.timestamp_ns;
            ring_buffer_.clear();
        }
        // Lock released — write to disk without blocking record()

        // Create output directory if it doesn't exist
        std::error_code ec;
        std::filesystem::create_directories(output_dir_, ec);
        if (ec) {
            // Log warning but continue — the write below will fail if dir is missing
        }

        const auto now   = std::chrono::system_clock::now();
        const auto epoch = now.time_since_epoch();
        const auto secs  = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();
        const auto seq   = s_flush_seq_.fetch_add(1, std::memory_order_relaxed);

        const std::string filename = output_dir_ + "/flight_" + std::to_string(secs) + "_" +
                                     std::to_string(seq) + ".flog";

        if (!write_log_file(filename, snapshot, start_ns)) {
            return "";
        }

        return filename;
    }

    /// Current number of buffered entries.
    [[nodiscard]] std::size_t entry_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return ring_buffer_.size();
    }

    /// Current buffered bytes.
    [[nodiscard]] std::size_t buffered_bytes() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return ring_buffer_.current_bytes();
    }

private:
    /// Validate and clamp max_size_mb to a sane range.
    static std::size_t validated_max_bytes(int max_size_mb) {
        constexpr int kDefaultMb = 64;
        constexpr int kMaxMb     = 4096;
        if (max_size_mb <= 0) {
            max_size_mb = kDefaultMb;
        } else if (max_size_mb > kMaxMb) {
            max_size_mb = kMaxMb;
        }
        return static_cast<std::size_t>(max_size_mb) * 1024 * 1024;
    }

    mutable std::mutex                  mutex_;
    std::atomic<bool>                   recording_{false};
    RecordRingBuffer                    ring_buffer_;
    std::string                         output_dir_;
    uint64_t                            last_timestamp_ns_ = 0;
    static inline std::atomic<uint64_t> s_flush_seq_{0};
};

}  // namespace drone::recorder
