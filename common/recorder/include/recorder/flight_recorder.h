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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
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
class RecordRingBuffer {
public:
    explicit RecordRingBuffer(std::size_t max_bytes) : max_bytes_(max_bytes) {}

    /// Push a new entry. Discards oldest entries if necessary.
    void push(RecordEntry entry) {
        const auto entry_size = entry.serialized_size();
        if (entry_size > max_bytes_) return;  // single entry exceeds entire buffer

        while (current_bytes_ + entry_size > max_bytes_ && !entries_.empty()) {
            current_bytes_ -= entries_.front().serialized_size();
            entries_.erase(entries_.begin());
        }

        current_bytes_ += entry_size;
        entries_.push_back(std::move(entry));
    }

    /// Access all entries (oldest first).
    [[nodiscard]] const std::vector<RecordEntry>& entries() const { return entries_; }

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
    std::size_t              max_bytes_     = 0;
    std::size_t              current_bytes_ = 0;
    std::vector<RecordEntry> entries_;
};

/// Write a complete log file from a set of record entries.
/// Returns true on success.
[[nodiscard]] inline bool write_log_file(const std::string&              path,
                                         const std::vector<RecordEntry>& entries,
                                         uint64_t                        start_time_ns = 0) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) return false;

    FlightLogFileHeader file_header{};
    file_header.start_time_ns = start_time_ns;
    const auto* fh_bytes      = reinterpret_cast<const char*>(&file_header);
    out.write(fh_bytes, sizeof(file_header));

    for (const auto& entry : entries) {
        const auto* rh_bytes = reinterpret_cast<const char*>(&entry.header);
        out.write(rh_bytes, sizeof(RecordHeader));
        out.write(entry.topic_name.data(), static_cast<std::streamsize>(entry.topic_name.size()));
        out.write(reinterpret_cast<const char*>(entry.payload.data()),
                  static_cast<std::streamsize>(entry.payload.size()));
    }

    return out.good();
}

/// Read all record entries from a log file.
/// Returns empty vector on failure.
struct ReadLogResult {
    FlightLogFileHeader      file_header{};
    std::vector<RecordEntry> entries;
    bool                     success = false;
};

[[nodiscard]] inline ReadLogResult read_log_file(const std::string& path) {
    ReadLogResult result;
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return result;

    // Read file header
    auto* fh_dst = reinterpret_cast<char*>(&result.file_header);
    in.read(fh_dst, sizeof(FlightLogFileHeader));
    if (!in.good() || result.file_header.magic != kFlightLogMagic) return result;

    // Read entries
    while (in.good() && in.peek() != std::char_traits<char>::eof()) {
        RecordEntry entry;
        auto*       rh_dst = reinterpret_cast<char*>(&entry.header);
        in.read(rh_dst, sizeof(RecordHeader));
        if (!in.good()) break;

        // Validate wire header
        if (entry.header.wire_header.magic != drone::ipc::kWireMagic) break;

        // Read topic name
        const auto topic_len = entry.header.topic_name_len;
        if (topic_len > 256) break;  // sanity limit
        entry.topic_name.resize(topic_len);
        in.read(entry.topic_name.data(), topic_len);
        if (!in.good()) break;

        // Read payload
        const auto payload_size = entry.header.wire_header.payload_size;
        if (payload_size > 64u * 1024u * 1024u) break;  // 64 MB sanity limit
        entry.payload.resize(payload_size);
        in.read(reinterpret_cast<char*>(entry.payload.data()),
                static_cast<std::streamsize>(payload_size));
        if (!in.good() && !in.eof()) break;
        // Trim to actual bytes read on partial read (truncated file)
        const auto actual = static_cast<std::size_t>(in.gcount());
        if (actual < payload_size) {
            entry.payload.resize(actual);
            entry.header.wire_header.payload_size = static_cast<uint32_t>(actual);
        }

        result.entries.push_back(std::move(entry));
    }

    result.success = true;
    return result;
}

/// Flight data recorder — subscribes to IPC topics and records traffic
/// to an in-memory ring buffer, then flushes to disk.
///
/// Thread-safe: all public methods are protected by a mutex.
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
    /// Reads: recorder.max_size_mb, recorder.output_dir
    template<typename ConfigT>
    explicit FlightRecorder(const ConfigT& cfg)
        : ring_buffer_(static_cast<std::size_t>(cfg.template get<int>("recorder.max_size_mb", 64)) *
                       1024 * 1024)
        , output_dir_(cfg.template get<std::string>("recorder.output_dir", "/tmp/flight_logs")) {}

    /// Record a message from a given topic.
    /// @tparam T  Trivially copyable IPC struct.
    template<typename T>
    void record(const std::string& topic, drone::ipc::WireMessageType msg_type, const T& msg,
                uint32_t seq = 0) {
        static_assert(std::is_trivially_copyable_v<T>, "record() requires trivially copyable T");
        if (!recording_.load(std::memory_order_acquire)) return;

        RecordEntry entry;
        entry.header.wire_header.magic        = drone::ipc::kWireMagic;
        entry.header.wire_header.version      = drone::ipc::kWireVersion;
        entry.header.wire_header.msg_type     = msg_type;
        entry.header.wire_header.payload_size = static_cast<uint32_t>(sizeof(T));
        entry.header.wire_header.sequence     = seq;
        entry.header.wire_header.timestamp_ns =
            static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        entry.header.topic_name_len =
            static_cast<uint16_t>(std::min(topic.size(), std::size_t{256}));

        entry.topic_name = topic.substr(0, entry.header.topic_name_len);
        entry.payload.resize(sizeof(T));
        const auto* src = reinterpret_cast<const uint8_t*>(&msg);
        std::copy(src, src + sizeof(T), entry.payload.data());

        std::lock_guard<std::mutex> lock(mutex_);
        ring_buffer_.push(std::move(entry));
    }

    /// Start recording.
    void start() { recording_.store(true, std::memory_order_release); }

    /// Stop recording.
    void stop() { recording_.store(false, std::memory_order_release); }

    /// Whether recording is active.
    [[nodiscard]] bool is_recording() const { return recording_.load(std::memory_order_acquire); }

    /// Flush the ring buffer to a timestamped log file.
    /// Returns the path of the written file, or empty string on failure.
    [[nodiscard]] std::string flush() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (ring_buffer_.size() == 0) return "";

        const auto now   = std::chrono::system_clock::now();
        const auto epoch = now.time_since_epoch();
        const auto secs  = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();

        const std::string filename = output_dir_ + "/flight_" + std::to_string(secs) + ".flog";

        const auto start_ns = ring_buffer_.entries().empty()
                                  ? 0ULL
                                  : ring_buffer_.entries().front().header.wire_header.timestamp_ns;

        if (!write_log_file(filename, ring_buffer_.entries(), start_ns)) {
            return "";
        }

        ring_buffer_.clear();
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
    mutable std::mutex mutex_;
    std::atomic<bool>  recording_{false};
    RecordRingBuffer   ring_buffer_;
    std::string        output_dir_;
};

}  // namespace drone::recorder
