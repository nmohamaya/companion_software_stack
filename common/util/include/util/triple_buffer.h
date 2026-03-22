// common/util/include/util/triple_buffer.h
// Lock-free triple buffer for single-producer / single-consumer latest-value handoff.
// The producer always has a free slot to write to (never blocks, never fails).
// The consumer always reads the most recent value (never stale data).
// Safe for non-trivially-copyable types — each slot is exclusively owned by one thread.
#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <optional>

namespace drone {

/// Lock-free triple buffer.
///
/// Three pre-allocated slots rotate through three roles:
///   WRITE  — producer writes here (exclusive to producer)
///   LATEST — handoff point (swapped atomically)
///   READ   — consumer reads here (exclusive to consumer)
///
/// Producer: write() stores a value in WRITE, then CAS-swaps WRITE↔LATEST.
/// Consumer: read()  CAS-swaps LATEST↔READ, then returns the READ slot.
///
/// The new-data flag is packed into the high bit of latest_idx_ so that the
/// index swap and the flag are atomically coupled in a single CAS — this
/// prevents a race where the consumer sees a stale flag from a prior write
/// after its own CAS has already rotated latest to a stale slot.
///
/// Memory ordering: acquire/release on CAS ensures the payload written by the
/// producer is visible to the consumer after the index swap completes.
template<typename T>
class TripleBuffer {
public:
    /// Producer: store a new value.  Always succeeds, never blocks.
    /// If the consumer has not read the previous value, it is overwritten.
    void write(T value) {
        const auto w = write_idx_.load(std::memory_order_relaxed);
        slots_[w]    = std::move(value);

        // Swap write ↔ latest, atomically setting the NEW_DATA flag.
        auto    expected = latest_idx_.load(std::memory_order_relaxed);
        uint8_t desired  = w | kNewDataBit;
        while (!latest_idx_.compare_exchange_weak(expected, desired, std::memory_order_acq_rel,
                                                  std::memory_order_relaxed)) {
            // On retry, desired stays the same (w | NEW_DATA).
        }
        write_idx_.store(expected & kIndexMask, std::memory_order_release);

        write_count_.fetch_add(1, std::memory_order_relaxed);
    }

    /// Consumer: read the latest value if new data is available.
    /// Returns std::nullopt if the producer has not written since the last read.
    /// Never blocks.
    [[nodiscard]] std::optional<T> read() {
        auto expected = latest_idx_.load(std::memory_order_acquire);

        // No new data — the NEW_DATA bit is clear.
        if (!(expected & kNewDataBit)) {
            return std::nullopt;
        }

        // Swap latest ↔ read, atomically clearing the NEW_DATA flag.
        auto    r       = read_idx_.load(std::memory_order_relaxed);
        uint8_t desired = r;  // no NEW_DATA bit — clears the flag
        while (!latest_idx_.compare_exchange_weak(expected, desired, std::memory_order_acq_rel,
                                                  std::memory_order_relaxed)) {
            if (!(expected & kNewDataBit)) {
                // Lost the race — producer hasn't written new data since
                // another consumer CAS or the flag was cleared.
                return std::nullopt;
            }
            // Retry with updated expected; desired stays r (no NEW_DATA bit).
        }
        uint8_t got_idx = expected & kIndexMask;
        read_idx_.store(got_idx, std::memory_order_release);

        read_count_.fetch_add(1, std::memory_order_relaxed);
        return std::move(slots_[got_idx]);
    }

    /// Number of write() calls (monotonically increasing).
    [[nodiscard]] uint64_t write_count() const {
        return write_count_.load(std::memory_order_relaxed);
    }

    /// Number of successful read() calls (monotonically increasing).
    [[nodiscard]] uint64_t read_count() const {
        return read_count_.load(std::memory_order_relaxed);
    }

private:
    static constexpr uint8_t kNewDataBit = 0x80;
    static constexpr uint8_t kIndexMask  = 0x03;

    std::array<T, 3> slots_{};

    // Cache-line aligned to prevent false sharing between producer and consumer.
    alignas(64) std::atomic<uint8_t> write_idx_{0};
    alignas(64) std::atomic<uint8_t> latest_idx_{1};  // bit 7 = new_data flag
    alignas(64) std::atomic<uint8_t> read_idx_{2};

    std::atomic<uint64_t> write_count_{0};
    std::atomic<uint64_t> read_count_{0};
};

}  // namespace drone
