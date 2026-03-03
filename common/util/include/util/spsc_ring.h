// common/util/include/util/spsc_ring.h
// Lock-free Single-Producer Single-Consumer ring buffer.
// Used for intra-process thread communication.
#pragma once
#include <array>
#include <atomic>
#include <optional>

namespace drone {

template<typename T, size_t N>
class SPSCRing {
    static_assert((N & (N - 1)) == 0, "N must be power of 2");

public:
    bool try_push(const T& item) {
        const uint64_t w = write_idx_.load(std::memory_order_relaxed);
        const uint64_t r = read_idx_.load(std::memory_order_acquire);
        if (w - r >= N) return false;  // ring full
        slots_[w & (N - 1)] = item;
        write_idx_.store(w + 1, std::memory_order_release);
        return true;
    }

    std::optional<T> try_pop() {
        const uint64_t r = read_idx_.load(std::memory_order_relaxed);
        const uint64_t w = write_idx_.load(std::memory_order_acquire);
        if (r >= w) return std::nullopt;  // ring empty
        T item = slots_[r & (N - 1)];
        read_idx_.store(r + 1, std::memory_order_release);
        return item;
    }

    uint64_t available() const {
        return write_idx_.load(std::memory_order_acquire) -
               read_idx_.load(std::memory_order_relaxed);
    }

private:
    alignas(64) std::atomic<uint64_t> write_idx_{0};
    alignas(64) std::atomic<uint64_t> read_idx_{0};
    std::array<T, N> slots_{};
};

}  // namespace drone
