// common/ipc/include/ipc/shm_reader.h
// SeqLock-based shared memory reader template — consumer side.
#pragma once
#include <atomic>
#include <cstring>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <type_traits>

template <typename T>
class ShmReader {
    static_assert(std::is_trivially_copyable_v<T>,
                  "SHM payload must be trivially copyable");
public:
    struct ShmBlock {
        std::atomic<uint64_t> seq;
        uint64_t timestamp_ns;
        T data;
    };

    bool open(const std::string& name) {
        name_ = name;
        fd_ = shm_open(name.c_str(), O_RDONLY, 0);
        if (fd_ < 0) return false;
        ptr_ = static_cast<const ShmBlock*>(
            mmap(nullptr, sizeof(ShmBlock), PROT_READ, MAP_SHARED, fd_, 0));
        return ptr_ != MAP_FAILED;
    }

    // Read with SeqLock retry
    bool read(T& out, uint64_t* timestamp_ns = nullptr) const {
        if (!ptr_) return false;
        for (int attempt = 0; attempt < 4; ++attempt) {
            uint64_t s1 = ptr_->seq.load(std::memory_order_acquire);
            if (s1 & 1) continue;  // writer is mid-write
            std::memcpy(&out, &ptr_->data, sizeof(T));
            if (timestamp_ns) *timestamp_ns = ptr_->timestamp_ns;
            std::atomic_thread_fence(std::memory_order_acquire);
            uint64_t s2 = ptr_->seq.load(std::memory_order_relaxed);
            if (s1 == s2) return true;
        }
        return false;  // torn read after 4 attempts
    }

    bool is_open() const { return ptr_ != nullptr && ptr_ != MAP_FAILED; }

    ~ShmReader() {
        if (ptr_ && ptr_ != MAP_FAILED)
            munmap(const_cast<ShmBlock*>(ptr_), sizeof(ShmBlock));
        if (fd_ >= 0) close(fd_);
    }

    ShmReader(const ShmReader&) = delete;
    ShmReader& operator=(const ShmReader&) = delete;

    ShmReader(ShmReader&& other) noexcept
        : fd_(other.fd_), ptr_(other.ptr_), name_(std::move(other.name_))
    {
        other.fd_ = -1;
        other.ptr_ = nullptr;
    }

    ShmReader& operator=(ShmReader&& other) noexcept {
        if (this != &other) {
            if (ptr_ && ptr_ != MAP_FAILED)
                munmap(const_cast<ShmBlock*>(ptr_), sizeof(ShmBlock));
            if (fd_ >= 0) close(fd_);
            fd_ = other.fd_;
            ptr_ = other.ptr_;
            name_ = std::move(other.name_);
            other.fd_ = -1;
            other.ptr_ = nullptr;
        }
        return *this;
    }

    ShmReader() = default;

private:
    int fd_ = -1;
    const ShmBlock* ptr_ = nullptr;
    std::string name_;
};
