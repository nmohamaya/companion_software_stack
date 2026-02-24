// common/ipc/include/ipc/shm_writer.h
// SeqLock-based shared memory writer template.
// All 7 processes use ShmWriter<T> / ShmReader<T> for zero-copy IPC.
#pragma once
#include <atomic>
#include <cstring>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <type_traits>
#include <chrono>

template <typename T>
class ShmWriter {
    static_assert(std::is_trivially_copyable_v<T>,
                  "SHM payload must be trivially copyable");
public:
    struct ShmBlock {
        std::atomic<uint64_t> seq{0};
        uint64_t timestamp_ns{0};
        T data;
    };

    bool create(const std::string& name) {
        name_ = name;
        fd_ = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd_ < 0) return false;
        if (ftruncate(fd_, sizeof(ShmBlock)) < 0) return false;
        ptr_ = static_cast<ShmBlock*>(
            mmap(nullptr, sizeof(ShmBlock),
                 PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));
        if (ptr_ == MAP_FAILED) { ptr_ = nullptr; return false; }
        ptr_->seq.store(0, std::memory_order_relaxed);
        return true;
    }

    void write(const T& data) {
        if (!ptr_) return;
        uint64_t s = ptr_->seq.load(std::memory_order_relaxed);
        ptr_->seq.store(s + 1, std::memory_order_release);  // odd = writing
        ptr_->timestamp_ns = now_ns();
        std::memcpy(&ptr_->data, &data, sizeof(T));
        ptr_->seq.store(s + 2, std::memory_order_release);  // even = done
    }

    ~ShmWriter() {
        if (ptr_) munmap(ptr_, sizeof(ShmBlock));
        if (fd_ >= 0) {
            shm_unlink(name_.c_str());
            close(fd_);
        }
    }

    ShmWriter(const ShmWriter&) = delete;
    ShmWriter& operator=(const ShmWriter&) = delete;

    ShmWriter(ShmWriter&& other) noexcept
        : fd_(other.fd_), ptr_(other.ptr_), name_(std::move(other.name_))
    {
        other.fd_ = -1;
        other.ptr_ = nullptr;
    }

    ShmWriter& operator=(ShmWriter&& other) noexcept {
        if (this != &other) {
            if (ptr_) munmap(ptr_, sizeof(ShmBlock));
            if (fd_ >= 0) { shm_unlink(name_.c_str()); close(fd_); }
            fd_ = other.fd_;
            ptr_ = other.ptr_;
            name_ = std::move(other.name_);
            other.fd_ = -1;
            other.ptr_ = nullptr;
        }
        return *this;
    }

    ShmWriter() = default;

private:
    int fd_ = -1;
    ShmBlock* ptr_ = nullptr;
    std::string name_;

    static uint64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
};
