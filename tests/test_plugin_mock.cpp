// tests/test_plugin_mock.cpp
// Mock plugin .so for testing PluginLoader.
// Compiled as a shared library (test_plugin_mock.so) by CMake when ENABLE_PLUGINS=ON.
//
// Provides a minimal ICamera implementation that can be loaded at runtime
// via dlopen/dlsym.

#include "hal/icamera.h"

#include <cstdint>
#include <string>
#include <vector>

namespace {

class MockPluginCamera final : public drone::hal::ICamera {
public:
    static constexpr uint32_t kWidth    = 640;
    static constexpr uint32_t kHeight   = 480;
    static constexpr uint32_t kChannels = 3;

    [[nodiscard]] bool open(uint32_t /*width*/, uint32_t /*height*/, int /*fps*/) override {
        opened_ = true;
        return true;
    }

    void close() override { opened_ = false; }

    [[nodiscard]] drone::hal::CapturedFrame capture() override {
        drone::hal::CapturedFrame frame;
        frame.width        = kWidth;
        frame.height       = kHeight;
        frame.channels     = kChannels;
        frame.stride       = kWidth * kChannels;
        frame.timestamp_ns = 12345;
        frame.sequence     = seq_++;
        frame.valid        = opened_;
        if (frame.valid) {
            frame.data = dummy_data_;  // copy — each valid frame owns its data
        }
        return frame;
    }

    [[nodiscard]] bool is_open() const override { return opened_; }

    [[nodiscard]] std::string name() const override { return "MockPluginCamera"; }

private:
    bool                 opened_ = false;
    uint64_t             seq_    = 0;
    std::vector<uint8_t> dummy_data_{std::vector<uint8_t>(kWidth * kHeight * kChannels, 128)};
};

}  // namespace

// The factory function: extern "C" with default visibility.
// PluginLoader::load<ICamera>() calls this to create an instance.
extern "C" __attribute__((visibility("default"))) drone::hal::ICamera* create_instance() {
    return new MockPluginCamera();  // NOLINT — ownership transferred to PluginHandle
}

// Second factory with a custom name, for testing custom factory_symbol.
extern "C" __attribute__((visibility("default"))) drone::hal::ICamera* create_camera_v2() {
    return new MockPluginCamera();  // NOLINT — ownership transferred to PluginHandle
}
