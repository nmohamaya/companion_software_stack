// common/util/include/util/sys_info_factory.h
// Factory for ISysInfo — creates platform-specific implementation.
// Include this header where you need to create an ISysInfo instance.
#pragma once

#include "util/ilogger.h"
#include "util/isys_info.h"
#include "util/jetson_sys_info.h"
#include "util/linux_sys_info.h"

#include <memory>
#include <string>

// MockSysInfo is only needed in test builds — production binaries
// should not pull in mock implementations.
#ifdef DRONE_ENABLE_MOCK
#include "util/mock_sys_info.h"
#endif

namespace drone::util {

/// Create a platform-specific ISysInfo implementation.
/// @param platform  One of "linux", "jetson", or "mock" (mock requires DRONE_ENABLE_MOCK).
///                  Unknown values log a warning and fall back to LinuxSysInfo.
/// @return Owning pointer to ISysInfo implementation.
inline std::unique_ptr<ISysInfo> create_sys_info(const std::string& platform) {
    if (platform == "linux") {
        return std::make_unique<LinuxSysInfo>();
    }
    if (platform == "jetson") {
        return std::make_unique<JetsonSysInfo>();
    }
#ifdef DRONE_ENABLE_MOCK
    if (platform == "mock") {
        return std::make_unique<MockSysInfo>();
    }
#endif
    DRONE_LOG_WARN("[SysInfoFactory] Unknown platform '{}' — falling back to LinuxSysInfo",
                   platform);
    return std::make_unique<LinuxSysInfo>();
}

}  // namespace drone::util
