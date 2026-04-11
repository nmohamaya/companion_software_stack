// common/util/include/util/sys_info_factory.h
// Factory for ISysInfo — creates platform-specific implementation.
// Include this header where you need to create an ISysInfo instance.
#pragma once

#include "util/isys_info.h"
#include "util/jetson_sys_info.h"
#include "util/linux_sys_info.h"
#include "util/mock_sys_info.h"

#include <memory>
#include <string>

namespace drone::util {

inline std::unique_ptr<ISysInfo> create_sys_info(const std::string& platform) {
    if (platform == "linux") {
        return std::make_unique<LinuxSysInfo>();
    }
    if (platform == "jetson") {
        return std::make_unique<JetsonSysInfo>();
    }
    if (platform == "mock") {
        return std::make_unique<MockSysInfo>();
    }
    // Default to Linux for unknown platforms
    return std::make_unique<LinuxSysInfo>();
}

}  // namespace drone::util
