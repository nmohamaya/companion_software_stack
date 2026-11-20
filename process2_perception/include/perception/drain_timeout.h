// Deadline check shared by perception's tracker and fusion drain paths.
#pragma once

#include "util/iclock.h"

#include <chrono>

namespace drone::perception {

[[nodiscard]] inline bool drain_timed_out(std::chrono::steady_clock::time_point start,
                                          std::chrono::milliseconds             timeout) {
    return util::get_clock().now() - start >= timeout;
}

}  // namespace drone::perception
