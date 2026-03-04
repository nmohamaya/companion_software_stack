// common/util/include/util/safe_name_copy.h
// Centralised, warning-safe string-to-fixed-buffer copy for SHM names.
//
// Wraps strncpy with targeted GCC pragma suppression so call sites don't
// need manual length arithmetic or scattered #pragma blocks.  All name
// buffers in the codebase (thread names, process names, topic names)
// should use this function instead of raw strncpy/snprintf/memcpy.
#pragma once

#include <cstring>

namespace drone::util {

/// Copy a null-terminated C string into a fixed-size char buffer,
/// truncating if necessary and always null-terminating.
///
/// @tparam N   Size of the destination buffer (deduced from the array).
/// @param dst  Destination char array — will be fully zeroed, then filled.
/// @param src  Source null-terminated string.
///
/// Usage:
///   char name[32];
///   safe_name_copy(name, "my_thread_name");
///
/// Why this exists:
///   GCC's -Wstringop-truncation (-Wall) and -Wformat-truncation (-Wextra)
///   fire when strncpy/snprintf intentionally truncate — which is by design
///   for our fixed-size SHM buffers.  This function uses a targeted pragma
///   to suppress the warning in one audited location rather than scattering
///   suppressions or using error-prone memcpy + manual length clamps.
template<std::size_t N>
inline void safe_name_copy(char (&dst)[N], const char* src) {
    static_assert(N > 0, "Destination buffer must be non-empty");
    std::memset(dst, 0, N);
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif
    std::strncpy(dst, src, N - 1);
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
    dst[N - 1] = '\0';  // Belt-and-suspenders — strncpy(N-1) + memset already guarantees this
}

}  // namespace drone::util
