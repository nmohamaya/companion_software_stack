// tests/test_helpers.h
// Shared test utilities — temp config file creation, cleanup, sanitizer detection.
// Include this instead of duplicating the create_temp_config pattern, and
// gate performance-budget tests on `is_sanitizer_build()` to avoid spurious
// failures under ASan / TSan / UBSan instrumentation overhead.
#pragma once

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

// ─── Sanitizer detection ────────────────────────────────────────────────────
//
// Returns true if the binary was built with any of: ASan / TSan / UBSan.
//
// Use to skip performance-budget tests that don't tolerate the sanitizer
// instrumentation overhead. Correctness tests should NOT use this guard.
//
// Clang: __has_feature is reliable for all three.
// GCC: __SANITIZE_ADDRESS__ / __SANITIZE_THREAD__ are reliable. GCC does
// NOT define a portable macro for UBSan, so UBSan-only GCC builds fall
// through to false here — but the project's CI uses Clang for sanitizer
// builds (see deploy/build.sh --asan/--tsan/--ubsan flags), so the Clang
// path covers the real cases.
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || \
    __has_feature(undefined_behavior_sanitizer)
#define DRONE_TEST_SANITIZER_BUILD 1
#endif
#endif
#if !defined(DRONE_TEST_SANITIZER_BUILD)
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define DRONE_TEST_SANITIZER_BUILD 1
#endif
#endif
#if !defined(DRONE_TEST_SANITIZER_BUILD)
#define DRONE_TEST_SANITIZER_BUILD 0
#endif

namespace drone::test {

/// True iff the binary was built with ASan / TSan / UBSan instrumentation.
/// Performance-budget tests should `GTEST_SKIP()` when this is true — the
/// sanitizer overhead defeats the budget without invalidating the tested
/// behaviour. Correctness tests must NOT skip on this; they should run on
/// every build.
constexpr bool is_sanitizer_build() {
    return DRONE_TEST_SANITIZER_BUILD == 1;
}

inline std::vector<std::string>& temp_files() {
    static std::vector<std::string> files;
    return files;
}

inline std::string create_temp_config(const std::string& json_content,
                                      const std::string& prefix = "test_cfg") {
    std::string       tmpl_str = "/tmp/" + prefix + "_XXXXXX.json";
    std::vector<char> tmpl(tmpl_str.begin(), tmpl_str.end());
    tmpl.push_back('\0');

    int fd = mkstemps(tmpl.data(), 5);
    if (fd < 0) {
        std::string   path = "/tmp/" + prefix + "_" + std::to_string(getpid()) + ".json";
        std::ofstream ofs(path);
        ofs << json_content;
        temp_files().push_back(path);
        return path;
    }
    ::close(fd);
    std::string   path(tmpl.data());
    std::ofstream ofs(path);
    ofs << json_content;
    temp_files().push_back(path);
    return path;
}

struct TempFileCleanup {
    ~TempFileCleanup() {
        for (auto& f : temp_files()) std::remove(f.c_str());
    }
};

}  // namespace drone::test
