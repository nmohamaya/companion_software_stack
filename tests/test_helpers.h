// tests/test_helpers.h
// Shared test utilities — temp config file creation and cleanup.
// Include this instead of duplicating the create_temp_config pattern.
#pragma once

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

namespace drone::test {

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
