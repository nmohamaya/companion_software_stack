// common/util/include/util/config.h
// JSON configuration system for all processes.
// Loads a JSON file and provides typed access to keys with defaults.
#pragma once
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace drone {

class Config {
public:
    /// Load configuration from a JSON file.
    /// Returns false if the file cannot be read; the config stays empty (all defaults).
    bool load(const std::string& path) {
        path_ = path;
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            spdlog::warn("Config file not found: {} — using defaults", path);
            data_ = nlohmann::json::object();
            return false;
        }
        try {
            data_ = nlohmann::json::parse(ifs);
            spdlog::info("Config loaded from: {}", path);
            return true;
        } catch (const nlohmann::json::parse_error& e) {
            spdlog::error("Config parse error in {}: {}", path, e.what());
            data_ = nlohmann::json::object();
            return false;
        }
    }

    /// Get a value by dot-separated key path (e.g. "video.width") with a default.
    template <typename T>
    T get(const std::string& key, const T& default_val) const {
        try {
            const auto* node = &data_;
            // Walk the dot-separated path
            size_t start = 0;
            while (start < key.size()) {
                size_t dot = key.find('.', start);
                std::string part = key.substr(start, dot - start);
                if (!node->contains(part)) return default_val;
                node = &(*node)[part];
                if (dot == std::string::npos) break;
                start = dot + 1;
            }
            return node->get<T>();
        } catch (...) {
            return default_val;
        }
    }

    /// Check if a key exists.
    bool has(const std::string& key) const {
        const auto* node = &data_;
        size_t start = 0;
        while (start < key.size()) {
            size_t dot = key.find('.', start);
            std::string part = key.substr(start, dot - start);
            if (!node->contains(part)) return false;
            node = &(*node)[part];
            if (dot == std::string::npos) break;
            start = dot + 1;
        }
        return true;
    }

    /// Get the raw JSON object for a section.
    nlohmann::json section(const std::string& key) const {
        try {
            const auto* node = &data_;
            size_t start = 0;
            while (start < key.size()) {
                size_t dot = key.find('.', start);
                std::string part = key.substr(start, dot - start);
                if (!node->contains(part)) return nlohmann::json::object();
                node = &(*node)[part];
                if (dot == std::string::npos) break;
                start = dot + 1;
            }
            return *node;
        } catch (...) {
            return nlohmann::json::object();
        }
    }

    /// Get the file path that was loaded.
    const std::string& path() const { return path_; }

    /// Check if config was successfully loaded.
    bool loaded() const { return !data_.empty(); }

    /// Get a reference to the raw JSON data.
    const nlohmann::json& raw() const { return data_; }

private:
    nlohmann::json data_ = nlohmann::json::object();
    std::string path_;
};

} // namespace drone
