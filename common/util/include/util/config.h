// common/util/include/util/config.h
// JSON configuration system for all processes.
// Loads a JSON file and provides typed access to keys with defaults.
#pragma once
#include "util/result.h"

#include <fstream>
#include <string>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace drone {

class Config {
public:
    /// Load configuration from a JSON file.
    /// Returns false if the file cannot be read; the config stays empty (all defaults).
    [[nodiscard]] bool load(const std::string& path) {
        auto r = load_config(path);
        return r.is_ok();
    }

    /// Load configuration from a JSON file (Result-based API).
    /// Returns Result<void> on success, or an Error with details.
    [[nodiscard]] util::VoidResult load_config(const std::string& path) {
        path_ = path;
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            spdlog::warn("Config file not found: {} — using defaults", path);
            data_ = nlohmann::json::object();
            return util::VoidResult::err(
                util::Error(util::ErrorCode::FileNotFound, "Config file not found: " + path));
        }
        try {
            data_ = nlohmann::json::parse(ifs);
            spdlog::info("Config loaded from: {}", path);
            return util::VoidResult::ok();
        } catch (const nlohmann::json::parse_error& e) {
            spdlog::error("Config parse error in {}: {}", path, e.what());
            data_ = nlohmann::json::object();
            return util::VoidResult::err(
                util::Error(util::ErrorCode::ParseError,
                            std::string("Config parse error in ") + path + ": " + e.what()));
        }
    }

    /// Get a value by dot-separated key path (e.g. "video.width") with a default.
    template<typename T>
    [[nodiscard]] T get(const std::string& key, const T& default_val) const {
        try {
            const auto* node = &data_;
            // Walk the dot-separated path
            size_t start = 0;
            while (start < key.size()) {
                size_t      dot  = key.find('.', start);
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

    /// Get a required value — returns Result<T> instead of silently falling
    /// back to a default.  Use this for values that must be present.
    template<typename T>
    [[nodiscard]] util::Result<T> require(const std::string& key) const {
        try {
            const auto* node  = &data_;
            size_t      start = 0;
            while (start < key.size()) {
                size_t      dot  = key.find('.', start);
                std::string part = key.substr(start, dot - start);
                if (!node->contains(part)) {
                    return util::Result<T>::err(
                        util::Error(util::ErrorCode::MissingKey, "Missing config key: " + key));
                }
                node = &(*node)[part];
                if (dot == std::string::npos) break;
                start = dot + 1;
            }
            return util::Result<T>::ok(node->get<T>());
        } catch (const nlohmann::json::type_error&) {
            return util::Result<T>::err(
                util::Error(util::ErrorCode::TypeMismatch, "Type mismatch for config key: " + key));
        } catch (const std::exception& e) {
            return util::Result<T>::err(
                util::Error(util::ErrorCode::InvalidValue,
                            std::string("Error reading config key '") + key + "': " + e.what()));
        }
    }

    /// Check if a key exists.
    [[nodiscard]] bool has(const std::string& key) const {
        const auto* node  = &data_;
        size_t      start = 0;
        while (start < key.size()) {
            size_t      dot  = key.find('.', start);
            std::string part = key.substr(start, dot - start);
            if (!node->contains(part)) return false;
            node = &(*node)[part];
            if (dot == std::string::npos) break;
            start = dot + 1;
        }
        return true;
    }

    /// Get the raw JSON object for a section.
    [[nodiscard]] nlohmann::json section(const std::string& key) const {
        try {
            const auto* node  = &data_;
            size_t      start = 0;
            while (start < key.size()) {
                size_t      dot  = key.find('.', start);
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
    [[nodiscard]] const std::string& path() const { return path_; }

    /// Check if config was successfully loaded.
    [[nodiscard]] bool loaded() const { return !data_.empty(); }

    /// Get a reference to the raw JSON data.
    [[nodiscard]] const nlohmann::json& raw() const { return data_; }

private:
    nlohmann::json data_ = nlohmann::json::object();
    std::string    path_;
};

}  // namespace drone
