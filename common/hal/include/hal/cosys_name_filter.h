// common/hal/include/hal/cosys_name_filter.h
//
// Shared include/exclude-substring filter for the Cosys-AirSim HAL backends.
//
// All three Cosys ground-truth backends (Echo radar, GT radar, segmentation)
// support the same per-object filtering rules:
//
//   - `include_substrings` (allowlist): if non-empty, ONLY object names
//     containing one of these substrings are kept.  Use this when the
//     scenario knows exactly what to detect.  Empty (unknown) names are
//     always rejected — they cannot match an include.
//   - `exclude_substrings` (blocklist): when no allowlist is configured,
//     drop named background ("Ground", "Sky", etc.).  The drone's own
//     vehicle name is always added to the blocklist.
//
// Safe-by-default semantics for unknown names depend on the backend:
//
//   - Sparse-source backends (Echo, GT radar) treat unknown as KEEP — a
//     single un-named multipath bounce on a real obstacle should not kill
//     the whole detection.  Pass `default_unknown_action::Keep`.
//
//   - Dense-source backends (Segmentation: every pixel emits a "name") MUST
//     treat unknown as DROP.  A partial color map otherwise paints every
//     unmapped pixel as an obstacle — an unsafe-by-default failure mode on
//     a flight-critical promotion path.  Pass `CosysNameFilterUnknown::Drop`.
#pragma once

#include "util/config.h"

#include <string>
#include <string_view>
#include <vector>

namespace drone::hal {

enum class CosysNameFilterUnknown {
    /// Keep object even if its name is empty (unknown color / un-named return).
    /// Use for sparse sources where unknowns are the result of partial
    /// data, not unmapped categories.
    Keep,
    /// Drop object when its name is empty.  Use for dense sources where
    /// every pixel is a "named" object — an empty name means the color
    /// map is partial and we MUST refuse rather than treat as obstacle.
    Drop,
};

/// Parse a comma-separated list, trimming whitespace, dropping empty tokens.
/// Free function so callers don't have to instantiate a filter just to
/// turn a config string into a vector.
[[nodiscard]] inline std::vector<std::string> parse_csv_substrings(std::string_view raw) {
    std::vector<std::string> out;
    size_t                   start = 0;
    while (start < raw.size()) {
        const size_t comma = raw.find(',', start);
        const size_t end   = (comma == std::string_view::npos) ? raw.size() : comma;
        std::string  tok(raw.substr(start, end - start));
        while (!tok.empty() && (tok.front() == ' ' || tok.front() == '\t')) tok.erase(0, 1);
        while (!tok.empty() && (tok.back() == ' ' || tok.back() == '\t')) tok.pop_back();
        if (!tok.empty()) out.push_back(std::move(tok));
        if (comma == std::string_view::npos) break;
        start = comma + 1;
    }
    return out;
}

class CosysNameFilter {
public:
    /// Build from a Config and the per-section prefix.  Reads
    /// `<section>.include_substrings` (allowlist, empty = blocklist mode)
    /// and `<section>.exclude_substrings` (blocklist seed; vehicle name
    /// appended automatically).
    CosysNameFilter(const drone::Config& cfg, std::string_view section,
                    std::string_view default_excludes, std::string_view vehicle_name,
                    CosysNameFilterUnknown unknown_action)
        : unknown_action_(unknown_action) {
        const std::string s(section);
        include_ = parse_csv_substrings(cfg.get<std::string>(s + ".include_substrings", ""));
        exclude_ = parse_csv_substrings(
            cfg.get<std::string>(s + ".exclude_substrings", std::string(default_excludes)));
        if (!vehicle_name.empty()) exclude_.emplace_back(vehicle_name);
    }

    /// Direct construction from already-parsed lists (for tests + callers
    /// that want to assemble the lists themselves).
    CosysNameFilter(std::vector<std::string> include, std::vector<std::string> exclude,
                    CosysNameFilterUnknown unknown_action)
        : include_(std::move(include))
        , exclude_(std::move(exclude))
        , unknown_action_(unknown_action) {}

    /// True ⇒ skip (excluded).  See header docstring for the semantics.
    [[nodiscard]] bool is_excluded(std::string_view obj_name) const noexcept {
        if (!include_.empty()) {
            // Allowlist mode: empty name cannot match → always exclude
            // (independent of unknown_action_; allowlists are explicit).
            if (obj_name.empty()) return true;
            for (const auto& sub : include_) {
                if (!sub.empty() && obj_name.find(sub) != std::string_view::npos) return false;
            }
            return true;
        }
        // Blocklist mode.
        if (obj_name.empty()) {
            return unknown_action_ == CosysNameFilterUnknown::Drop;
        }
        for (const auto& sub : exclude_) {
            if (!sub.empty() && obj_name.find(sub) != std::string_view::npos) return true;
        }
        return false;
    }

    [[nodiscard]] bool   has_allowlist() const noexcept { return !include_.empty(); }
    [[nodiscard]] size_t include_size() const noexcept { return include_.size(); }
    [[nodiscard]] size_t exclude_size() const noexcept { return exclude_.size(); }
    [[nodiscard]] CosysNameFilterUnknown unknown_action() const noexcept {
        return unknown_action_;
    }

private:
    std::vector<std::string> include_;
    std::vector<std::string> exclude_;
    CosysNameFilterUnknown   unknown_action_;
};

}  // namespace drone::hal
