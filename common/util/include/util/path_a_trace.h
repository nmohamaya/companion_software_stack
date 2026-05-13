// common/util/include/util/path_a_trace.h
//
// Diagnostic trace writer for the PATH A mask-projection pipeline
// (Epic #520, Issue #612).  Writes one JSONL line per published voxel
// batch so `tools/plot_voxel_trace.py` can visualise where voxels
// actually land in 3D versus the scene's ground-truth object positions.
//
// Line schema (compact for grep/plot):
//   {
//     "t_ns":     uint64,                      // batch emission time
//     "seq":      uint64,                      // source video-frame sequence
//     "pose_t":   [x, y, z],                   // SLAM translation (world-frame, m)
//     "pose_q":   [w, x, y, z],                // SLAM quaternion
//     "det_bb":   [[x, y, w, h, class, conf], ...],   // detector bboxes (px)
//     "sam_bb":   [[x, y, w, h, conf], ...],          // SAM mask bboxes (px)
//     "voxels":   [[wx, wy, wz, label, conf], ...]    // world-frame voxel positions
//   }
//
// Enabled via `perception.path_a.diag.trace_voxels`.  Default off — production
// builds pay zero cost (constructor returns with `enabled_=false`).
//
// Thread-safety: single-writer (only `mask_projection_thread` calls
// `record_batch`).  The constructor opens the file once; writes are
// buffered by `std::ofstream`.  No locking needed.
//
// Path confinement: `trace_path` is config-controlled; the constructor
// accepts relative paths that don't escape the tree via `..`, and
// absolute paths that contain a `drone_logs` segment somewhere in their
// normalised form (the project convention for every diagnostic artifact).
// Rejects paths that escape via `..` or absolute paths without a
// `drone_logs` segment (e.g. `/etc/cron.d/...`, `/tmp/evil.jsonl`).
// Scenario runners rewrite this to an absolute
// `<scenario_log_dir>/path_a_voxel_trace.jsonl` before process launch.
//
// Flight-critical constraint (DR-026, docs/guides/DESIGN_RATIONALE.md):
// `record_batch()` performs `std::ofstream` I/O from the caller thread
// (`mask_projection_thread`).  The CPP_PATTERNS_GUIDE observability rule
// prohibits mutex-backed observability on flight-critical threads; the
// exception documented in DR-026 allows this diagnostic writer because
// (a) it is opt-in with default `trace_voxels=false`, (b) it is never
// enabled in production configs, and (c) the `enabled_` branch short-
// circuits to a single `bool` load in the hot path.
//
// File format: JSON lines (RFC 7464 loose interpretation) — each line is an
// independent JSON object, parseable with one `json::parse()` per line.
// Python plotter example:
//     with open(path) as f:
//         for line in f:
//             rec = json.loads(line)
//             ...
//
// Run-size estimate: ~0.5-2 KB per batch at 30 Hz = ~50 KB/s = ~3 MB/min.
// Truncated automatically on open (one trace per run).

#pragma once

#include "hal/iinference_backend.h"
#include "hal/isemantic_projector.h"
#include "ipc/ipc_types.h"
#include "util/ilogger.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace drone::util {

class PathATrace {
public:
    /// Constructor.  If `enabled` is false, the path cannot be opened, or the
    /// path fails confinement (see `is_path_confined`), the writer is inert —
    /// `record_batch()` becomes a no-op.
    PathATrace(bool enabled, const std::string& path) : enabled_(enabled), path_(path) {
        if (!enabled_) return;
        if (!is_path_confined(path)) {
            DRONE_LOG_ERROR(
                "[PathATrace] Refusing trace path '{}' — must stay under drone_logs/ or a "
                "project-root-relative subdirectory; no absolute or ../-escaping paths",
                path);
            enabled_ = false;
            return;
        }
        try {
            std::filesystem::create_directories(std::filesystem::path(path).parent_path());
        } catch (const std::exception& e) {
            DRONE_LOG_WARN("[PathATrace] Could not create parent dir for '{}': {}", path, e.what());
        }
        out_.open(path, std::ios::out | std::ios::trunc);
        if (!out_.is_open()) {
            DRONE_LOG_ERROR("[PathATrace] Cannot open '{}' for write — trace disabled", path);
            enabled_ = false;
            return;
        }
        DRONE_LOG_INFO("[PathATrace] Writing voxel trace to '{}'", path);
    }

    /// Confine trace paths to the project tree.  Accepts:
    ///   - relative paths whose lexically-normalised form does not start
    ///     with `..` (the typical CI / manual case, e.g.
    ///     `drone_logs/path_a_voxel_trace.jsonl`).
    ///   - absolute paths that contain a `drone_logs` segment somewhere
    ///     in their normalised form.  This handles the scenario runners,
    ///     which rewrite `trace_path` to an absolute
    ///     `<scenario_log_dir>/path_a_voxel_trace.jsonl` before launching
    ///     each process so the run directory is self-contained regardless
    ///     of the process's working directory.  A `drone_logs` segment
    ///     requirement rejects accidental writes to `/tmp`, `/etc`, or the
    ///     user's home — the project convention is that every diagnostic
    ///     artifact lives under a `drone_logs` tree.
    /// Rejects:
    ///   - absolute paths that have no `drone_logs` segment (e.g.
    ///     `/etc/cron.d/evil.jsonl`, `/tmp/absolute_escape.jsonl`).
    ///   - relative paths that normalise to an escape (`../../etc/foo`).
    ///
    /// Trade-off (PR #623 review-memory-safety P2): this is weaker than
    /// "reject all absolute paths".  The check uses *exact* segment
    /// equality (`segment == "drone_logs"`), so `/etc/drone_logs_fake/x`
    /// or `/var/drone_logsx/y` are still rejected — but any directory
    /// literally named `drone_logs` anywhere on the filesystem will pass.
    /// Risk is bounded because (a) the writer only opens for write (no
    /// read of secrets), (b) trace content is JSONL telemetry the
    /// caller already controls, and (c) production builds default to
    /// `trace_voxels=false` so the writer is inert.  Symlink resolution
    /// is intentionally *not* performed: `lexically_normal()` only
    /// collapses `.` and `..` textually, leaving symlink targets to the
    /// OS at write time.  See DR-026 for the full analysis.
    static bool is_path_confined(const std::string& path) {
        if (path.empty()) return false;
        std::filesystem::path p(path);
        const auto            normalised = p.lexically_normal();
        const auto            first      = normalised.begin();
        if (first == normalised.end()) return false;

        if (p.is_absolute()) {
            for (const auto& segment : normalised) {
                if (segment == "drone_logs") return true;
            }
            return false;
        }

        if (*first == "..") return false;
        return true;
    }

    ~PathATrace() {
        if (out_.is_open()) {
            out_.flush();
            out_.close();
            DRONE_LOG_INFO("[PathATrace] Closed '{}' — {} lines written", path_, lines_written_);
        }
    }

    PathATrace(const PathATrace&)            = delete;
    PathATrace& operator=(const PathATrace&) = delete;

    [[nodiscard]] bool enabled() const { return enabled_; }

    /// Record one published batch.
    ///
    /// **Thread-safety:** NOT thread-safe.  Must be called from a single thread
    /// only — the thread that owns the `PathATrace` instance.  The class holds
    /// no mutex; concurrent callers would race on `out_` and `lines_written_`.
    ///
    /// Expects:
    ///   @param t_ns    Batch emission time (source-frame timestamp)
    ///   @param seq     Source video-frame sequence number
    ///   @param pose    SLAM pose used for this projection
    ///   @param dets    Detector bboxes fed into MaskDepthProjector
    ///   @param masks   SAM mask bboxes fed into MaskDepthProjector
    ///   @param voxels  Resulting world-frame VoxelUpdates (pre-conversion-to-wire)
    void record_batch(uint64_t t_ns, uint64_t seq, const drone::ipc::Pose& pose,
                      const std::vector<drone::hal::InferenceDetection>& dets,
                      const std::vector<drone::hal::InferenceDetection>& masks,
                      const std::vector<drone::hal::VoxelUpdate>&        voxels) {
        if (!enabled_ || !out_.is_open()) return;

        // Build compact JSON per line.  Arrays-of-arrays (not arrays-of-
        // objects) keep each record small — the plotter knows the schema.
        nlohmann::json j;
        j["t_ns"]   = t_ns;
        j["seq"]    = seq;
        j["pose_t"] = {pose.translation[0], pose.translation[1], pose.translation[2]};
        j["pose_q"] = {pose.quaternion[0], pose.quaternion[1], pose.quaternion[2],
                       pose.quaternion[3]};

        auto& det_arr = j["det_bb"] = nlohmann::json::array();
        det_arr.get_ref<nlohmann::json::array_t&>().reserve(dets.size());
        for (const auto& d : dets) {
            det_arr.push_back(nlohmann::json::array(
                {d.bbox.x, d.bbox.y, d.bbox.w, d.bbox.h, d.class_id, d.confidence}));
        }

        auto& sam_arr = j["sam_bb"] = nlohmann::json::array();
        sam_arr.get_ref<nlohmann::json::array_t&>().reserve(masks.size());
        for (const auto& m : masks) {
            sam_arr.push_back(
                nlohmann::json::array({m.bbox.x, m.bbox.y, m.bbox.w, m.bbox.h, m.confidence}));
        }

        auto& vox_arr = j["voxels"] = nlohmann::json::array();
        vox_arr.get_ref<nlohmann::json::array_t&>().reserve(voxels.size());
        for (const auto& v : voxels) {
            vox_arr.push_back(
                nlohmann::json::array({v.position_m.x(), v.position_m.y(), v.position_m.z(),
                                       static_cast<int>(v.semantic_label), v.confidence}));
        }

        // Single dump() → one line → one write.  nlohmann::json has no stream-
        // sink overload (only indent/width params), so the intermediate
        // std::string is unavoidable.  Acceptable because `enabled_` defaults
        // to false in production (see DR-033).
        out_ << j.dump() << '\n';
        ++lines_written_;
        if ((lines_written_ % kFlushIntervalLines) == 0) out_.flush();
    }

private:
    // Flush every N lines so in-progress analyses can `tail -f` the file.
    // At 30 Hz, N=50 yields ~1.7 s tail latency — matches the
    // cosys_telemetry_poller's 2 s flush cadence (diagnostic-only paths
    // should feel similar under the developer's terminal).
    static constexpr uint64_t kFlushIntervalLines = 50;

    bool          enabled_{false};
    std::string   path_;
    std::ofstream out_;
    uint64_t      lines_written_{0};
};

}  // namespace drone::util
