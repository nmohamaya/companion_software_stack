// process2_perception/include/perception/itracker.h
// Abstract tracker interface — strategy pattern for multi-object tracking.
// Concrete implementation: ByteTrackTracker (two-stage IoU association).
// Issue #113 — Phase 1B.  Issue #297 — Result<> factory cleanup.
#pragma once
#include "perception/types.h"
#include "util/result.h"

#include <memory>
#include <string>

namespace drone {
class Config;
}

namespace drone::perception {

/// Abstract multi-object tracker interface.
/// Implementations consume per-frame 2D detections and produce tracked objects
/// with persistent IDs, velocity estimates, and track state (tentative/confirmed/lost).
class ITracker {
public:
    virtual ~ITracker() = default;

    /// Feed one frame of detections; returns currently confirmed tracks.
    [[nodiscard]] virtual TrackedObjectList update(const Detection2DList& detections) = 0;

    /// Human-readable name for logging.
    [[nodiscard]] virtual std::string name() const = 0;

    /// Reset all internal state (tracks, IDs) — useful for unit tests or mode switches.
    virtual void reset() = 0;
};

/// Factory: create a tracker from a backend name.
///
/// Instantiates the tracker type matching the given backend string.
/// cfg is used to override tunable parameters only; the backend is selected
/// explicitly by the caller (not read from config).
/// Supported backends: "bytetrack" (default).
/// Unknown backends log a warning and fall back to ByteTrack with default params.
[[nodiscard]] drone::util::Result<std::unique_ptr<ITracker>> create_tracker(
    const std::string& backend = "bytetrack", const drone::Config* cfg = nullptr);

}  // namespace drone::perception
