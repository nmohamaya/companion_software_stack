// process2_perception/include/perception/itracker.h
// Abstract tracker interface — strategy pattern for multi-object tracking.
// Concrete implementations: SortTracker (SORT), ByteTrackTracker (ByteTrack two-stage).
// Issue #113 — Phase 1B.
#pragma once
#include "perception/types.h"

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
/// Supported backends: "sort" (default), "bytetrack".
/// Looks up "perception.tracker.backend" in cfg if provided.
std::unique_ptr<ITracker> create_tracker(const std::string&   backend = "sort",
                                         const drone::Config* cfg     = nullptr);

}  // namespace drone::perception
