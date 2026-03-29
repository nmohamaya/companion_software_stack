// common/ipc/include/ipc/ipc_types.h
// Shared data structures for inter-process communication.
// All types MUST be trivially copyable (no std::string, std::vector, etc.)
#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <type_traits>

namespace drone::ipc {

// ═══════════════════════════════════════════════════════════
// Video Frame (Process 1 → Process 2, Process 3)
// ═══════════════════════════════════════════════════════════
struct VideoFrame {
    uint64_t timestamp_ns;
    uint64_t sequence_number;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t stride;
    uint8_t  pixel_data[1920 * 1080 * 3];  // RGB24 max

    /// Validate frame dimensions fit within the fixed pixel_data buffer.
    [[nodiscard]] bool validate() const {
        if (width == 0 || height == 0 || channels == 0) return false;
        return static_cast<uint64_t>(width) * height * channels <= sizeof(pixel_data);
    }
};

struct StereoFrame {
    uint64_t timestamp_ns;
    uint64_t sequence_number;
    uint32_t width;
    uint32_t height;
    uint8_t  left_data[640 * 480];   // GRAY8
    uint8_t  right_data[640 * 480];  // GRAY8

    [[nodiscard]] bool validate() const {
        if (width == 0 || height == 0) return false;
        uint64_t size = static_cast<uint64_t>(width) * height;
        return size <= sizeof(left_data) && size <= sizeof(right_data);
    }
};

// ═══════════════════════════════════════════════════════════
// Detected Objects (Process 2 → Process 4)
// ═══════════════════════════════════════════════════════════
static constexpr int MAX_DETECTED_OBJECTS = 64;

enum class ObjectClass : uint8_t {
    UNKNOWN       = 0,
    PERSON        = 1,
    VEHICLE_CAR   = 2,
    VEHICLE_TRUCK = 3,
    DRONE         = 4,
    ANIMAL        = 5,
    BUILDING      = 6,
    TREE          = 7,
};

struct DetectedObject {
    uint32_t    track_id;
    ObjectClass class_id;
    float       confidence;
    float       position_x, position_y, position_z;  // vehicle frame (m)
    float       velocity_x, velocity_y, velocity_z;  // m/s
    float       heading;                             // radians
    float       bbox_x, bbox_y, bbox_w, bbox_h;      // image-space
    bool        has_camera;
    bool        has_radar;
    float       estimated_radius_m;  // back-projected obstacle radius (0 = unknown)
    float       estimated_height_m;  // back-projected obstacle height (0 = unknown)
    uint32_t    radar_update_count;  // number of radar updates received

    [[nodiscard]] bool validate() const {
        return std::isfinite(confidence) && confidence >= 0.0f && confidence <= 1.0f &&
               std::isfinite(position_x) && std::isfinite(position_y) &&
               std::isfinite(position_z) && std::isfinite(velocity_x) &&
               std::isfinite(velocity_y) && std::isfinite(velocity_z);
    }
};

struct DetectedObjectList {
    uint64_t       timestamp_ns;
    uint32_t       frame_sequence;
    uint32_t       num_objects;
    DetectedObject objects[MAX_DETECTED_OBJECTS];

    [[nodiscard]] bool validate() const {
        if (num_objects > MAX_DETECTED_OBJECTS) return false;
        for (uint32_t i = 0; i < num_objects; ++i) {
            if (!objects[i].validate()) return false;
        }
        return true;
    }
};

// ═══════════════════════════════════════════════════════════
// SLAM Pose (Process 3 → Process 4, Process 5, Process 6)
// ═══════════════════════════════════════════════════════════
struct alignas(64) Pose {
    uint64_t timestamp_ns;
    double   translation[3];  // x, y, z in world frame
    double   quaternion[4];   // w, x, y, z
    double   velocity[3];     // vx, vy, vz
    double   covariance[36];  // 6x6 pose covariance
    uint32_t quality;         // 0=lost, 1=degraded, 2=good, 3=excellent (ground truth)

    [[nodiscard]] bool validate() const {
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(translation[i]) || !std::isfinite(velocity[i])) return false;
        }
        for (int i = 0; i < 4; ++i) {
            if (!std::isfinite(quaternion[i])) return false;
        }
        return quality <= 3;  // 0=lost, 1=degraded, 2=good, 3=excellent (ground truth)
    }
};

// ═══════════════════════════════════════════════════════════
// Mission Status (Process 4 → Process 5, Process 7)
// ═══════════════════════════════════════════════════════════
enum class MissionState : uint8_t {
    IDLE      = 0,
    PREFLIGHT = 1,
    TAKEOFF   = 2,
    NAVIGATE  = 3,
    LOITER    = 4,
    RTL       = 5,
    LAND      = 6,
    EMERGENCY = 7,
    SURVEY    = 8  // Post-takeoff obstacle survey before navigation
};

// ═══════════════════════════════════════════════════════════
// Fault Classification — consumed by MissionStatus and FaultManager
// ═══════════════════════════════════════════════════════════

/// Graduated response severity (stored in MissionStatus::fault_action).
enum class FaultAction : uint8_t {
    NONE           = 0,  // nominal — no action
    WARN           = 1,  // alert GCS, continue mission
    LOITER         = 2,  // hold position, wait for recovery
    RTL            = 3,  // return to launch
    EMERGENCY_LAND = 4   // land immediately at current position
};

inline const char* fault_action_name(FaultAction a) {
    switch (a) {
        case FaultAction::NONE: return "NONE";
        case FaultAction::WARN: return "WARN";
        case FaultAction::LOITER: return "LOITER";
        case FaultAction::RTL: return "RTL";
        case FaultAction::EMERGENCY_LAND: return "EMERGENCY_LAND";
        default: return "UNKNOWN";
    }
}

/// Bitmask of active fault conditions (stored in MissionStatus::active_faults).
enum class FaultType : uint32_t {
    FAULT_NONE             = 0,
    FAULT_CRITICAL_PROCESS = 1 << 0,   // comms or SLAM died
    FAULT_POSE_STALE       = 1 << 1,   // no pose update within timeout
    FAULT_BATTERY_LOW      = 1 << 2,   // battery below warning threshold
    FAULT_BATTERY_CRITICAL = 1 << 3,   // battery below critical threshold
    FAULT_THERMAL_WARNING  = 1 << 4,   // thermal zone 2 (hot)
    FAULT_THERMAL_CRITICAL = 1 << 5,   // thermal zone 3 (critical)
    FAULT_PERCEPTION_DEAD  = 1 << 6,   // perception process died
    FAULT_FC_LINK_LOST     = 1 << 7,   // FC not connected for >timeout
    FAULT_GEOFENCE_BREACH  = 1 << 8,   // outside geofence boundary
    FAULT_BATTERY_RTL      = 1 << 9,   // battery below RTL threshold
    FAULT_VIO_DEGRADED     = 1 << 10,  // VIO quality degraded → LOITER
    FAULT_VIO_LOST         = 1 << 11,  // VIO tracking lost → RTL
};

/// Bitwise operators for FaultType bitmask usage with uint32_t.
constexpr uint32_t operator|(FaultType a, FaultType b) {
    return static_cast<uint32_t>(a) | static_cast<uint32_t>(b);
}
constexpr uint32_t operator&(uint32_t a, FaultType b) {
    return a & static_cast<uint32_t>(b);
}
constexpr uint32_t& operator|=(uint32_t& a, FaultType b) {
    return a |= static_cast<uint32_t>(b);
}
constexpr bool operator==(uint32_t a, FaultType b) {
    return a == static_cast<uint32_t>(b);
}
constexpr bool operator!=(uint32_t a, FaultType b) {
    return !(a == b);
}
constexpr uint32_t to_uint(FaultType f) {
    return static_cast<uint32_t>(f);
}

/// Convert active fault bitmask to a human-readable pipe-separated string.
inline std::string fault_flags_string(uint32_t flags) {
    if (flags == 0) return "FAULT_NONE";
    std::string result;
    struct Flag {
        uint32_t    bit;
        const char* name;
    };
    static constexpr Flag kFlags[] = {
        {static_cast<uint32_t>(FaultType::FAULT_CRITICAL_PROCESS), "FAULT_CRITICAL_PROCESS"},
        {static_cast<uint32_t>(FaultType::FAULT_POSE_STALE), "FAULT_POSE_STALE"},
        {static_cast<uint32_t>(FaultType::FAULT_BATTERY_LOW), "FAULT_BATTERY_LOW"},
        {static_cast<uint32_t>(FaultType::FAULT_BATTERY_CRITICAL), "FAULT_BATTERY_CRITICAL"},
        {static_cast<uint32_t>(FaultType::FAULT_THERMAL_WARNING), "FAULT_THERMAL_WARNING"},
        {static_cast<uint32_t>(FaultType::FAULT_THERMAL_CRITICAL), "FAULT_THERMAL_CRITICAL"},
        {static_cast<uint32_t>(FaultType::FAULT_PERCEPTION_DEAD), "FAULT_PERCEPTION_DEAD"},
        {static_cast<uint32_t>(FaultType::FAULT_FC_LINK_LOST), "FAULT_FC_LINK_LOST"},
        {static_cast<uint32_t>(FaultType::FAULT_GEOFENCE_BREACH), "FAULT_GEOFENCE_BREACH"},
        {static_cast<uint32_t>(FaultType::FAULT_BATTERY_RTL), "FAULT_BATTERY_RTL"},
        {static_cast<uint32_t>(FaultType::FAULT_VIO_DEGRADED), "FAULT_VIO_DEGRADED"},
        {static_cast<uint32_t>(FaultType::FAULT_VIO_LOST), "FAULT_VIO_LOST"},
    };
    uint32_t known = 0;
    for (const auto& f : kFlags) {
        if (flags & f.bit) {
            if (!result.empty()) result += '|';
            result += f.name;
            known |= f.bit;
        }
    }
    uint32_t unknown = flags & ~known;
    if (unknown) {
        if (!result.empty()) result += '|';
        char buf[32];
        std::snprintf(buf, sizeof(buf), "FAULT_UNKNOWN(0x%X)", unknown);
        result += buf;
    }
    return result;
}

struct MissionStatus {
    uint64_t     timestamp_ns;
    uint64_t     correlation_id;  // cross-process trace ID (0 = none)
    MissionState state;
    uint32_t     current_waypoint;
    uint32_t     total_waypoints;
    float        progress_percent;
    float        target_x, target_y, target_z;
    float        battery_percent;
    bool         mission_active;
    uint32_t     active_faults;  // bitmask of FaultType (0 = nominal)
    uint8_t      fault_action;   // current FaultAction severity (0 = NONE)

    [[nodiscard]] bool validate() const {
        return std::isfinite(progress_percent) && std::isfinite(target_x) &&
               std::isfinite(target_y) && std::isfinite(target_z) && std::isfinite(battery_percent);
    }
};

// ═══════════════════════════════════════════════════════════
// Trajectory Command (Process 4 → Process 5)
// ═══════════════════════════════════════════════════════════
struct TrajectoryCmd {
    uint64_t timestamp_ns;
    uint64_t correlation_id;                // cross-process trace ID (0 = none)
    float    target_x, target_y, target_z;  // world frame (m)
    float    target_yaw;                    // radians
    float    velocity_x, velocity_y, velocity_z;
    float    yaw_rate;
    uint8_t  coordinate_frame;  // MAVLink frame enum
    bool     valid;

    [[nodiscard]] bool validate() const {
        if (!valid) return true;  // stop commands (valid=false) are always OK
        return std::isfinite(target_x) && std::isfinite(target_y) && std::isfinite(target_z) &&
               std::isfinite(target_yaw) && std::isfinite(velocity_x) &&
               std::isfinite(velocity_y) && std::isfinite(velocity_z) && std::isfinite(yaw_rate);
    }
};

// ═══════════════════════════════════════════════════════════
// Payload Commands (Process 4 → Process 6)
// ═══════════════════════════════════════════════════════════
enum class PayloadAction : uint8_t {
    NONE               = 0,
    GIMBAL_POINT       = 1,
    CAMERA_CAPTURE     = 2,
    CAMERA_START_VIDEO = 3,
    CAMERA_STOP_VIDEO  = 4
};

struct PayloadCommand {
    uint64_t      timestamp_ns;
    uint64_t      correlation_id;  // cross-process trace ID (0 = none)
    PayloadAction action;
    float         gimbal_pitch, gimbal_yaw;  // degrees
    uint64_t      sequence_id;
    bool          valid;

    [[nodiscard]] bool validate() const {
        return std::isfinite(gimbal_pitch) && std::isfinite(gimbal_yaw);
    }
};

// ═══════════════════════════════════════════════════════════
// FC Command (Process 4 → Process 5)
// ═══════════════════════════════════════════════════════════
enum class FCCommandType : uint8_t {
    NONE     = 0,
    ARM      = 1,
    DISARM   = 2,
    TAKEOFF  = 3,
    SET_MODE = 4,
    RTL      = 5,
    LAND     = 6
};

struct FCCommand {
    uint64_t      timestamp_ns;
    uint64_t      correlation_id;  // cross-process trace ID (0 = none)
    FCCommandType command;
    float         param1;       // TAKEOFF: altitude_m; SET_MODE: mode_id
    uint64_t      sequence_id;  // monotonic, for dedup
    bool          valid;

    [[nodiscard]] bool validate() const {
        return static_cast<uint8_t>(command) <= static_cast<uint8_t>(FCCommandType::LAND) &&
               std::isfinite(param1);
    }
};

// ═══════════════════════════════════════════════════════════
// FC State (Process 5 → Process 4)
// ═══════════════════════════════════════════════════════════
struct FCState {
    uint64_t timestamp_ns;
    float    gps_lat, gps_lon, gps_alt;
    float    rel_alt;
    float    roll, pitch, yaw;
    float    vx, vy, vz;
    float    battery_voltage;
    float    battery_remaining;
    uint8_t  flight_mode;
    bool     armed;
    bool     connected;
    uint8_t  gps_fix_type;
    uint8_t  satellites_visible;

    [[nodiscard]] bool validate() const {
        return std::isfinite(battery_voltage) && battery_voltage >= 0.0f &&
               battery_voltage <= 60.0f && std::isfinite(battery_remaining) &&
               battery_remaining >= 0.0f && battery_remaining <= 100.0f && std::isfinite(roll) &&
               std::isfinite(pitch) && std::isfinite(yaw) && std::isfinite(vx) &&
               std::isfinite(vy) && std::isfinite(vz);
    }
};

// ═══════════════════════════════════════════════════════════
// GCS Commands (Process 5 → Process 4)
// ═══════════════════════════════════════════════════════════
enum class GCSCommandType : uint8_t {
    NONE           = 0,
    ARM            = 1,
    DISARM         = 2,
    TAKEOFF        = 3,
    LAND           = 4,
    RTL            = 5,
    MISSION_START  = 6,
    MISSION_PAUSE  = 7,
    MISSION_ABORT  = 8,
    MISSION_UPLOAD = 9  // Upload new waypoints mid-flight
};

struct GCSCommand {
    uint64_t       timestamp_ns;
    uint64_t       correlation_id;  // cross-process trace ID (0 = none)
    GCSCommandType command;
    float          param1, param2, param3;
    uint64_t       sequence_id;
    bool           valid;

    [[nodiscard]] bool validate() const {
        return static_cast<uint8_t>(command) <=
               static_cast<uint8_t>(GCSCommandType::MISSION_UPLOAD);
    }
};

// ═══════════════════════════════════════════════════════════
// Mission Upload (Process 5 → Process 4)  — mid-flight
// waypoint upload.  The GCS sends MISSION_UPLOAD, then the
// planner reads waypoints from this segment.
// ═══════════════════════════════════════════════════════════
static constexpr uint8_t kMaxUploadWaypoints = 32;

struct IpcWaypoint {
    float x{}, y{}, z{};           // world-frame position
    float yaw{};                   // heading (rad)
    float radius{2.0f};            // acceptance radius (m)
    float speed{2.0f};             // cruise speed (m/s)
    bool  trigger_payload{false};  // capture at this waypoint

    [[nodiscard]] bool validate() const {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(yaw) &&
               std::isfinite(radius) && radius > 0.0f && std::isfinite(speed) && speed > 0.0f;
    }
};
static_assert(std::is_trivially_copyable_v<IpcWaypoint>,
              "IpcWaypoint must be trivially copyable for IPC");

struct MissionUpload {
    uint64_t    timestamp_ns{};
    uint64_t    correlation_id{};
    uint8_t     num_waypoints{};
    IpcWaypoint waypoints[kMaxUploadWaypoints]{};
    bool        valid{false};

    [[nodiscard]] bool validate() const {
        if (num_waypoints > kMaxUploadWaypoints) return false;
        for (uint8_t i = 0; i < num_waypoints; ++i) {
            if (!waypoints[i].validate()) return false;
        }
        return true;
    }
};
static_assert(std::is_trivially_copyable_v<MissionUpload>,
              "MissionUpload must be trivially copyable for IPC");

// ═══════════════════════════════════════════════════════════
// Payload Status (Process 6 → Process 4, Process 7)
// ═══════════════════════════════════════════════════════════
struct PayloadStatus {
    uint64_t timestamp_ns;
    float    gimbal_pitch, gimbal_yaw;
    uint32_t images_captured;
    bool     recording_video;
    bool     gimbal_stabilized;
    uint8_t  num_plugins_active;

    [[nodiscard]] bool validate() const {
        return std::isfinite(gimbal_pitch) && std::isfinite(gimbal_yaw);
    }
};

// ═══════════════════════════════════════════════════════════
// Process Health Entry — per-process liveness state
// ═══════════════════════════════════════════════════════════
static constexpr uint8_t kMaxTrackedProcesses = 8;  // 7 processes + 1 FC link

struct ProcessHealthEntry {
    char     name[32]     = {};     // Process name (e.g. "video_capture")
    bool     alive        = false;  // Last known liveness state
    uint64_t last_seen_ns = 0;      // Timestamp of last liveliness event
};

// ═══════════════════════════════════════════════════════════
// System Health (Process 7 → all)
// ═══════════════════════════════════════════════════════════
struct SystemHealth {
    uint64_t timestamp_ns;
    float    cpu_usage_percent;
    float    memory_usage_percent;
    float    disk_usage_percent;
    float    max_temp_c;
    float    gpu_temp_c;
    float    cpu_temp_c;
    uint32_t total_healthy;
    uint32_t total_degraded;
    uint32_t total_dead;
    float    power_watts;
    uint8_t  thermal_zone;  // 0=normal, 1=warm, 2=hot, 3=critical

    // ── Stack-level status (Phase 4) ────────────────────────
    uint8_t  stack_status   = 0;  // StackStatus enum (0=NOMINAL, 1=DEGRADED, 2=CRITICAL)
    uint32_t total_restarts = 0;  // Cumulative restart count across all processes

    // ── Process health (populated by LivelinessMonitor) ─────
    ProcessHealthEntry processes[kMaxTrackedProcesses] = {};
    uint8_t            num_processes                   = 0;      // Number of tracked processes
    bool               critical_failure                = false;  // True if a critical process died

    [[nodiscard]] bool validate() const {
        return std::isfinite(cpu_usage_percent) && std::isfinite(memory_usage_percent) &&
               std::isfinite(max_temp_c) && std::isfinite(power_watts) && thermal_zone <= 3 &&
               num_processes <= kMaxTrackedProcesses;
    }
};

// ═══════════════════════════════════════════════════════════
// Thread Health Entry — per-thread liveness state
// ═══════════════════════════════════════════════════════════
static constexpr uint8_t kMaxTrackedThreads = 16;

struct ThreadHealthEntry {
    char     name[32] = {};     // Thread name (e.g. "mission_cam")
    bool     healthy  = true;   // false when watchdog detects stuck
    bool     critical = false;  // true for mission-critical threads
    uint64_t last_ns  = 0;      // Last heartbeat timestamp (ns)
};

// ═══════════════════════════════════════════════════════════
// Thread Health (each process → Process 7)
// ═══════════════════════════════════════════════════════════
struct ThreadHealth {
    char              process_name[32]            = {};
    ThreadHealthEntry threads[kMaxTrackedThreads] = {};
    uint8_t           num_threads                 = 0;
    uint64_t          timestamp_ns                = 0;
};

static_assert(std::is_trivially_copyable_v<ThreadHealth>,
              "ThreadHealth must be trivially copyable for IPC");

// ═══════════════════════════════════════════════════════════
// Fault Injection Overrides — written by the fault_injector tool
// and read by the processes that own the overridden resources.
// Each field uses a sentinel value (<0) to indicate "no override".
// ═══════════════════════════════════════════════════════════
struct alignas(64) FaultOverrides {
    // FC state overrides (consumed by Process 5 comms)
    float   battery_percent = -1.0f;  // <0 = no override
    float   battery_voltage = -1.0f;  // <0 = no override
    int32_t fc_connected    = -1;     // <0 = no override, 0 = disconnected, 1 = connected
    // System health overrides (consumed by Process 7 system monitor)
    int32_t thermal_zone      = -1;     // <0 = no override, 0-3 = zone
    float   cpu_temp_override = -1.0f;  // <0 = no override
    // VIO quality override (consumed by Process 3 SLAM/VIO)
    int32_t vio_quality = -1;  // <0 = no override, 0-3 = quality level
    // Sequence counter – incremented by the injector so consumers can
    // detect new writes vs stale values.
    uint64_t sequence = 0;
};

static_assert(std::is_trivially_copyable_v<FaultOverrides>,
              "FaultOverrides must be trivially copyable for IPC");

// ═══════════════════════════════════════════════════════════
// Radar Detections (Process 2 → Process 4)
// ═══════════════════════════════════════════════════════════

/// Single radar return — range, bearing, Doppler velocity.
struct RadarDetection {
    uint64_t timestamp_ns{0};
    float    range_m{0.0f};
    float    azimuth_rad{0.0f};
    float    elevation_rad{0.0f};
    float    radial_velocity_mps{0.0f};
    float    rcs_dbsm{0.0f};
    float    snr_db{0.0f};
    float    confidence{0.0f};
    uint32_t track_id{0};

    [[nodiscard]] bool validate() const {
        return std::isfinite(range_m) && std::isfinite(azimuth_rad) &&
               std::isfinite(elevation_rad) && std::isfinite(radial_velocity_mps) &&
               std::isfinite(rcs_dbsm) && std::isfinite(snr_db) && std::isfinite(confidence) &&
               range_m >= 0.0f && confidence >= 0.0f && confidence <= 1.0f;
    }
};

static constexpr uint32_t MAX_RADAR_DETECTIONS = 128;

struct RadarDetectionList {
    uint64_t       timestamp_ns{0};
    uint32_t       num_detections{0};
    RadarDetection detections[MAX_RADAR_DETECTIONS];

    [[nodiscard]] bool validate() const {
        if (num_detections > MAX_RADAR_DETECTIONS) return false;
        for (uint32_t i = 0; i < num_detections; ++i) {
            if (!detections[i].validate()) return false;
        }
        return true;
    }
};

static_assert(std::is_trivially_copyable_v<RadarDetection>,
              "RadarDetection must be trivially copyable for IPC");
static_assert(std::is_trivially_copyable_v<RadarDetectionList>,
              "RadarDetectionList must be trivially copyable for IPC");

// ═══════════════════════════════════════════════════════════
// IPC Topic Names — centralised to avoid typos
// ═══════════════════════════════════════════════════════════
namespace topics {
constexpr const char* VIDEO_MISSION_CAM = "/drone_mission_cam";
constexpr const char* VIDEO_STEREO_CAM  = "/drone_stereo_cam";
constexpr const char* DETECTED_OBJECTS  = "/detected_objects";
constexpr const char* SLAM_POSE         = "/slam_pose";
constexpr const char* MISSION_STATUS    = "/mission_status";
constexpr const char* TRAJECTORY_CMD    = "/trajectory_cmd";
constexpr const char* PAYLOAD_COMMANDS  = "/payload_commands";
constexpr const char* FC_COMMANDS       = "/fc_commands";
constexpr const char* FC_STATE          = "/fc_state";
constexpr const char* GCS_COMMANDS      = "/gcs_commands";
constexpr const char* MISSION_UPLOAD    = "/mission_upload";
constexpr const char* PAYLOAD_STATUS    = "/payload_status";
constexpr const char* SYSTEM_HEALTH     = "/system_health";
constexpr const char* FAULT_OVERRIDES   = "/fault_overrides";
constexpr const char* RADAR_DETECTIONS  = "/radar_detections";

// ── Per-process thread health channels ──────────────────
constexpr const char* THREAD_HEALTH_VIDEO_CAPTURE   = "/drone_thread_health_video_capture";
constexpr const char* THREAD_HEALTH_PERCEPTION      = "/drone_thread_health_perception";
constexpr const char* THREAD_HEALTH_SLAM_VIO_NAV    = "/drone_thread_health_slam_vio_nav";
constexpr const char* THREAD_HEALTH_MISSION_PLANNER = "/drone_thread_health_mission_planner";
constexpr const char* THREAD_HEALTH_COMMS           = "/drone_thread_health_comms";
constexpr const char* THREAD_HEALTH_PAYLOAD_MANAGER = "/drone_thread_health_payload_manager";
constexpr const char* THREAD_HEALTH_SYSTEM_MONITOR  = "/drone_thread_health_system_monitor";
}  // namespace topics

}  // namespace drone::ipc
