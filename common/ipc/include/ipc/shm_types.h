// common/ipc/include/ipc/shm_types.h
// Shared data structures for inter-process communication via SHM.
// All types MUST be trivially copyable (no std::string, std::vector, etc.)
#pragma once
#include <cstdint>
#include <atomic>

namespace drone::ipc {

// ═══════════════════════════════════════════════════════════
// Video Frame SHM (Process 1 → Process 2, Process 3)
// ═══════════════════════════════════════════════════════════
struct ShmVideoFrame {
    uint64_t timestamp_ns;
    uint64_t sequence_number;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t stride;
    uint8_t  pixel_data[1920 * 1080 * 3];  // RGB24 max
};

struct ShmStereoFrame {
    uint64_t timestamp_ns;
    uint64_t sequence_number;
    uint32_t width;
    uint32_t height;
    uint8_t  left_data[640 * 480];   // GRAY8
    uint8_t  right_data[640 * 480];  // GRAY8
};

// ═══════════════════════════════════════════════════════════
// Detected Objects SHM (Process 2 → Process 4)
// ═══════════════════════════════════════════════════════════
static constexpr int MAX_DETECTED_OBJECTS = 64;

enum class ObjectClass : uint8_t {
    UNKNOWN = 0, PERSON = 1, VEHICLE_CAR = 2, VEHICLE_TRUCK = 3,
    DRONE = 4, ANIMAL = 5, BUILDING = 6, TREE = 7,
};

struct ShmDetectedObject {
    uint32_t track_id;
    ObjectClass class_id;
    float confidence;
    float position_x, position_y, position_z;   // vehicle frame (m)
    float velocity_x, velocity_y, velocity_z;   // m/s
    float heading;                               // radians
    float bbox_x, bbox_y, bbox_w, bbox_h;       // image-space
    bool  has_camera, has_lidar, has_radar;
};

struct ShmDetectedObjectList {
    uint64_t timestamp_ns;
    uint32_t frame_sequence;
    uint32_t num_objects;
    ShmDetectedObject objects[MAX_DETECTED_OBJECTS];
};

// ═══════════════════════════════════════════════════════════
// SLAM Pose SHM (Process 3 → Process 4, Process 5, Process 6)
// ═══════════════════════════════════════════════════════════
struct alignas(64) ShmPose {
    uint64_t timestamp_ns;
    double translation[3];     // x, y, z in world frame
    double quaternion[4];      // w, x, y, z
    double velocity[3];        // vx, vy, vz
    double covariance[36];     // 6x6 pose covariance
    uint32_t quality;          // 0=lost, 1=degraded, 2=good
};

// ═══════════════════════════════════════════════════════════
// Mission Status SHM (Process 4 → Process 5, Process 7)
// ═══════════════════════════════════════════════════════════
enum class MissionState : uint8_t {
    IDLE = 0, PREFLIGHT = 1, TAKEOFF = 2, NAVIGATE = 3,
    LOITER = 4, RTL = 5, LAND = 6, EMERGENCY = 7
};

struct ShmMissionStatus {
    uint64_t timestamp_ns;
    MissionState state;
    uint32_t current_waypoint;
    uint32_t total_waypoints;
    float progress_percent;
    float target_x, target_y, target_z;
    float battery_percent;
    bool  mission_active;
};

// ═══════════════════════════════════════════════════════════
// Trajectory Command SHM (Process 4 → Process 5)
// ═══════════════════════════════════════════════════════════
struct ShmTrajectoryCmd {
    uint64_t timestamp_ns;
    float target_x, target_y, target_z;    // world frame (m)
    float target_yaw;                       // radians
    float velocity_x, velocity_y, velocity_z;
    float yaw_rate;
    uint8_t coordinate_frame;               // MAVLink frame enum
    bool valid;
};

// ═══════════════════════════════════════════════════════════
// Payload Commands SHM (Process 4 → Process 6)
// ═══════════════════════════════════════════════════════════
enum class PayloadAction : uint8_t {
    NONE = 0, GIMBAL_POINT = 1, CAMERA_CAPTURE = 2,
    CAMERA_START_VIDEO = 3, CAMERA_STOP_VIDEO = 4
};

struct ShmPayloadCommand {
    uint64_t timestamp_ns;
    PayloadAction action;
    float gimbal_pitch, gimbal_yaw;  // degrees
    uint64_t sequence_id;
    bool valid;
};

// ═══════════════════════════════════════════════════════════
// FC State SHM (Process 5 → Process 4)
// ═══════════════════════════════════════════════════════════
struct ShmFCState {
    uint64_t timestamp_ns;
    float gps_lat, gps_lon, gps_alt;
    float rel_alt;
    float roll, pitch, yaw;
    float vx, vy, vz;
    float battery_voltage;
    float battery_remaining;
    uint8_t flight_mode;
    bool armed;
    bool connected;
    uint8_t gps_fix_type;
    uint8_t satellites_visible;
};

// ═══════════════════════════════════════════════════════════
// GCS Commands SHM (Process 5 → Process 4)
// ═══════════════════════════════════════════════════════════
enum class GCSCommandType : uint8_t {
    NONE = 0, ARM = 1, DISARM = 2, TAKEOFF = 3, LAND = 4,
    RTL = 5, MISSION_START = 6, MISSION_PAUSE = 7, MISSION_ABORT = 8
};

struct ShmGCSCommand {
    uint64_t timestamp_ns;
    GCSCommandType command;
    float param1, param2, param3;
    uint64_t sequence_id;
    bool valid;
};

// ═══════════════════════════════════════════════════════════
// Payload Status SHM (Process 6 → Process 4, Process 7)
// ═══════════════════════════════════════════════════════════
struct ShmPayloadStatus {
    uint64_t timestamp_ns;
    float gimbal_pitch, gimbal_yaw;
    uint32_t images_captured;
    bool recording_video;
    bool gimbal_stabilized;
    uint8_t num_plugins_active;
};

// ═══════════════════════════════════════════════════════════
// System Health SHM (Process 7 → all)
// ═══════════════════════════════════════════════════════════
struct ShmSystemHealth {
    uint64_t timestamp_ns;
    float cpu_usage_percent;
    float memory_usage_percent;
    float disk_usage_percent;
    float max_temp_c;
    float gpu_temp_c;
    float cpu_temp_c;
    uint32_t total_healthy;
    uint32_t total_degraded;
    uint32_t total_dead;
    float power_watts;
    uint8_t thermal_zone;  // 0=normal, 1=warm, 2=hot, 3=critical
};

// ═══════════════════════════════════════════════════════════
// SHM Segment Names — centralised to avoid typos
// ═══════════════════════════════════════════════════════════
namespace shm_names {
    constexpr const char* VIDEO_MISSION_CAM   = "/drone_mission_cam";
    constexpr const char* VIDEO_STEREO_CAM    = "/drone_stereo_cam";
    constexpr const char* DETECTED_OBJECTS    = "/detected_objects";
    constexpr const char* SLAM_POSE           = "/slam_pose";
    constexpr const char* MISSION_STATUS      = "/mission_status";
    constexpr const char* TRAJECTORY_CMD      = "/trajectory_cmd";
    constexpr const char* PAYLOAD_COMMANDS    = "/payload_commands";
    constexpr const char* FC_STATE            = "/fc_state";
    constexpr const char* GCS_COMMANDS        = "/gcs_commands";
    constexpr const char* PAYLOAD_STATUS      = "/payload_status";
    constexpr const char* SYSTEM_HEALTH       = "/system_health";
}

} // namespace drone::ipc
