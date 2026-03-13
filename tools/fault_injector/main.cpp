// tools/fault_injector/main.cpp
// ═══════════════════════════════════════════════════════════════════
// Fault Injection CLI — writes directly to SHM IPC channels to
// simulate faults and GCS commands for integration testing.
//
// Usage:
//   fault_injector battery <percent>          — override FC battery %
//   fault_injector fc_disconnect              — set FC state to disconnected
//   fault_injector fc_reconnect               — set FC state to connected
//   fault_injector gcs_command <cmd> [params] — inject GCS command
//   fault_injector thermal_zone <0-3>         — override thermal zone
//   fault_injector mission_upload <json_file> — upload new waypoints
//   fault_injector sequence <json_file>       — timed sequence of faults
//
// The tool attaches to existing SHM segments (created by running processes)
// and writes data directly without taking ownership (no shm_unlink on exit).
// ═══════════════════════════════════════════════════════════════════
#include "ipc/ipc_types.h"
#include "ipc/message_bus.h"
#include "ipc/message_bus_factory.h"
#include "ipc/shm_writer.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ── Timing helper ────────────────────────────────────────────
static uint64_t now_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

// ── Colour helpers ───────────────────────────────────────────
static constexpr const char* GREEN  = "\033[0;32m";
static constexpr const char* RED    = "\033[0;31m";
static constexpr const char* YELLOW = "\033[1;33m";
static constexpr const char* CYAN   = "\033[0;36m";
static constexpr const char* NC     = "\033[0m";

static void print_ok(const std::string& msg) {
    std::cout << GREEN << "[OK] " << NC << msg << std::endl;
}

static void print_err(const std::string& msg) {
    std::cerr << RED << "[ERR] " << NC << msg << std::endl;
}

static void print_info(const std::string& msg) {
    std::cout << CYAN << "[INJ] " << NC << msg << std::endl;
}

// ── IPC backend selection ───────────────────────────────────
static std::string g_ipc_backend = "shm";

/// Publish a message via the transport-agnostic MessageBus.
/// Lazily creates the bus and caches publishers per topic.
template<typename T>
static void publish_via_bus(const std::string& topic, const T& msg) {
    static auto bus = drone::ipc::create_message_bus(g_ipc_backend);
    static std::unordered_map<std::string, std::unique_ptr<drone::ipc::IPublisher<T>>> pubs;
    static std::unordered_map<std::string, bool> first_advertise;

    auto it = pubs.find(topic);
    if (it == pubs.end()) {
        auto pub = bus.advertise<T>(topic);
        if (!pub) {
            print_err("Failed to advertise on topic: " + topic);
            return;
        }
        pubs[topic]            = std::move(pub);
        first_advertise[topic] = true;
        it                     = pubs.find(topic);
    }

    // Zenoh needs a brief delay after first advertise for discovery
    if (first_advertise[topic] && g_ipc_backend == "zenoh") {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        first_advertise[topic] = false;
    }

    it->second->publish(msg);
}

// ═══════════════════════════════════════════════════════════════
// Helpers — fault override SHM segment
// ═══════════════════════════════════════════════════════════════

/// Process-local mirror of the current override state.
static drone::ipc::FaultOverrides g_overrides = {
    /*.battery_percent=*/-1.0f,   /*.battery_voltage=*/-1.0f,
    /*.fc_connected=*/-1,         /*.thermal_zone=*/-1,
    /*.cpu_temp_override=*/-1.0f, /*.sequence=*/0};

static ShmWriter<drone::ipc::FaultOverrides>& get_override_writer() {
    static ShmWriter<drone::ipc::FaultOverrides> writer;
    static bool                                  initialised = false;
    if (!initialised) {
        // Try attach first (segment may already exist), then create.
        bool ok = writer.attach(drone::ipc::topics::FAULT_OVERRIDES);
        if (!ok) {
            ok = writer.create_non_owning(drone::ipc::topics::FAULT_OVERRIDES);
        }
        if (!ok) {
            print_err("Failed to attach to or create fault override SHM segment");
            throw std::runtime_error("fault_injector: unable to open fault override SHM segment");
        }
        writer.write(g_overrides);
        initialised = true;
    }
    return writer;
}

/// Mutate the process-local override state and flush to SHM.
static void update_overrides(std::function<void(drone::ipc::FaultOverrides&)> mutator) {
    mutator(g_overrides);
    ++g_overrides.sequence;
    get_override_writer().write(g_overrides);
}

// ═══════════════════════════════════════════════════════════════
// Command: battery <percent>
// Sets battery override in /fault_overrides.
// ═══════════════════════════════════════════════════════════════
static int cmd_battery(float percent) {
    if (percent < 0.0f || percent > 100.0f) {
        print_err("Battery percent must be 0-100 (got " + std::to_string(percent) + ")");
        return 1;
    }

    update_overrides([&](drone::ipc::FaultOverrides& ovr) {
        ovr.battery_percent = percent;
        ovr.battery_voltage = 12.0f + percent * 0.048f;
    });

    std::ostringstream oss;
    oss << "Battery override: " << percent << "% (voltage=" << (12.0f + percent * 0.048f) << "V)";
    print_ok(oss.str());
    return 0;
}

// ═══════════════════════════════════════════════════════════════
// Command: fc_disconnect / fc_reconnect
// Sets FC connected override in /fault_overrides.
// ═══════════════════════════════════════════════════════════════
static int cmd_fc_link(bool connected) {
    update_overrides(
        [&](drone::ipc::FaultOverrides& ovr) { ovr.fc_connected = connected ? 1 : 0; });
    print_ok(connected ? "FC link restored" : "FC link disconnected (stale timestamp)");
    return 0;
}

// ═══════════════════════════════════════════════════════════════
// Command: gcs_command <cmd> [p1] [p2] [p3]
// Injects a GCS command via /gcs_commands channel.
// ═══════════════════════════════════════════════════════════════
static drone::ipc::GCSCommandType parse_gcs_cmd(const std::string& cmd) {
    if (cmd == "arm") return drone::ipc::GCSCommandType::ARM;
    if (cmd == "disarm") return drone::ipc::GCSCommandType::DISARM;
    if (cmd == "takeoff") return drone::ipc::GCSCommandType::TAKEOFF;
    if (cmd == "land") return drone::ipc::GCSCommandType::LAND;
    if (cmd == "rtl") return drone::ipc::GCSCommandType::RTL;
    if (cmd == "mission_start") return drone::ipc::GCSCommandType::MISSION_START;
    if (cmd == "mission_pause") return drone::ipc::GCSCommandType::MISSION_PAUSE;
    if (cmd == "mission_abort") return drone::ipc::GCSCommandType::MISSION_ABORT;
    if (cmd == "mission_upload") return drone::ipc::GCSCommandType::MISSION_UPLOAD;
    return drone::ipc::GCSCommandType::NONE;
}

static int cmd_gcs_command(const std::string& cmd, float p1, float p2, float p3) {
    auto gcs_type = parse_gcs_cmd(cmd);
    if (gcs_type == drone::ipc::GCSCommandType::NONE && cmd != "none") {
        print_err("Unknown GCS command: " + cmd);
        std::cerr << "  Valid commands: arm, disarm, takeoff, land, rtl, "
                     "mission_start, mission_pause, mission_abort, mission_upload"
                  << std::endl;
        return 1;
    }

    static uint64_t        seq = 0;
    drone::ipc::GCSCommand shm_cmd{};
    shm_cmd.timestamp_ns   = now_ns();
    shm_cmd.correlation_id = now_ns();  // simple unique ID
    shm_cmd.command        = gcs_type;
    shm_cmd.param1         = p1;
    shm_cmd.param2         = p2;
    shm_cmd.param3         = p3;
    shm_cmd.sequence_id    = ++seq;
    shm_cmd.valid          = true;

    if (g_ipc_backend != "shm") {
        publish_via_bus<drone::ipc::GCSCommand>(drone::ipc::topics::GCS_COMMANDS, shm_cmd);
    } else {
        ShmWriter<drone::ipc::GCSCommand> writer;
        if (!writer.attach(drone::ipc::topics::GCS_COMMANDS)) {
            print_err("Cannot attach to SHM segment " +
                      std::string(drone::ipc::topics::GCS_COMMANDS) +
                      ". Ensure the stack is running.");
            return 1;
        }
        writer.write(shm_cmd);
    }

    std::ostringstream oss;
    oss << "GCS command injected: " << cmd << " (type=" << static_cast<int>(gcs_type)
        << ", p1=" << p1 << ", p2=" << p2 << ", p3=" << p3 << ", seq=" << seq << ")";
    print_ok(oss.str());
    return 0;
}

// ═══════════════════════════════════════════════════════════════
// Command: thermal_zone <0-3>
// Sets thermal zone override in /fault_overrides.
// ═══════════════════════════════════════════════════════════════
static int cmd_thermal_zone(uint8_t zone) {
    if (zone > 3) {
        print_err("Thermal zone must be 0-3");
        return 1;
    }

    float temps[] = {50.0f, 75.0f, 88.0f, 98.0f};
    update_overrides([&](drone::ipc::FaultOverrides& ovr) {
        ovr.thermal_zone      = static_cast<int32_t>(zone);
        ovr.cpu_temp_override = temps[zone];
    });

    std::ostringstream oss;
    oss << "Thermal zone override: " << static_cast<int>(zone) << " (temp=" << temps[zone] << "°C)";
    print_ok(oss.str());
    return 0;
}

// ═══════════════════════════════════════════════════════════════
// Command: mission_upload <json_file>
// Uploads new waypoints via /mission_upload channel.
// JSON format: {"waypoints": [{"x":..,"y":..,"z":..,"yaw":..},...]}
// ═══════════════════════════════════════════════════════════════
static int cmd_mission_upload(const std::string& json_file) {
    std::ifstream ifs(json_file);
    if (!ifs) {
        print_err("Cannot open file: " + json_file);
        return 1;
    }

    json j;
    try {
        ifs >> j;
    } catch (const json::exception& e) {
        print_err("JSON parse error: " + std::string(e.what()));
        return 1;
    }

    if (!j.contains("waypoints") || !j["waypoints"].is_array()) {
        print_err("JSON must contain a 'waypoints' array");
        return 1;
    }

    auto& wps = j["waypoints"];
    if (wps.size() > drone::ipc::kMaxUploadWaypoints) {
        print_err("Too many waypoints (max " + std::to_string(drone::ipc::kMaxUploadWaypoints) +
                  ")");
        return 1;
    }

    drone::ipc::MissionUpload upload{};
    upload.timestamp_ns   = now_ns();
    upload.correlation_id = now_ns();
    upload.num_waypoints  = static_cast<uint8_t>(wps.size());
    upload.valid          = true;

    for (size_t i = 0; i < wps.size(); ++i) {
        auto& wp                            = wps[i];
        upload.waypoints[i].x               = wp.value("x", 0.0f);
        upload.waypoints[i].y               = wp.value("y", 0.0f);
        upload.waypoints[i].z               = wp.value("z", 5.0f);
        upload.waypoints[i].yaw             = wp.value("yaw", 0.0f);
        upload.waypoints[i].radius          = wp.value("radius", 2.0f);
        upload.waypoints[i].speed           = wp.value("speed", 2.0f);
        upload.waypoints[i].trigger_payload = wp.value("trigger_payload", false);
    }

    // Also inject a MISSION_UPLOAD GCS command to trigger the planner
    drone::ipc::GCSCommand shm_cmd{};
    shm_cmd.timestamp_ns   = now_ns();
    shm_cmd.correlation_id = upload.correlation_id;
    shm_cmd.command        = drone::ipc::GCSCommandType::MISSION_UPLOAD;
    shm_cmd.param1         = static_cast<float>(upload.num_waypoints);
    shm_cmd.sequence_id    = 1;
    shm_cmd.valid          = true;

    if (g_ipc_backend != "shm") {
        publish_via_bus<drone::ipc::MissionUpload>(drone::ipc::topics::MISSION_UPLOAD, upload);
        publish_via_bus<drone::ipc::GCSCommand>(drone::ipc::topics::GCS_COMMANDS, shm_cmd);
    } else {
        ShmWriter<drone::ipc::MissionUpload> writer;
        if (!writer.attach(drone::ipc::topics::MISSION_UPLOAD)) {
            print_err("Cannot attach to SHM segment " +
                      std::string(drone::ipc::topics::MISSION_UPLOAD) +
                      ". Ensure the stack is running.");
            return 1;
        }
        writer.write(upload);

        ShmWriter<drone::ipc::GCSCommand> gcs_writer;
        if (gcs_writer.attach(drone::ipc::topics::GCS_COMMANDS)) {
            gcs_writer.write(shm_cmd);
        }
    }

    std::ostringstream oss;
    oss << "Mission uploaded: " << wps.size() << " waypoints from " << json_file;
    print_ok(oss.str());
    return 0;
}

// ═══════════════════════════════════════════════════════════════
// Command: sequence <json_file>
// Executes a timed sequence of fault injections.
// JSON format:
// {
//   "steps": [
//     {"delay_s": 0, "action": "battery", "value": 50.0},
//     {"delay_s": 5, "action": "battery", "value": 25.0},
//     {"delay_s": 5, "action": "fc_disconnect"},
//     {"delay_s": 10, "action": "fc_reconnect"},
//     {"delay_s": 2, "action": "gcs_command", "command": "rtl"}
//   ]
// }
// ═══════════════════════════════════════════════════════════════
static int execute_step(const json& step) {
    std::string action = step.value("action", "");

    if (action == "battery") {
        return cmd_battery(step.value("value", 50.0f));
    } else if (action == "fc_disconnect") {
        return cmd_fc_link(false);
    } else if (action == "fc_reconnect") {
        return cmd_fc_link(true);
    } else if (action == "gcs_command") {
        return cmd_gcs_command(step.value("command", "none"), step.value("param1", 0.0f),
                               step.value("param2", 0.0f), step.value("param3", 0.0f));
    } else if (action == "thermal_zone") {
        return cmd_thermal_zone(step.value("value", 0));
    } else if (action == "mission_upload") {
        return cmd_mission_upload(step.value("file", ""));
    } else {
        print_err("Unknown action in sequence: " + action);
        return 1;
    }
}

static int cmd_sequence(const std::string& json_file) {
    std::ifstream ifs(json_file);
    if (!ifs) {
        print_err("Cannot open sequence file: " + json_file);
        return 1;
    }

    json j;
    try {
        ifs >> j;
    } catch (const json::exception& e) {
        print_err("JSON parse error: " + std::string(e.what()));
        return 1;
    }

    if (!j.contains("steps") || !j["steps"].is_array()) {
        print_err("Sequence JSON must contain a 'steps' array");
        return 1;
    }

    auto& steps = j["steps"];
    print_info("Executing fault sequence: " + std::to_string(steps.size()) + " steps");

    int errors = 0;
    for (size_t i = 0; i < steps.size(); ++i) {
        auto& step    = steps[i];
        float delay_s = step.value("delay_s", 0.0f);

        if (delay_s > 0) {
            std::ostringstream oss;
            oss << "Waiting " << delay_s << "s before step " << (i + 1);
            print_info(oss.str());
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(delay_s * 1000)));
        }

        std::ostringstream oss;
        oss << "Step " << (i + 1) << "/" << steps.size() << ": " << step.value("action", "???");
        print_info(oss.str());

        if (execute_step(step) != 0) {
            ++errors;
        }
    }

    if (errors == 0) {
        print_ok("Sequence completed: " + std::to_string(steps.size()) + " steps, 0 errors");
    } else {
        print_err("Sequence completed with " + std::to_string(errors) + " errors");
    }
    return errors > 0 ? 1 : 0;
}

// ═══════════════════════════════════════════════════════════════
// Usage / Help
// ═══════════════════════════════════════════════════════════════
static void print_usage(const char* argv0) {
    std::cout << YELLOW << "Fault Injector — IPC-based fault injection for integration testing\n"
              << NC << "\nUsage:\n"
              << "  " << argv0 << " [--ipc <shm|zenoh>] <command> [args...]\n"
              << "\nCommands:\n"
              << "  battery <percent>            Override FC battery level\n"
              << "  fc_disconnect                 Simulate FC link loss\n"
              << "  fc_reconnect                  Restore FC link\n"
              << "  gcs_command <cmd> [p1 p2 p3]  Inject GCS command\n"
              << "  thermal_zone <0-3>            Override thermal zone\n"
              << "  mission_upload <json_file>     Upload new waypoints\n"
              << "  sequence <json_file>           Timed fault sequence\n"
              << "\nOptions:\n"
              << "  --ipc <shm|zenoh>  IPC backend (default: shm)\n"
              << "\nGCS commands: arm, disarm, takeoff, land, rtl,\n"
              << "              mission_start, mission_pause, mission_abort, mission_upload\n"
              << "\nExamples:\n"
              << "  " << argv0 << " battery 25                # Trigger battery WARN\n"
              << "  " << argv0 << " --ipc zenoh gcs_command rtl  # Send RTL via Zenoh\n"
              << "  " << argv0 << " sequence fault_seq.json      # Run timed sequence\n"
              << std::endl;
}

// ═══════════════════════════════════════════════════════════════
// Numeric parse helpers — return nonzero exit on invalid input
// ═══════════════════════════════════════════════════════════════
static bool safe_stof(const char* s, float& out, const char* label) {
    try {
        size_t pos = 0;
        out        = std::stof(s, &pos);
        if (pos != std::string(s).size()) throw std::invalid_argument("trailing chars");
        return true;
    } catch (const std::exception&) {
        print_err(std::string("Invalid ") + label + ": '" + s + "' (expected a number)");
        return false;
    }
}

static bool safe_stoi(const char* s, int& out, const char* label) {
    try {
        size_t pos = 0;
        out        = std::stoi(s, &pos);
        if (pos != std::string(s).size()) throw std::invalid_argument("trailing chars");
        return true;
    } catch (const std::exception&) {
        print_err(std::string("Invalid ") + label + ": '" + s + "' (expected an integer)");
        return false;
    }
}

// ═══════════════════════════════════════════════════════════════
// Main entry
// ═══════════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    // Parse --ipc <backend> before command dispatch
    const char* argv0      = argv[0];  // preserve for usage output after argv shift
    int         arg_offset = 1;
    if (std::string(argv[1]) == "--ipc") {
        if (argc < 3) {
            print_err("--ipc requires an argument: shm or zenoh");
            return 1;
        }
        g_ipc_backend = argv[2];
        if (g_ipc_backend != "shm" && g_ipc_backend != "zenoh") {
            print_err("Invalid IPC backend '" + g_ipc_backend + "' (must be 'shm' or 'zenoh')");
            return 1;
        }
#ifndef HAVE_ZENOH
        if (g_ipc_backend == "zenoh") {
            print_err("Zenoh backend requested but not compiled in (HAVE_ZENOH not defined)");
            return 1;
        }
#endif
        arg_offset = 3;
        if (argc < 4) {
            print_usage(argv[0]);
            return 1;
        }
    }

    // Shift argv/argc past --ipc <backend>
    argc -= (arg_offset - 1);
    argv += (arg_offset - 1);

    std::string cmd = argv[1];

    if (cmd == "battery") {
        if (argc < 3) {
            print_err("Usage: fault_injector battery <percent>");
            return 1;
        }
        float pct = 0.0f;
        if (!safe_stof(argv[2], pct, "battery percent")) return 1;
        return cmd_battery(pct);

    } else if (cmd == "fc_disconnect") {
        return cmd_fc_link(false);

    } else if (cmd == "fc_reconnect") {
        return cmd_fc_link(true);

    } else if (cmd == "gcs_command") {
        if (argc < 3) {
            print_err("Usage: fault_injector gcs_command <cmd> [p1 p2 p3]");
            return 1;
        }
        float p1 = 0.0f, p2 = 0.0f, p3 = 0.0f;
        if (argc > 3 && !safe_stof(argv[3], p1, "param1")) return 1;
        if (argc > 4 && !safe_stof(argv[4], p2, "param2")) return 1;
        if (argc > 5 && !safe_stof(argv[5], p3, "param3")) return 1;
        return cmd_gcs_command(argv[2], p1, p2, p3);

    } else if (cmd == "thermal_zone") {
        if (argc < 3) {
            print_err("Usage: fault_injector thermal_zone <0-3>");
            return 1;
        }
        int zone = 0;
        if (!safe_stoi(argv[2], zone, "thermal zone")) return 1;
        return cmd_thermal_zone(static_cast<uint8_t>(zone));

    } else if (cmd == "mission_upload") {
        if (argc < 3) {
            print_err("Usage: fault_injector mission_upload <json_file>");
            return 1;
        }
        return cmd_mission_upload(argv[2]);

    } else if (cmd == "sequence") {
        if (argc < 3) {
            print_err("Usage: fault_injector sequence <json_file>");
            return 1;
        }
        return cmd_sequence(argv[2]);

    } else if (cmd == "--help" || cmd == "-h" || cmd == "help") {
        print_usage(argv0);
        return 0;

    } else {
        print_err("Unknown command: " + cmd);
        print_usage(argv0);
        return 1;
    }
}
