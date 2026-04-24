// tools/cosys_telemetry_poller/main.cpp
// ═══════════════════════════════════════════════════════════════════
// Cosys-AirSim sim-side telemetry poller for scenario runs.
//
// Polls, via the Cosys-AirSim RPC, at 10 Hz:
//   - simGetCollisionInfo(vehicle)     → collision events (has_collided,
//                                          impact point, normal, object hit,
//                                          penetration depth, count)
//   - simGetGroundTruthKinematics(..)  → true drone pose (position +
//                                          orientation + linear/angular vel),
//                                          independent of SLAM drift
//
// Output: JSONL, one record per tick, to a file given on the command line.
// Complements the perception-side voxel trace (Issue #612) by giving us
// the ground-truth frame to compare SLAM pose against AND the UE5 physics-
// level collision ledger.
//
// Usage:
//   cosys_telemetry_poller --out drone_logs/<run>/cosys_telemetry.jsonl
//                          [--collisions drone_logs/<run>/collisions.log]
//                          [--vehicle Drone0] [--host 127.0.0.1]
//                          [--port 41451] [--rate_hz 10]
//
// Stops on SIGINT/SIGTERM — scenario runner sends SIGTERM at shutdown.
// Exit codes: 0 normal, 1 RPC setup failure, 2 bad args.
// ═══════════════════════════════════════════════════════════════════
#define HAVE_COSYS_AIRSIM 1
// Include order matters — cosys_rpc_client.h pulls in the base header set
// AirLib's RpcLibAdaptorsBase depends on.  Same pattern as
// tools/cosys_populate_scene/main.cpp.
#include "hal/cosys_rpc_client.h"

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include "api/RpcLibAdaptorsBase.hpp"

#include <rpc/client.h>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

#include <atomic>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

// Lock-free across the signal handler / main loop boundary.
static_assert(std::atomic<bool>::is_always_lock_free,
              "cosys_telemetry_poller requires a lock-free atomic<bool> for the signal handler");
std::atomic<bool> g_running{true};

void on_signal(int) {
    g_running.store(false, std::memory_order_release);
}

/// Parse an integer from a C string with range validation.  Replaces
/// `std::atoi` (forbidden by CLAUDE.md — silent 0 on bad input).
std::optional<int> parse_int(const char* s, int min_val, int max_val) {
    if (s == nullptr || *s == '\0') return std::nullopt;
    std::string_view sv(s);
    int              out{0};
    auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), out);
    if (ec != std::errc{} || ptr != sv.data() + sv.size()) return std::nullopt;
    if (out < min_val || out > max_val) return std::nullopt;
    return out;
}

uint64_t now_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

struct Args {
    std::string out_path{"drone_logs/cosys_telemetry.jsonl"};
    std::string collisions_path;  // optional — no path = no log
    std::string vehicle{"Drone0"};
    std::string host{"127.0.0.1"};
    uint16_t    port{41451};
    int         rate_hz{10};
};

bool parse_args(int argc, char** argv, Args& a) {
    for (int i = 1; i < argc; ++i) {
        std::string s    = argv[i];
        auto        next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "error: " << name << " requires an argument\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (s == "--out") {
            if (auto v = next("--out"))
                a.out_path = v;
            else
                return false;
        } else if (s == "--collisions") {
            if (auto v = next("--collisions"))
                a.collisions_path = v;
            else
                return false;
        } else if (s == "--vehicle") {
            if (auto v = next("--vehicle"))
                a.vehicle = v;
            else
                return false;
        } else if (s == "--host") {
            if (auto v = next("--host"))
                a.host = v;
            else
                return false;
        } else if (s == "--port") {
            auto v = next("--port");
            if (!v) return false;
            auto parsed = parse_int(v, 1, 65535);
            if (!parsed) {
                std::cerr << "error: --port must be an integer in [1, 65535]: " << v << "\n";
                return false;
            }
            a.port = static_cast<uint16_t>(*parsed);
        } else if (s == "--rate_hz") {
            auto v = next("--rate_hz");
            if (!v) return false;
            auto parsed = parse_int(v, 1, 1000);
            if (!parsed) {
                std::cerr << "error: --rate_hz must be an integer in [1, 1000]: " << v << "\n";
                return false;
            }
            a.rate_hz = *parsed;
        } else if (s == "-h" || s == "--help") {
            std::cout << "cosys_telemetry_poller — collision + ground-truth kinematics telemetry\n"
                      << "Options:\n"
                      << "  --out <path>         JSONL output (default "
                         "drone_logs/cosys_telemetry.jsonl)\n"
                      << "  --collisions <path>  Optional compact human-readable collision log\n"
                      << "  --vehicle <name>     Vehicle name (default Drone0)\n"
                      << "  --host <ip>          (default 127.0.0.1)\n"
                      << "  --port <port>        (default 41451)\n"
                      << "  --rate_hz <n>        Poll rate (default 10)\n";
            std::exit(0);
        } else {
            std::cerr << "warning: unknown arg: " << s << "\n";
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Args a;
    if (!parse_args(argc, argv, a)) return 2;

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::ofstream out(a.out_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << "[cosys_telemetry_poller] cannot open output '" << a.out_path << "'\n";
        return 2;
    }

    std::ofstream coll_log;
    if (!a.collisions_path.empty()) {
        coll_log.open(a.collisions_path, std::ios::out | std::ios::trunc);
    }

    // Connect to Cosys-AirSim RPC.
    msr::airlib::MultirotorRpcLibClient client(a.host, a.port, /*timeout_sec=*/5);
    try {
        client.confirmConnection();
    } catch (const std::exception& e) {
        std::cerr << "[cosys_telemetry_poller] RPC connect failed: " << e.what() << "\n";
        return 1;
    }
    std::cerr << "[cosys_telemetry_poller] Connected " << a.host << ":" << a.port << " vehicle='"
              << a.vehicle << "' rate=" << a.rate_hz << " Hz\n"
              << "[cosys_telemetry_poller] out=" << a.out_path;
    if (!a.collisions_path.empty()) std::cerr << " collisions=" << a.collisions_path;
    std::cerr << "\n";

    const auto period           = std::chrono::milliseconds(1000 / a.rate_hz);
    uint64_t   collision_count  = 0;
    uint64_t   last_coll_tstamp = 0;
    uint64_t   tick             = 0;
    auto       next_tick        = std::chrono::steady_clock::now();

    while (g_running.load(std::memory_order_acquire)) {
        json rec;
        rec["t_ns"] = now_ns();
        rec["tick"] = tick;

        // Collision info.  Cosys time_stamp is nanoseconds since engine start;
        // a fresh impact bumps the timestamp and has_collided stays true while
        // contact continues, so gate "new collision" on a timestamp change.
        try {
            auto       ci    = client.simGetCollisionInfo(a.vehicle);
            const bool fresh = (ci.time_stamp != last_coll_tstamp) && ci.has_collided;
            if (fresh) {
                ++collision_count;
                last_coll_tstamp = ci.time_stamp;
                if (coll_log.is_open()) {
                    coll_log << "[" << now_ns() << "] collision #" << collision_count;
                    coll_log << " with '" << ci.object_name << "' (id=" << ci.object_id << ")";
                    coll_log << " at (" << ci.impact_point.x() << ", " << ci.impact_point.y();
                    coll_log << ", " << ci.impact_point.z() << ")";
                    coll_log << " pen=" << ci.penetration_depth << "\n";
                    coll_log.flush();
                }
            }
            rec["coll"] = {
                {"has", ci.has_collided},
                {"count", collision_count},
                {"fresh", fresh},
                {"ts", static_cast<uint64_t>(ci.time_stamp)},
                {"obj", ci.object_name},
                {"id", ci.object_id},
                {"pt", {ci.impact_point.x(), ci.impact_point.y(), ci.impact_point.z()}},
                {"nrm", {ci.normal.x(), ci.normal.y(), ci.normal.z()}},
                {"pen", ci.penetration_depth},
            };
        } catch (const std::exception& e) {
            rec["coll_err"] = e.what();
        }

        // Ground-truth kinematics.  Independent of SLAM drift — lets us
        // cross-check the `camera_pose` the perception pipeline uses against
        // the simulator's real-world pose.
        //
        // Frame conversion: AirSim returns NED (Z-down).  Our SLAM publishes
        // ENU-like (Z-up) via CosysVIOBackend (see
        // process3_slam_vio_nav/include/slam/ivio_backend.h poll_loop:
        // `p.position = (x, y, -z)` and `q_enu = (w, x, -y, -z)`).  Apply the
        // same conversion here so the telemetry log lands in the SAME frame
        // that `plot_voxel_trace.py` expects when it diffs SLAM vs GT.
        // Without this, the pose-error plot showed a spurious 10 m offset
        // that was pure Z-sign artefact and masked real diagnostic work.
        try {
            auto k    = client.simGetGroundTruthKinematics(a.vehicle);
            rec["gt"] = {
                {"pos", {k.pose.position.x(), k.pose.position.y(), -k.pose.position.z()}},
                {"q",
                 {k.pose.orientation.w(), k.pose.orientation.x(), -k.pose.orientation.y(),
                  -k.pose.orientation.z()}},
                {"lv", {k.twist.linear.x(), k.twist.linear.y(), -k.twist.linear.z()}},
                {"av", {k.twist.angular.x(), k.twist.angular.y(), -k.twist.angular.z()}},
                {"la",
                 {k.accelerations.linear.x(), k.accelerations.linear.y(),
                  -k.accelerations.linear.z()}},
                {"aa",
                 {k.accelerations.angular.x(), k.accelerations.angular.y(),
                  -k.accelerations.angular.z()}},
            };
        } catch (const std::exception& e) {
            rec["gt_err"] = e.what();
        }

        out << rec.dump() << '\n';
        if ((tick % 20) == 0) out.flush();  // flush every ~2 s so `tail -f` works
        ++tick;

        next_tick += period;
        auto now = std::chrono::steady_clock::now();
        if (now > next_tick) next_tick = now;  // reset if we fell behind
        std::this_thread::sleep_until(next_tick);
    }

    out.flush();
    out.close();
    if (coll_log.is_open()) coll_log.close();
    std::cerr << "[cosys_telemetry_poller] Stopped — " << tick << " ticks, " << collision_count
              << " collisions\n";
    return 0;
}
