// tools/cosys_populate_scene/main.cpp
// ═══════════════════════════════════════════════════════════════════
// Cosys-AirSim scene populator — spawns/destroys objects listed in a
// simulator-agnostic scene JSON via the simSpawnObject RPC.
//
// Part of epic #480 (proving-ground). This is the Cosys adapter; the
// same scene JSON schema will drive other-sim adapters in Phase D.
//
// Usage:
//   cosys_populate_scene spawn   --scene <path.json> --stamp <path.txt>
//                                [--host 127.0.0.1] [--port 41451]
//   cosys_populate_scene destroy --stamp <path.txt>
//                                [--host 127.0.0.1] [--port 41451]
//
// Scene JSON schema:
//   { "objects": [
//       { "name": "pillar_01", "asset": "Pillar_50x500",
//         "x": 8, "y": 12, "z": 0, "yaw_deg": 0,
//         "scale": [2, 2, 3],             // or a scalar: "scale": 2.0
//         "physics_enabled": false },     // optional, default false
//       ...
//   ] }
//
// Exit codes: 0 success (or all destroys attempted), 1 partial failure
// (some spawns/destroys threw), 2 setup failure (bad args, no connection).
// ═══════════════════════════════════════════════════════════════════
#define HAVE_COSYS_AIRSIM 1
#include "hal/cosys_rpc_client.h"

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace {

struct Args {
    std::string mode;   // "spawn" | "destroy"
    std::string scene;  // scene JSON path (spawn only)
    std::string stamp;  // names-written file
    std::string host = "127.0.0.1";
    uint16_t    port = 41451;
};

void print_usage(std::ostream& os) {
    os << "Usage:\n"
       << "  cosys_populate_scene spawn   --scene <path.json> --stamp <path.txt> "
          "[--host H] [--port P]\n"
       << "  cosys_populate_scene destroy --stamp <path.txt> [--host H] [--port P]\n";
}

bool parse_args(int argc, char** argv, Args& out) {
    if (argc < 2) {
        print_usage(std::cerr);
        return false;
    }
    const std::string first = argv[1];
    if (first == "-h" || first == "--help") {
        print_usage(std::cout);
        return false;
    }
    out.mode = first;
    if (out.mode != "spawn" && out.mode != "destroy") {
        std::cerr << "Unknown mode: " << out.mode << "\n";
        print_usage(std::cerr);
        return false;
    }
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        auto              need_next = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value after " << flag << "\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "--scene") {
            auto v = need_next("--scene");
            if (!v) return false;
            out.scene = v;
        } else if (a == "--stamp") {
            auto v = need_next("--stamp");
            if (!v) return false;
            out.stamp = v;
        } else if (a == "--host") {
            auto v = need_next("--host");
            if (!v) return false;
            out.host = v;
        } else if (a == "--port") {
            auto v = need_next("--port");
            if (!v) return false;
            out.port = static_cast<uint16_t>(std::stoi(v));
        } else if (a == "-h" || a == "--help") {
            print_usage(std::cout);
            return false;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            print_usage(std::cerr);
            return false;
        }
    }
    if (out.mode == "spawn" && out.scene.empty()) {
        std::cerr << "spawn requires --scene\n";
        return false;
    }
    if (out.stamp.empty()) {
        std::cerr << "missing --stamp\n";
        return false;
    }
    return true;
}

// NED ↔ our coord note: scene JSON uses +X=North, +Y=East, +Z=Up (same convention
// as scenario waypoints). AirSim internally uses NED (+Z=Down). Convert by
// negating Z when handing to AirSim.
msr::airlib::Pose make_pose(double x, double y, double z, double yaw_rad) {
    using namespace msr::airlib;
    Vector3r pos(static_cast<float>(x), static_cast<float>(y), static_cast<float>(-z));
    // Yaw about +Z (world up); AirSim quaternion uses NED so rotate about -Z.
    const float            half = static_cast<float>(-yaw_rad * 0.5);
    Quaternionr            q(std::cos(half), 0.f, 0.f, std::sin(half));
    return Pose(pos, q);
}

msr::airlib::Vector3r parse_scale(const json& obj) {
    using namespace msr::airlib;
    if (!obj.contains("scale")) {
        return Vector3r(1.f, 1.f, 1.f);
    }
    const auto& s = obj["scale"];
    if (s.is_number()) {
        const float f = s.get<float>();
        return Vector3r(f, f, f);
    }
    if (s.is_array() && s.size() == 3) {
        return Vector3r(s[0].get<float>(), s[1].get<float>(), s[2].get<float>());
    }
    std::cerr << "  WARN: scale must be scalar or [x,y,z], got "
              << s.dump() << " — defaulting to 1.0\n";
    return Vector3r(1.f, 1.f, 1.f);
}

int do_spawn(const Args& args) {
    std::ifstream ifs(args.scene);
    if (!ifs) {
        std::cerr << "Cannot open scene file: " << args.scene << "\n";
        return 2;
    }
    json scene;
    try {
        ifs >> scene;
    } catch (const std::exception& e) {
        std::cerr << "Scene JSON parse error: " << e.what() << "\n";
        return 2;
    }
    if (!scene.contains("objects") || !scene["objects"].is_array()) {
        std::cerr << "Scene file missing 'objects' array\n";
        return 2;
    }

    drone::hal::CosysRpcClient client(args.host, args.port);
    if (!client.connect()) {
        std::cerr << "Failed to connect to Cosys-AirSim at "
                  << args.host << ":" << args.port << "\n";
        return 2;
    }
    std::cout << "Connected to " << client.endpoint() << "\n";

    std::ofstream stamp(args.stamp, std::ios::out | std::ios::trunc);
    if (!stamp) {
        std::cerr << "Cannot write stamp file: " << args.stamp << "\n";
        return 2;
    }

    int spawned = 0;
    int failed  = 0;
    for (const auto& obj : scene["objects"]) {
        const std::string name  = obj.value("name", "");
        const std::string asset = obj.value("asset", "");
        if (name.empty() || asset.empty()) {
            std::cerr << "  skip: object missing name or asset: " << obj.dump() << "\n";
            ++failed;
            continue;
        }
        const double x       = obj.value("x", 0.0);
        const double y       = obj.value("y", 0.0);
        const double z       = obj.value("z", 0.0);
        const double yaw_deg = obj.value("yaw_deg", 0.0);
        const bool   physics = obj.value("physics_enabled", false);
        const auto   pose    = make_pose(x, y, z, yaw_deg * (M_PI / 180.0));
        const auto   scale   = parse_scale(obj);

        bool ok = client.with_client([&](auto& rpc) {
            try {
                const auto returned = rpc.simSpawnObject(name, asset, pose, scale, physics);
                std::cout << "  + " << name << "  <-  " << asset << "  at ("
                          << x << ", " << y << ", " << z << ", yaw=" << yaw_deg << "°)"
                          << "  -> " << returned << "\n";
                stamp << returned << "\n";
                ++spawned;
            } catch (const std::exception& e) {
                std::cerr << "  ! spawn failed for " << name << " (" << asset << "): "
                          << e.what() << "\n";
                ++failed;
            }
        });
        if (!ok) {
            std::cerr << "  ! RPC client disconnected while spawning " << name << "\n";
            ++failed;
        }
    }

    stamp.flush();
    std::cout << "Spawned " << spawned << " / " << (spawned + failed) << " objects\n";
    client.disconnect();
    return failed == 0 ? 0 : 1;
}

int do_destroy(const Args& args) {
    std::ifstream ifs(args.stamp);
    if (!ifs) {
        // Stamp file missing is not fatal — nothing to destroy.
        std::cout << "No stamp file at " << args.stamp << " — nothing to destroy\n";
        return 0;
    }
    std::vector<std::string> names;
    std::string              line;
    while (std::getline(ifs, line)) {
        if (!line.empty()) names.push_back(line);
    }
    if (names.empty()) {
        std::cout << "Stamp file empty — nothing to destroy\n";
        return 0;
    }

    drone::hal::CosysRpcClient client(args.host, args.port);
    if (!client.connect()) {
        std::cerr << "Failed to connect — stamp file left in place for retry\n";
        return 2;
    }

    int destroyed = 0;
    int failed    = 0;
    for (const auto& n : names) {
        bool ok = client.with_client([&](auto& rpc) {
            try {
                const bool r = rpc.simDestroyObject(n);
                std::cout << "  - " << n << (r ? "  destroyed" : "  (not found)") << "\n";
                ++destroyed;
            } catch (const std::exception& e) {
                std::cerr << "  ! destroy failed for " << n << ": " << e.what() << "\n";
                ++failed;
            }
        });
        if (!ok) {
            std::cerr << "  ! RPC client disconnected while destroying " << n << "\n";
            ++failed;
        }
    }

    client.disconnect();
    std::cout << "Destroyed " << destroyed << " / " << names.size()
              << " objects (" << failed << " failed)\n";

    // Remove the stamp file on fully-successful teardown.
    if (failed == 0) {
        std::remove(args.stamp.c_str());
    }
    return failed == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parse_args(argc, argv, args)) {
        return 2;
    }
    return args.mode == "spawn" ? do_spawn(args) : do_destroy(args);
}
