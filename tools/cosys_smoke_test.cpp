// Cosys-AirSim smoke test — verify each sensor API works against a live UE5 server.
// Tests only what's configured in settings.json. Safe against wrong-name crashes
// (uses the exact names from the Blocks environment settings).
//
// Usage:
//   1. Start UE5 with Blocks, press Play (server listens on 41451)
//   2. Compile via tools/build_cosys_smoke_test.sh
//   3. Run the binary
#define HAVE_COSYS_AIRSIM 1
#include "hal/cosys_rpc_client.h"

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

int main() {
    using namespace msr::airlib;

    std::cout << "=== Cosys-AirSim Smoke Test ===" << std::endl;
    drone::hal::CosysRpcClient client("127.0.0.1", 41451);

    if (!client.connect()) {
        std::cerr << "FAILED: could not connect to AirSim" << std::endl;
        return 1;
    }
    std::cout << "[OK] Connected to " << client.endpoint() << std::endl;

    int passed = 0;
    int failed = 0;

    // ── Test 1: IMU ────────────────────────────────────────
    std::cout << "\n[1/3] IMU (getImuData, empty vehicle_name)" << std::endl;
    client.with_client([&](auto& rpc) {
        try {
            auto        imu = rpc.getImuData("imu", "");
            const float g_mag =
                std::sqrt(imu.linear_acceleration.x() * imu.linear_acceleration.x() +
                          imu.linear_acceleration.y() * imu.linear_acceleration.y() +
                          imu.linear_acceleration.z() * imu.linear_acceleration.z());
            std::cout << "  accel: [" << imu.linear_acceleration.x() << ", "
                      << imu.linear_acceleration.y() << ", " << imu.linear_acceleration.z()
                      << "] m/s^2 (magnitude " << g_mag << ")" << std::endl;
            if (g_mag > 9.0f && g_mag < 10.5f) {
                std::cout << "  [PASS] gravity magnitude within expected range" << std::endl;
                ++passed;
            } else {
                std::cerr << "  [FAIL] gravity magnitude out of range" << std::endl;
                ++failed;
            }
        } catch (const std::exception& e) {
            std::cerr << "  [FAIL] " << e.what() << std::endl;
            ++failed;
        }
    });

    // ── Test 2: Camera (RGB Scene) ──────────────────────────
    // NOTE: Pass vehicle_name — wrong/missing name has caused UE5 crashes in the past
    // (ASimModeBase::getCamera segfault when camera not found).
    std::cout << "\n[2/3] Camera RGB (simGetImages Scene, front_center, Drone0)" << std::endl;
    client.with_client([&](auto& rpc) {
        try {
            std::vector<ImageCaptureBase::ImageRequest> requests = {
                ImageCaptureBase::ImageRequest("front_center", ImageCaptureBase::ImageType::Scene,
                                               /*pixels_as_float*/ false, /*compress*/ false)};
            auto responses = rpc.simGetImages(requests, "");
            if (responses.empty()) {
                std::cerr << "  [FAIL] empty response" << std::endl;
                ++failed;
                return;
            }
            const auto& r = responses[0];
            std::cout << "  dimensions: " << r.width << "x" << r.height << std::endl;
            std::cout << "  uint8 bytes: " << r.image_data_uint8.size() << std::endl;
            const size_t expected = static_cast<size_t>(r.width) * r.height * 3;
            if (r.width > 0 && r.height > 0 && r.image_data_uint8.size() == expected) {
                std::cout << "  [PASS] " << r.width << "x" << r.height << " RGB" << std::endl;
                ++passed;
            } else {
                std::cerr << "  [FAIL] unexpected size (expected " << expected << ")" << std::endl;
                ++failed;
            }
        } catch (const std::exception& e) {
            std::cerr << "  [FAIL] " << e.what() << std::endl;
            ++failed;
        }
    });

    // ── Test 3: Depth ───────────────────────────────────────
    std::cout << "\n[3/3] Depth (simGetImages DepthPerspective, front_center, Drone0)" << std::endl;
    client.with_client([&](auto& rpc) {
        try {
            std::vector<ImageCaptureBase::ImageRequest> requests  = {ImageCaptureBase::ImageRequest(
                "front_center", ImageCaptureBase::ImageType::DepthPerspective,
                /*pixels_as_float*/ true, /*compress*/ false)};
            auto                                        responses = rpc.simGetImages(requests, "");
            if (responses.empty()) {
                std::cerr << "  [FAIL] empty response" << std::endl;
                ++failed;
                return;
            }
            const auto& r = responses[0];
            std::cout << "  dimensions: " << r.width << "x" << r.height << std::endl;
            std::cout << "  float pixels: " << r.image_data_float.size() << std::endl;
            if (!r.image_data_float.empty()) {
                float min_d = r.image_data_float[0];
                float max_d = r.image_data_float[0];
                for (float d : r.image_data_float) {
                    if (d < min_d) min_d = d;
                    if (d > max_d) max_d = d;
                }
                std::cout << "  depth range: [" << min_d << ", " << max_d << "] m" << std::endl;
                if (max_d > min_d && max_d > 0.1f) {
                    std::cout << "  [PASS] depth varies across scene" << std::endl;
                    ++passed;
                } else {
                    std::cerr << "  [FAIL] flat depth" << std::endl;
                    ++failed;
                }
            } else {
                std::cerr << "  [FAIL] no float data" << std::endl;
                ++failed;
            }
        } catch (const std::exception& e) {
            std::cerr << "  [FAIL] " << e.what() << std::endl;
            ++failed;
        }
    });

    // LiDAR test skipped — requires settings.json Sensors: { "LidarSensor1": {...} }
    std::cout << "\n[skip] LiDAR — add to settings.json then re-run for radar-proxy test"
              << std::endl;

    std::cout << "\n=== " << passed << " passed, " << failed << " failed ===" << std::endl;
    client.disconnect();
    return failed == 0 ? 0 : 1;
}
