// tests/benchmark/compare_to_baseline_main.cpp
//
// CLI tool: compare current perception run metrics against a stored baseline.
// Exits 0 if all metrics are within thresholds, 1 if any regression detected,
// 2 on usage/parse error.
//
// Usage:
//   compare_to_baseline --baseline benchmarks/baseline.json --current run.json
//   compare_to_baseline --baseline bl.json --current run.json --recall-threshold 0.10

#include "benchmark/baseline_capture.h"
#include "benchmark/baseline_comparator.h"

#include <charconv>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

namespace {

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " --baseline <path> --current <path> [options]\n\n"
              << "Options:\n"
              << "  --recall-threshold <pct>     Max recall drop     (default: 0.05)\n"
              << "  --precision-threshold <pct>  Max precision drop  (default: 0.05)\n"
              << "  --ap-threshold <pct>         Max mAP drop        (default: 0.05)\n"
              << "  --mota-threshold <pct>       Max MOTA drop       (default: 0.05)\n"
              << "  --motp-threshold <pct>       Max MOTP drop       (default: 0.05)\n"
              << "  --latency-threshold <pct>    Max latency p95 rise (default: 0.20)\n"
              << "  --help                       Show this message\n";
}

bool parse_threshold(const char* str, double& out) {
    std::string_view sv(str);
    auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), out);
    if (ec != std::errc{} || ptr != sv.data() + sv.size()) {
        std::cerr << "Error: invalid threshold value: " << str << "\n";
        return false;
    }
    if (out <= 0.0 || out > 1.0) {
        std::cerr << "Error: threshold must be in (0.0, 1.0]: " << str << "\n";
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string                            baseline_path;
    std::string                            current_path;
    drone::benchmark::ComparisonThresholds thresholds;

    for (int i = 1; i < argc; ++i) {
        std::string_view arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        if (arg == "--baseline" && i + 1 < argc) {
            baseline_path = argv[++i];
        } else if (arg == "--current" && i + 1 < argc) {
            current_path = argv[++i];
        } else if (arg == "--recall-threshold" && i + 1 < argc) {
            if (!parse_threshold(argv[++i], thresholds.recall_drop_pct)) return 2;
        } else if (arg == "--precision-threshold" && i + 1 < argc) {
            if (!parse_threshold(argv[++i], thresholds.precision_drop_pct)) return 2;
        } else if (arg == "--ap-threshold" && i + 1 < argc) {
            if (!parse_threshold(argv[++i], thresholds.ap_drop_pct)) return 2;
        } else if (arg == "--mota-threshold" && i + 1 < argc) {
            if (!parse_threshold(argv[++i], thresholds.mota_drop_pct)) return 2;
        } else if (arg == "--motp-threshold" && i + 1 < argc) {
            if (!parse_threshold(argv[++i], thresholds.motp_drop_pct)) return 2;
        } else if (arg == "--latency-threshold" && i + 1 < argc) {
            if (!parse_threshold(argv[++i], thresholds.latency_rise_pct)) return 2;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage(argv[0]);
            return 2;
        }
    }

    if (baseline_path.empty() || current_path.empty()) {
        std::cerr << "Error: --baseline and --current are required\n\n";
        print_usage(argv[0]);
        return 2;
    }

    drone::benchmark::BaselineCapture baseline;
    if (!baseline.load_json(baseline_path)) {
        std::cerr << "Error: could not load baseline: " << baseline_path << "\n";
        return 2;
    }

    drone::benchmark::BaselineCapture current;
    if (!current.load_json(current_path)) {
        std::cerr << "Error: could not load current run: " << current_path << "\n";
        return 2;
    }

    const auto result = drone::benchmark::compare_baselines(baseline, current, thresholds);
    std::cout << drone::benchmark::format_comparison(result);

    return result.passed ? 0 : 1;
}
