// common/util/include/util/arg_parser.h
// Simple command-line argument parser for all processes.
#pragma once
#include <string>
#include <cstring>

struct ParsedArgs {
    std::string config_path = "config/default.json";
    std::string log_level   = "info";
    bool help               = false;
    bool simulation         = false;
};

inline ParsedArgs parse_args(int argc, char* argv[], const char* process_name) {
    ParsedArgs args;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            args.help = true;
            std::printf("Usage: %s [OPTIONS]\n", process_name);
            std::printf("  --config <path>    Config file path\n");
            std::printf("  --log-level <lvl>  Log level (trace/debug/info/warn/error)\n");
            std::printf("  --sim              Run in simulation mode\n");
            std::printf("  --help             Show this help\n");
        } else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            args.config_path = argv[++i];
        } else if (std::strcmp(argv[i], "--log-level") == 0 && i + 1 < argc) {
            args.log_level = argv[++i];
        } else if (std::strcmp(argv[i], "--sim") == 0) {
            args.simulation = true;
        }
    }
    return args;
}
