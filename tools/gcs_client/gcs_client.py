#!/usr/bin/env python3
"""
GCS Client — Ground Control Station subscriber for the drone companion stack.

Connects to the drone's Zenoh network endpoint (TCP) as a client and
subscribes to all drone/* key expressions, decoding wire-format headers
and printing telemetry to stdout.

Usage:
    python3 gcs_client.py --drone-ip 192.168.1.10 --port 7447

Dependencies:
    pip install eclipse-zenoh

Wire Format (24-byte packed header):
    [0..3]   magic         — 0x4E4F5244 ("DRON" LE)
    [4]      version       — 1
    [5]      flags         — reserved (0)
    [6..7]   msg_type      — WireMessageType enum (LE uint16)
    [8..11]  payload_size  — bytes following header
    [12..19] timestamp_ns  — sender's steady_clock timestamp
    [20..23] sequence      — per-topic monotonic counter
"""

import argparse
import json
import signal
import struct
import sys
import time
from datetime import datetime

try:
    import zenoh
except ImportError:
    print("ERROR: 'eclipse-zenoh' package required.  Install with:")
    print("  pip install eclipse-zenoh")
    sys.exit(1)

# ── Wire format constants ────────────────────────────────────────────────────
WIRE_MAGIC = 0x4E4F5244  # "DRON" little-endian
WIRE_VERSION = 1
WIRE_HEADER_SIZE = 24
WIRE_HEADER_FMT = "<IBBHIQ"  # magic(4) ver(1) flags(1) type(2) size(4) ts(8) seq(4) = 24

# Message type names (matches WireMessageType enum in wire_format.h)
MSG_TYPE_NAMES = {
    0:  "UNKNOWN",
    1:  "VIDEO_FRAME",
    2:  "STEREO_FRAME",
    10: "DETECTIONS",
    20: "SLAM_POSE",
    30: "MISSION_STATUS",
    31: "TRAJECTORY_CMD",
    32: "PAYLOAD_COMMAND",
    33: "FC_COMMAND",
    40: "FC_STATE",
    41: "GCS_COMMAND",
    50: "PAYLOAD_STATUS",
    60: "SYSTEM_HEALTH",
}

# ── Statistics ────────────────────────────────────────────────────────────────
stats = {
    "total_messages": 0,
    "total_bytes": 0,
    "invalid_headers": 0,
    "per_type": {},
    "start_time": time.monotonic(),
}


def decode_wire_header(data: bytes):
    """Decode a 24-byte wire header from raw bytes.

    Returns a dict with header fields, or None if validation fails.
    """
    if len(data) < WIRE_HEADER_SIZE:
        return None

    magic, version, flags, msg_type, payload_size, timestamp_ns, sequence = \
        struct.unpack_from(WIRE_HEADER_FMT, data, 0)

    if magic != WIRE_MAGIC:
        return None
    if version != WIRE_VERSION:
        return None

    return {
        "magic": magic,
        "version": version,
        "flags": flags,
        "msg_type": msg_type,
        "msg_type_name": MSG_TYPE_NAMES.get(msg_type, f"UNKNOWN({msg_type})"),
        "payload_size": payload_size,
        "timestamp_ns": timestamp_ns,
        "sequence": sequence,
    }


def format_bytes(n: int) -> str:
    """Human-readable byte count."""
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024:
            return f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} TB"


def print_stats():
    """Print accumulated statistics."""
    elapsed = time.monotonic() - stats["start_time"]
    print("\n╔══════════════════════════════════════════════╗")
    print("║             GCS Client Statistics            ║")
    print("╠══════════════════════════════════════════════╣")
    print(f"║  Duration        : {elapsed:.1f}s")
    print(f"║  Total messages  : {stats['total_messages']}")
    print(f"║  Total data      : {format_bytes(stats['total_bytes'])}")
    print(f"║  Invalid headers : {stats['invalid_headers']}")
    if elapsed > 0:
        print(f"║  Avg rate        : {stats['total_messages'] / elapsed:.1f} msg/s")
    print("║  ────────────────────────────────────────────")
    for type_name, count in sorted(stats["per_type"].items()):
        print(f"║  {type_name:20s} : {count}")
    print("╚══════════════════════════════════════════════╝")


def on_sample(sample):
    """Callback for every Zenoh sample received."""
    key = str(sample.key_expr)
    payload = bytes(sample.payload)
    data_len = len(payload)

    stats["total_messages"] += 1
    stats["total_bytes"] += data_len

    hdr = decode_wire_header(payload)
    if hdr is None:
        # Not wire-formatted — raw Zenoh message (e.g. local SHM mode)
        stats["invalid_headers"] += 1
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{ts}] {key}  raw={format_bytes(data_len)}")
        return

    type_name = hdr["msg_type_name"]
    stats["per_type"][type_name] = stats["per_type"].get(type_name, 0) + 1

    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(
        f"[{ts}] {key}  "
        f"type={type_name}  "
        f"seq={hdr['sequence']}  "
        f"payload={format_bytes(hdr['payload_size'])}  "
        f"wire_ts={hdr['timestamp_ns']}"
    )


def build_zenoh_config(drone_ip: str, port: int, protocol: str) -> str:
    """Build a Zenoh JSON5 config for GCS client mode."""
    endpoint = f"{protocol}/{drone_ip}:{port}"

    config = {
        "mode": "client",
        "connect": {
            "endpoints": [endpoint],
        },
        "scouting": {
            "multicast": {"enabled": False},
            "gossip": {"enabled": False},
        },
        "transport": {
            "unicast": {
                "max_links": 1,
            },
        },
    }
    return json.dumps(config)


def main():
    parser = argparse.ArgumentParser(
        description="GCS Client — subscribe to drone telemetry over Zenoh network",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Connect to drone on default port:
  python3 gcs_client.py --drone-ip 192.168.1.10

  # Connect on custom port with specific topics:
  python3 gcs_client.py --drone-ip 10.0.0.2 --port 7448 --key "drone/slam/*"

  # Subscribe to all topics (default):
  python3 gcs_client.py --drone-ip 192.168.1.10 --key "drone/**"
""",
    )
    parser.add_argument(
        "--drone-ip",
        required=True,
        help="IP address of the drone's Zenoh listener",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=7447,
        help="Zenoh TCP port on the drone (default: 7447)",
    )
    parser.add_argument(
        "--protocol",
        default="tcp",
        choices=["tcp", "udp", "tls"],
        help="Transport protocol (default: tcp)",
    )
    parser.add_argument(
        "--key",
        default="drone/**",
        help="Zenoh key expression to subscribe to (default: drone/**)",
    )
    parser.add_argument(
        "--stats-interval",
        type=float,
        default=0,
        help="Print stats every N seconds (0 = disabled)",
    )
    args = parser.parse_args()

    print("╔══════════════════════════════════════════════╗")
    print("║          GCS Client — Drone Telemetry        ║")
    print("╠══════════════════════════════════════════════╣")
    print(f"║  Drone    : {args.drone_ip}:{args.port}")
    print(f"║  Protocol : {args.protocol}")
    print(f"║  Key expr : {args.key}")
    print("╚══════════════════════════════════════════════╝")
    print()

    # Build and open Zenoh session in client mode
    config_json = build_zenoh_config(args.drone_ip, args.port, args.protocol)
    config = zenoh.Config.from_json5(config_json)

    print(f"Connecting to {args.protocol}/{args.drone_ip}:{args.port} ...")
    session = zenoh.open(config)
    print("Connected!  Subscribing to:", args.key)
    print("Press Ctrl+C to stop.\n")

    # Subscribe
    sub = session.declare_subscriber(args.key, on_sample)

    # Handle graceful shutdown
    running = True

    def signal_handler(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        last_stats = time.monotonic()
        while running:
            time.sleep(0.1)
            if args.stats_interval > 0:
                now = time.monotonic()
                if now - last_stats >= args.stats_interval:
                    print_stats()
                    last_stats = now
    finally:
        print("\nShutting down...")
        sub.undeclare()
        session.close()
        print_stats()


if __name__ == "__main__":
    main()
