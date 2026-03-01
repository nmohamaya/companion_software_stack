# Network Transport — Drone ↔ GCS

> **Phase E** of the Zenoh IPC Migration (Issue #50)

## Overview

Phase E extends the Zenoh pub/sub infrastructure to support **network
transport** between a drone and a Ground Control Station (GCS). The same
key expressions used for local inter-process communication
(`drone/slam/pose`, `drone/comms/fc_state`, etc.) work transparently
over a TCP link to a remote GCS subscriber.

```
 ┌──────────────────────────────┐        TCP/7447        ┌──────────────┐
 │         DRONE (Peer)         │◄───────────────────────►│  GCS (Client)│
 │  P1  P2  P3  P4  P5  P6  P7 │                         │  gcs_client  │
 │      Zenoh Pub/Sub (SHM)     │                         │  (Python)    │
 └──────────────────────────────┘                         └──────────────┘
```

## Architecture

### Session Modes

| Mode | Config `mode` | Scouting | Endpoints | Use case |
|------|--------------|----------|-----------|----------|
| **Local** | `peer` | disabled | none | Development / single-machine |
| **Drone** | `peer` | enabled | `tcp/0.0.0.0:7447` (listen) | Flight hardware |
| **GCS** | `client` | disabled | `tcp/<drone-ip>:7447` (connect) | Ground station |

### Configuration

Network transport is configured in the application config JSON:

```json
{
  "ipc_backend": "zenoh",
  "zenoh": {
    "shm_pool_size_mb": 64,
    "network": {
      "enabled": true,
      "mode": "peer",
      "listen_port": 7447,
      "listen_address": "0.0.0.0",
      "protocol": "tcp",
      "connect_endpoints": [],
      "multicast_scouting": true,
      "gossip_scouting": true
    }
  }
}
```

When `zenoh.network.enabled` is `false` (the default), the Zenoh session
runs in local peer mode with no network listeners — identical behaviour
to Phase D.

> **Security warning:** The example above uses plaintext TCP with no
> authentication. This is acceptable for development and bench testing.
> For production / flight hardware, use `"protocol": "tls"` with mutual
> TLS certificates and configure Zenoh authentication. See
> [PRODUCTION_READINESS.md](../PRODUCTION_READINESS.md) items 2.1 and 2.6.

### Wire Format

Messages crossing the network boundary are prefixed with a **24-byte
packed header**:

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                     magic (0x4E4F5244)                        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   version     |    flags      |         msg_type              |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                     payload_size                              |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                                                               |
|                     timestamp_ns (64-bit)                      |
|                                                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                     sequence                                  |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

| Field | Size | Description |
|-------|------|-------------|
| `magic` | 4 B | `0x4E4F5244` — ASCII "DRON" (little-endian) |
| `version` | 1 B | Wire format version (currently `1`) |
| `flags` | 1 B | Reserved — always `0` |
| `msg_type` | 2 B | `WireMessageType` enum value |
| `payload_size` | 4 B | Bytes of payload following the header |
| `timestamp_ns` | 8 B | Sender's `steady_clock` timestamp |
| `sequence` | 4 B | Per-topic monotonic counter |

**Magic validation**: A receiver checks bytes `[0..3]` against
`0x4E4F5244` to quickly reject non-drone traffic.

### Message Type IDs

| ID | Name | SHM Type | Source |
|----|------|----------|--------|
| 1 | `VIDEO_FRAME` | `ShmVideoFrame` | P1 |
| 2 | `STEREO_FRAME` | `ShmStereoFrame` | P1 |
| 10 | `DETECTIONS` | `ShmDetectedObjectList` | P2 |
| 20 | `SLAM_POSE` | `ShmPose` | P3 |
| 30 | `MISSION_STATUS` | `ShmMissionStatus` | P4 |
| 31 | `TRAJECTORY_CMD` | `ShmTrajectoryCmd` | P4 |
| 32 | `PAYLOAD_COMMAND` | `ShmPayloadCommand` | P4 |
| 33 | `FC_COMMAND` | `ShmFCCommand` | P4 |
| 40 | `FC_STATE` | `ShmFCState` | P5 |
| 41 | `GCS_COMMAND` | `ShmGCSCommand` | P5 |
| 50 | `PAYLOAD_STATUS` | `ShmPayloadStatus` | P6 |
| 60 | `SYSTEM_HEALTH` | `ShmSystemHealth` | P7 |

IDs are **stable** — never renumber existing entries.

## Files

| File | Purpose |
|------|---------|
| `common/ipc/include/ipc/zenoh_network_config.h` | Network config struct + factories |
| `common/ipc/include/ipc/wire_format.h` | Wire header, serialisation helpers |
| `common/ipc/include/ipc/zenoh_session.h` | `configure_network()` method |
| `common/ipc/include/ipc/message_bus_factory.h` | Config-aware `create_message_bus(cfg)` |
| `config/default.json` | Default config (network disabled) |
| `config/hardware.json` | Hardware config (network enabled) |
| `tools/gcs_client/gcs_client.py` | Python GCS subscriber tool |
| `tests/test_zenoh_network.cpp` | 23 network-specific tests |

## Usage

### Drone Side

All 7 processes automatically read network config from the JSON config
file via the config-aware factory:

```cpp
drone::Config cfg;
cfg.load(args.config_path);
auto bus = drone::ipc::create_message_bus(cfg);
```

When `zenoh.network.enabled` is `true`, the session opens a TCP
listener on `0.0.0.0:7447` and remote subscribers can connect.

### GCS Side (Python)

```bash
cd tools/gcs_client
pip install -r requirements.txt
python3 gcs_client.py --drone-ip 192.168.1.10
```

The GCS client connects in Zenoh **client mode**, subscribes to
`drone/**`, and decodes wire-format headers for display.

### C++ GCS Subscriber

```cpp
auto net_cfg = ZenohNetworkConfig::make_gcs("192.168.1.10");
ZenohSession session;
session.configure_network(net_cfg);
session.open_session();
// subscribe as usual via bus_subscribe<T>(...)
```

## Testing

```bash
# Build with Zenoh enabled
cmake -B build -DENABLE_ZENOH=ON
cmake --build build -j

# Run network-specific tests
./build/bin/test_zenoh_network

# Run full suite
cd build && ctest --output-on-failure -j$(nproc)
```

## Design Decisions

- **No Protobuf**: Wire format uses packed C structs (already
  trivially copyable). Protobuf adds ~3 MB dependency + codegen.
  Upgradable later if multi-vendor interop is needed (see ADR-002).

- **Peer + Client**: Drone runs as Zenoh **peer** with a TCP listener.
  GCS connects as **client**. This avoids needing a separate Zenoh
  router process.

- **Same key expressions**: No translation layer.  `drone/slam/pose`
  on the drone is `drone/slam/pose` on the GCS.  Zenoh handles
  routing transparently.

## Related

- [ADR-002: Zenoh Migration](adr/ADR-002-zenoh-migration.md)
- [Key Expressions](ipc-key-expressions.md)
- [Issue #50](../../issues/50)
