# GCS Client

A Python tool for subscribing to drone telemetry over a Zenoh network link.

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Connect to drone on default port
python3 gcs_client.py --drone-ip 192.168.1.10

# Subscribe to specific topics
python3 gcs_client.py --drone-ip 192.168.1.10 --key "drone/slam/*"

# Show periodic statistics
python3 gcs_client.py --drone-ip 10.0.0.2 --stats-interval 5
```

## Options

| Flag | Default | Description |
|------|---------|-------------|
| `--drone-ip` | *(required)* | IP address of the drone |
| `--port` | 7447 | Zenoh TCP port on the drone |
| `--protocol` | tcp | Transport protocol (tcp/udp/tls) |
| `--key` | `drone/**` | Zenoh key expression filter |
| `--stats-interval` | 0 | Print stats every N seconds (0 = off) |

## Wire Format

Messages are prefixed with a 24-byte packed header:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | magic | `0x4E4F5244` ("DRON" LE) |
| 4 | 1 | version | Wire format version (1) |
| 5 | 1 | flags | Reserved |
| 6 | 2 | msg_type | `WireMessageType` enum |
| 8 | 4 | payload_size | Payload bytes after header |
| 12 | 8 | timestamp_ns | Sender's timestamp |
| 20 | 4 | sequence | Per-topic counter |

See `common/ipc/include/ipc/wire_format.h` for the C++ definition.
