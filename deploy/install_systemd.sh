#!/usr/bin/env bash
# deploy/install_systemd.sh — Install drone companion stack systemd units.
#
# Copies service files and target to /etc/systemd/system/, creates
# required directories, and enables the target for auto-start on boot.
#
# Usage:
#   sudo ./deploy/install_systemd.sh                     # defaults
#   sudo ./deploy/install_systemd.sh --bin-dir /usr/local/bin
#   sudo ./deploy/install_systemd.sh --config /etc/drone/my_config.json
#   sudo ./deploy/install_systemd.sh --no-enable          # install without auto-start
#
# Prerequisites:
#   - Binaries built (deploy/build.sh) and available in --bin-dir
#   - Config file at --config path
#   - Run as root (or with sudo)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_DIR="/etc/systemd/system"
UNIT_SRC="${SCRIPT_DIR}/systemd"

# Defaults
BIN_DIR="/opt/drone/bin"
CONFIG_PATH="/etc/drone/config.json"
LOG_DIR="/var/log/drone"
ENABLE_UNITS=true

# ── Parse arguments ──────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --bin-dir)
            BIN_DIR="$2"
            shift 2
            ;;
        --config)
            CONFIG_PATH="$2"
            shift 2
            ;;
        --log-dir)
            LOG_DIR="$2"
            shift 2
            ;;
        --no-enable)
            ENABLE_UNITS=false
            shift
            ;;
        -h|--help)
            echo "Usage: sudo $0 [--bin-dir DIR] [--config FILE] [--log-dir DIR] [--no-enable]"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# ── Preflight checks ────────────────────────────────────────
if [[ $EUID -ne 0 ]]; then
    echo "ERROR: This script must be run as root (or via sudo)."
    exit 1
fi

if [[ ! -d "$UNIT_SRC" ]]; then
    echo "ERROR: systemd unit directory not found: $UNIT_SRC"
    exit 1
fi

if ! command -v systemctl &>/dev/null; then
    echo "ERROR: systemctl not found — is systemd available?"
    exit 1
fi

echo "╔══════════════════════════════════════════════╗"
echo "║  Drone Stack — systemd Installation          ║"
echo "╚══════════════════════════════════════════════╝"
echo "  Binary dir : $BIN_DIR"
echo "  Config     : $CONFIG_PATH"
echo "  Log dir    : $LOG_DIR"
echo "  Auto-start : $ENABLE_UNITS"
echo ""

# ── Create directories ───────────────────────────────────────
echo "[1/5] Creating directories..."
mkdir -p "$BIN_DIR"
mkdir -p "$(dirname "$CONFIG_PATH")"
mkdir -p "$LOG_DIR"

# ── Verify binaries exist ────────────────────────────────────
echo "[2/5] Checking binaries..."
BINARIES=(video_capture perception slam_vio_nav comms mission_planner payload_manager system_monitor)
MISSING=()
for bin in "${BINARIES[@]}"; do
    if [[ ! -x "${BIN_DIR}/${bin}" ]]; then
        MISSING+=("$bin")
    fi
done

if [[ ${#MISSING[@]} -gt 0 ]]; then
    echo "WARNING: Missing binaries in ${BIN_DIR}: ${MISSING[*]}"
    echo "  Run 'deploy/build.sh' first, then copy binaries to ${BIN_DIR}:"
    echo "    sudo cp build/bin/* ${BIN_DIR}/"
    echo ""
fi

# ── Patch and install unit files ─────────────────────────────
echo "[3/5] Installing systemd units..."
for unit_file in "${UNIT_SRC}"/*.service "${UNIT_SRC}"/*.target; do
    if [[ ! -f "$unit_file" ]]; then
        continue
    fi
    base="$(basename "$unit_file")"

    # Substitute paths in the unit file
    sed \
        -e "s|/opt/drone/bin|${BIN_DIR}|g" \
        -e "s|/etc/drone/config.json|${CONFIG_PATH}|g" \
        -e "s|/var/log/drone|${LOG_DIR}|g" \
        "$unit_file" > "${SYSTEMD_DIR}/${base}"

    echo "  Installed: ${base}"
done

# ── Reload systemd ───────────────────────────────────────────
echo "[4/5] Reloading systemd daemon..."
systemctl daemon-reload

# ── Enable units ─────────────────────────────────────────────
if $ENABLE_UNITS; then
    echo "[5/5] Enabling drone-stack.target for auto-start..."
    systemctl enable drone-stack.target
else
    echo "[5/5] Skipping enable (--no-enable)."
fi

echo ""
echo "✓ Installation complete!"
echo ""
echo "Commands:"
echo "  sudo systemctl start  drone-stack.target   # Launch all"
echo "  sudo systemctl stop   drone-stack.target   # Stop all"
echo "  sudo systemctl status drone-stack.target   # Health check"
echo "  journalctl -u drone-perception -f          # Follow logs"
echo ""
echo "Per-process control:"
echo "  sudo systemctl restart drone-comms"
echo "  sudo systemctl status  drone-system-monitor"
