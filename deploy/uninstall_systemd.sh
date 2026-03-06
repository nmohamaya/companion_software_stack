#!/usr/bin/env bash
# deploy/uninstall_systemd.sh — Remove drone companion stack systemd units.
#
# Stops all services, disables the target, removes unit files.
#
# Usage:
#   sudo ./deploy/uninstall_systemd.sh
#   sudo ./deploy/uninstall_systemd.sh --keep-logs    # preserve /var/log/drone
set -euo pipefail

SYSTEMD_DIR="/etc/systemd/system"
LOG_DIR="/var/log/drone"
KEEP_LOGS=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --keep-logs) KEEP_LOGS=true; shift ;;
        -h|--help)
            echo "Usage: sudo $0 [--keep-logs]"
            exit 0
            ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: This script must be run as root (or via sudo)."
    exit 1
fi

echo "╔══════════════════════════════════════════════╗"
echo "║  Drone Stack — systemd Uninstallation        ║"
echo "╚══════════════════════════════════════════════╝"

# ── Stop & disable ───────────────────────────────────────────
echo "[1/3] Stopping and disabling services..."
systemctl stop drone-stack.target 2>/dev/null || true
systemctl disable drone-stack.target 2>/dev/null || true

UNITS=(
    drone-video-capture.service
    drone-perception.service
    drone-slam-vio-nav.service
    drone-comms.service
    drone-mission-planner.service
    drone-payload-manager.service
    drone-system-monitor.service
    drone-stack.target
)

for unit in "${UNITS[@]}"; do
    systemctl stop "$unit" 2>/dev/null || true
    systemctl disable "$unit" 2>/dev/null || true
done

# ── Remove unit files ────────────────────────────────────────
echo "[2/3] Removing unit files..."
for unit in "${UNITS[@]}"; do
    if [[ -f "${SYSTEMD_DIR}/${unit}" ]]; then
        rm -f "${SYSTEMD_DIR}/${unit}"
        echo "  Removed: ${unit}"
    fi
done

systemctl daemon-reload
systemctl reset-failed 2>/dev/null || true

# ── Clean up logs ────────────────────────────────────────────
if $KEEP_LOGS; then
    echo "[3/3] Keeping log directory: ${LOG_DIR}"
else
    if [[ -d "$LOG_DIR" ]]; then
        echo "[3/3] Removing log directory: ${LOG_DIR}"
        rm -rf "$LOG_DIR"
    else
        echo "[3/3] No log directory to remove."
    fi
fi

echo ""
echo "✓ Uninstallation complete."
