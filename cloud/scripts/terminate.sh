#!/usr/bin/env bash
# ============================================================================
# cloud/scripts/terminate.sh — Graceful cloud instance shutdown
# ============================================================================
# Stops containers, optionally collects artifacts, and terminates the instance.
#
# Usage:
#   CLOUD_PROVIDER=aws CLOUD_REGION=us-east-1 ./terminate.sh <instance-ip>
#
# Set ARTIFACT_STORE_URL to collect artifacts before termination.
# ============================================================================
set -euo pipefail

INSTANCE_IP="${1:?Usage: terminate.sh <instance-ip>}"
CLOUD_PROVIDER="${CLOUD_PROVIDER:-aws}"
CLOUD_REGION="${CLOUD_REGION:-us-east-1}"
SSH_OPTS="-o StrictHostKeyChecking=no -o ConnectTimeout=10"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROVIDER_SCRIPT="${SCRIPT_DIR}/../providers/${CLOUD_PROVIDER}.sh"

if [[ ! -f "$PROVIDER_SCRIPT" ]]; then
    echo "ERROR: Unknown cloud provider '${CLOUD_PROVIDER}'"
    exit 1
fi

# shellcheck source=/dev/null
source "$PROVIDER_SCRIPT"

echo "=== Terminating deployment at ${INSTANCE_IP} ==="

# Step 1: Collect artifacts (if configured)
if [[ -n "${ARTIFACT_STORE_URL:-}" ]]; then
    echo "[1/3] Collecting artifacts..."
    "${SCRIPT_DIR}/collect_artifacts.sh" "$INSTANCE_IP" || echo "  WARNING: Artifact collection failed (continuing with shutdown)"
else
    echo "[1/3] Skipping artifact collection (ARTIFACT_STORE_URL not set)"
fi

# Step 2: Stop containers gracefully
echo "[2/3] Stopping containers..."
# shellcheck disable=SC2086
ssh ${SSH_OPTS} "ubuntu@${INSTANCE_IP}" \
    "cd /opt/companion-stack && docker compose -f docker/docker-compose.cosys.yml down --timeout 30" \
    2>/dev/null || echo "  WARNING: Container shutdown failed (instance may already be stopped)"

# Step 3: Terminate instance via provider
echo "[3/3] Terminating instance..."
INSTANCE_ID=$(get_instance_id_from_ip "$INSTANCE_IP")
if [[ -n "$INSTANCE_ID" && "$INSTANCE_ID" != "None" ]]; then
    terminate_instance "$INSTANCE_ID"
    echo "Instance ${INSTANCE_ID} terminated."
else
    echo "WARNING: Could not resolve instance ID for ${INSTANCE_IP}"
    echo "You may need to terminate the instance manually."
fi

echo "=== Shutdown complete ==="
