#!/usr/bin/env bash
# ============================================================================
# cloud/scripts/collect_artifacts.sh — Collect logs and artifacts from instance
# ============================================================================
# Downloads logs and perception artifacts from a running cloud deployment,
# then uploads them to the configured artifact store.
#
# Supports artifact store schemes: s3://, gs://, file://
#
# Usage:
#   ARTIFACT_STORE_URL=s3://bucket/runs ./collect_artifacts.sh <instance-ip>
#   ARTIFACT_STORE_URL=file:///tmp/artifacts ./collect_artifacts.sh <instance-ip>
# ============================================================================
set -euo pipefail

INSTANCE_IP="${1:?Usage: collect_artifacts.sh <instance-ip>}"
ARTIFACT_STORE_URL="${ARTIFACT_STORE_URL:?Set ARTIFACT_STORE_URL (s3://, gs://, file://)}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
SSH_OPTS="-o StrictHostKeyChecking=no -o ConnectTimeout=10"

echo "=== Collecting artifacts from ${INSTANCE_IP} ==="
echo "Run ID: ${RUN_ID}"

TMPDIR=$(mktemp -d)
# Ensure cleanup on exit
cleanup() { rm -rf "$TMPDIR"; }
trap cleanup EXIT

# Collect logs and artifacts from remote instance
echo "Downloading logs..."
# shellcheck disable=SC2086
scp -r ${SSH_OPTS} "ubuntu@${INSTANCE_IP}:/opt/companion-stack/logs/" "${TMPDIR}/" 2>/dev/null || true

echo "Downloading artifacts..."
# shellcheck disable=SC2086
scp -r ${SSH_OPTS} "ubuntu@${INSTANCE_IP}:/opt/companion-stack/artifacts/" "${TMPDIR}/" 2>/dev/null || true

# Also grab docker compose logs as a file
echo "Downloading container logs..."
# shellcheck disable=SC2086
ssh ${SSH_OPTS} "ubuntu@${INSTANCE_IP}" \
    "cd /opt/companion-stack && docker compose -f docker/docker-compose.cosys.yml logs --no-color 2>&1" \
    > "${TMPDIR}/docker-compose.log" 2>/dev/null || true

# Upload to artifact store (provider-neutral via URL scheme)
echo "Uploading to ${ARTIFACT_STORE_URL}${RUN_ID}/..."

case "$ARTIFACT_STORE_URL" in
    s3://*)
        aws s3 cp --recursive "$TMPDIR" "${ARTIFACT_STORE_URL}${RUN_ID}/"
        ;;
    gs://*)
        gsutil -m cp -r "${TMPDIR}/*" "${ARTIFACT_STORE_URL}${RUN_ID}/"
        ;;
    file://*)
        local_path="${ARTIFACT_STORE_URL#file://}/${RUN_ID}"
        mkdir -p "$local_path"
        cp -r "${TMPDIR}"/* "$local_path/"
        ;;
    *)
        echo "ERROR: Unsupported artifact store scheme: ${ARTIFACT_STORE_URL}"
        echo "Supported: s3://, gs://, file://"
        exit 1
        ;;
esac

echo "Artifacts uploaded to ${ARTIFACT_STORE_URL}${RUN_ID}/"
