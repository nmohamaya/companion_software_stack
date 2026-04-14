#!/usr/bin/env bash
# ============================================================================
# cloud/scripts/launch.sh — Provider-neutral cloud deployment launcher
# ============================================================================
# Dispatches to cloud/providers/$CLOUD_PROVIDER.sh for provisioning.
# Adding a new cloud provider means adding one file under cloud/providers/.
#
# Required provider functions:
#   provision_instance      — create GPU instance, print instance ID
#   get_instance_ip         — given instance ID, print public IP
#   get_instance_id_from_ip — reverse lookup instance ID from public IP
#   setup_instance          — SSH into instance, install prerequisites
#   terminate_instance      — terminate instance by ID
#
# Environment variables:
#   CLOUD_PROVIDER      — provider name (default: aws)
#   CLOUD_REGION        — deployment region (default: us-east-1)
#   GPU_INSTANCE_TYPE   — instance type override (default: provider-specific)
#   SPOT_MAX_PRICE      — max spot price in USD (default: 0.50)
#   ARTIFACT_STORE_URL  — where to upload artifacts (s3://, gs://, file://)
#   RUN_DURATION_HOURS  — auto-terminate after N hours (default: 2)
#   REPO_URL            — git repo URL to clone on instance
#   REPO_BRANCH         — git branch to check out (default: main)
# ============================================================================
set -euo pipefail

CLOUD_PROVIDER="${CLOUD_PROVIDER:-aws}"
CLOUD_REGION="${CLOUD_REGION:-us-east-1}"
GPU_INSTANCE_TYPE="${GPU_INSTANCE_TYPE:-}"
SPOT_MAX_PRICE="${SPOT_MAX_PRICE:-0.50}"
ARTIFACT_STORE_URL="${ARTIFACT_STORE_URL:-}"
RUN_DURATION_HOURS="${RUN_DURATION_HOURS:-2}"
REPO_URL="${REPO_URL:-https://github.com/nmohamaya/companion_software_stack.git}"
REPO_BRANCH="${REPO_BRANCH:-main}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROVIDER_SCRIPT="${SCRIPT_DIR}/../providers/${CLOUD_PROVIDER}.sh"

if [[ ! -f "$PROVIDER_SCRIPT" ]]; then
    echo "ERROR: Unknown cloud provider '${CLOUD_PROVIDER}'"
    echo "Available providers:"
    for f in "${SCRIPT_DIR}"/../providers/*.sh; do
        [[ -f "$f" ]] && basename "$f" .sh
    done
    exit 1
fi

# Source provider-specific functions (must define the 4 required functions)
# shellcheck source=/dev/null
source "$PROVIDER_SCRIPT"

# Verify provider implements required interface
for fn in provision_instance get_instance_ip get_instance_id_from_ip setup_instance terminate_instance; do
    if ! declare -f "$fn" > /dev/null 2>&1; then
        echo "ERROR: Provider '${CLOUD_PROVIDER}' does not implement ${fn}()"
        exit 1
    fi
done

echo "=== Cloud Deployment: ${CLOUD_PROVIDER} ==="
echo "Region:        ${CLOUD_REGION}"
echo "Instance type: ${GPU_INSTANCE_TYPE:-<provider default>}"
echo "Spot max price: \$${SPOT_MAX_PRICE}"
echo "Run duration:  ${RUN_DURATION_HOURS}h"
echo "Repo:          ${REPO_URL} (${REPO_BRANCH})"
echo ""

# Step 1: Provision GPU instance
echo "[1/3] Provisioning instance..."
INSTANCE_ID=$(provision_instance)
echo "  Instance ID: ${INSTANCE_ID}"

# Step 2: Wait for instance and configure
echo "[2/3] Waiting for instance and configuring..."
INSTANCE_IP=$(get_instance_ip "$INSTANCE_ID")
echo "  Public IP: ${INSTANCE_IP}"
setup_instance "$INSTANCE_IP"

# Step 3: Deploy containers via docker compose
echo "[3/3] Deploying containers..."
# shellcheck disable=SC2087
ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "ubuntu@${INSTANCE_IP}" << REMOTE
set -euo pipefail
cd /opt/companion-stack

# Build and launch both containers
docker compose -f docker/docker-compose.cosys.yml up --build -d

echo "Waiting for services to become healthy..."
for i in \$(seq 1 60); do
    # Check that ALL services report healthy (not just any one)
    if docker compose -f docker/docker-compose.cosys.yml ps --format json 2>/dev/null | \
        python3 -c "import sys,json; data=json.loads(sys.stdin.read()); svcs=data if isinstance(data,list) else [data]; sys.exit(0 if svcs and all(s.get('Health')=='healthy' for s in svcs) else 1)" 2>/dev/null; then
        echo "All services healthy after \${i}0 seconds."
        break
    fi
    if [[ "\$i" -eq 60 ]]; then
        echo "WARNING: Services did not become healthy within 600s"
        docker compose -f docker/docker-compose.cosys.yml ps
    fi
    sleep 10
done

docker compose -f docker/docker-compose.cosys.yml ps
REMOTE

echo ""
echo "=== Deployment complete ==="
echo "Instance:  ${INSTANCE_ID}"
echo "Public IP: ${INSTANCE_IP}"
echo ""
echo "Commands:"
echo "  Logs:      ssh ubuntu@${INSTANCE_IP} 'cd /opt/companion-stack && docker compose -f docker/docker-compose.cosys.yml logs -f'"
echo "  Status:    ssh ubuntu@${INSTANCE_IP} 'cd /opt/companion-stack && docker compose -f docker/docker-compose.cosys.yml ps'"
echo "  Terminate: CLOUD_PROVIDER=${CLOUD_PROVIDER} CLOUD_REGION=${CLOUD_REGION} ${SCRIPT_DIR}/terminate.sh ${INSTANCE_IP}"
