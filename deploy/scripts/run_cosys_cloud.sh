#!/usr/bin/env bash
# deploy/scripts/run_cosys_cloud.sh — Launch Cosys-AirSim + companion stack on
# AWS (or any cloud with NVIDIA GPU).
#
# Usage:
#   bash deploy/scripts/run_cosys_cloud.sh                    # Launch with docker compose
#   bash deploy/scripts/run_cosys_cloud.sh --provision        # Request AWS spot instance first
#   bash deploy/scripts/run_cosys_cloud.sh --down             # Tear down
#   bash deploy/scripts/run_cosys_cloud.sh --logs             # Stream logs
#   bash deploy/scripts/run_cosys_cloud.sh --status           # Check running status
#
# Environment variables:
#   AWS_INSTANCE_TYPE    EC2 instance type    (default: g5.xlarge)
#   AWS_REGION           AWS region           (default: us-east-1)
#   AWS_AMI_ID           AMI with NVIDIA drivers (default: auto-detect)
#   AWS_KEY_NAME         SSH key pair name    (required for --provision)
#   AWS_SECURITY_GROUP   Security group ID    (required for --provision)
#   AWS_SPOT_PRICE       Max spot price USD   (default: 1.50)
#
# Prerequisites:
#   - AWS CLI configured (aws configure)
#   - Docker + Docker Compose installed
#   - NVIDIA Container Toolkit (for GPU passthrough)
#
# Issue: #463 (part of Epic #459)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_DIR="$(dirname "$DEPLOY_DIR")"

# ── Defaults ──────────────────────────────────────────────────
AWS_INSTANCE_TYPE="${AWS_INSTANCE_TYPE:-g5.xlarge}"
AWS_REGION="${AWS_REGION:-us-east-1}"
AWS_SPOT_PRICE="${AWS_SPOT_PRICE:-1.50}"
COMPOSE_FILE="${DEPLOY_DIR}/docker-compose.cosys.yml"

ACTION="up"

# ── Parse arguments ──────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        --provision)  ACTION="provision" ;;
        --down)       ACTION="down" ;;
        --logs)       ACTION="logs" ;;
        --status)     ACTION="status" ;;
        --help|-h)
            echo "Usage: $0 [--provision|--down|--logs|--status]"
            echo ""
            echo "  (default)     Launch docker compose stack"
            echo "  --provision   Request AWS spot instance + launch"
            echo "  --down        Tear down docker compose"
            echo "  --logs        Stream container logs"
            echo "  --status      Show running container status"
            exit 0
            ;;
    esac
done

# ── Helper functions ─────────────────────────────────────────
check_docker() {
    if ! command -v docker &>/dev/null; then
        echo "ERROR: docker not found. Install Docker Engine."
        exit 1
    fi
    if ! docker compose version &>/dev/null; then
        echo "ERROR: docker compose not found. Install Docker Compose V2."
        exit 1
    fi
}

check_nvidia_docker() {
    if ! docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi &>/dev/null; then
        echo "ERROR: NVIDIA Container Toolkit not working."
        echo "       Install: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/"
        exit 1
    fi
    echo "[OK] NVIDIA Container Toolkit verified"
}

check_aws_cli() {
    if ! command -v aws &>/dev/null; then
        echo "ERROR: AWS CLI not found. Install and configure: aws configure"
        exit 1
    fi
    if ! aws sts get-caller-identity &>/dev/null; then
        echo "ERROR: AWS CLI not configured. Run: aws configure"
        exit 1
    fi
    echo "[OK] AWS CLI configured"
}

# ── Provision AWS spot instance ──────────────────────────────
provision_spot_instance() {
    check_aws_cli

    if [[ -z "${AWS_KEY_NAME:-}" ]]; then
        echo "ERROR: AWS_KEY_NAME must be set for provisioning"
        exit 1
    fi
    if [[ -z "${AWS_SECURITY_GROUP:-}" ]]; then
        echo "ERROR: AWS_SECURITY_GROUP must be set for provisioning"
        exit 1
    fi

    echo "══════════════════════════════════════════════════════════"
    echo "  AWS Spot Instance Provisioning"
    echo "  Instance Type : ${AWS_INSTANCE_TYPE}"
    echo "  Region        : ${AWS_REGION}"
    echo "  Max Price     : \$${AWS_SPOT_PRICE}/hr"
    echo "══════════════════════════════════════════════════════════"

    # Find latest Deep Learning AMI (Ubuntu 22.04) if not specified
    if [[ -z "${AWS_AMI_ID:-}" ]]; then
        echo "Looking up latest NVIDIA Deep Learning AMI..."
        AWS_AMI_ID=$(aws ec2 describe-images \
            --region "$AWS_REGION" \
            --owners amazon \
            --filters \
                "Name=name,Values=Deep Learning Base OSS Nvidia Driver GPU AMI (Ubuntu 22.04)*" \
                "Name=state,Values=available" \
            --query 'sort_by(Images, &CreationDate)[-1].ImageId' \
            --output text 2>/dev/null || echo "")

        if [[ -z "$AWS_AMI_ID" || "$AWS_AMI_ID" == "None" ]]; then
            echo "ERROR: Could not find suitable AMI. Set AWS_AMI_ID manually."
            exit 1
        fi
        echo "Using AMI: ${AWS_AMI_ID}"
    fi

    # Request spot instance
    echo "Requesting spot instance..."
    SPOT_REQUEST=$(aws ec2 request-spot-instances \
        --region "$AWS_REGION" \
        --spot-price "$AWS_SPOT_PRICE" \
        --instance-count 1 \
        --type "one-time" \
        --launch-specification "{
            \"ImageId\": \"${AWS_AMI_ID}\",
            \"InstanceType\": \"${AWS_INSTANCE_TYPE}\",
            \"KeyName\": \"${AWS_KEY_NAME}\",
            \"SecurityGroupIds\": [\"${AWS_SECURITY_GROUP}\"]
        }" \
        --query 'SpotInstanceRequests[0].SpotInstanceRequestId' \
        --output text 2>/dev/null)

    if [[ -z "$SPOT_REQUEST" || "$SPOT_REQUEST" == "None" ]]; then
        echo "ERROR: Spot instance request failed"
        exit 1
    fi
    echo "Spot request: ${SPOT_REQUEST}"

    # Wait for fulfillment
    echo "Waiting for spot instance fulfillment (up to 5 minutes)..."
    INSTANCE_ID=""
    for _ in $(seq 1 60); do
        INSTANCE_ID=$(aws ec2 describe-spot-instance-requests \
            --region "$AWS_REGION" \
            --spot-instance-request-ids "$SPOT_REQUEST" \
            --query 'SpotInstanceRequests[0].InstanceId' \
            --output text 2>/dev/null || echo "None")

        if [[ -n "$INSTANCE_ID" && "$INSTANCE_ID" != "None" ]]; then
            break
        fi
        sleep 5
    done

    if [[ -z "$INSTANCE_ID" || "$INSTANCE_ID" == "None" ]]; then
        echo "ERROR: Spot instance not fulfilled within timeout"
        echo "       Cancel with: aws ec2 cancel-spot-instance-requests --spot-instance-request-ids ${SPOT_REQUEST}"
        exit 1
    fi

    # Get public IP
    PUBLIC_IP=$(aws ec2 describe-instances \
        --region "$AWS_REGION" \
        --instance-ids "$INSTANCE_ID" \
        --query 'Reservations[0].Instances[0].PublicIpAddress' \
        --output text 2>/dev/null)

    echo ""
    echo "════════════════════════════════════════════════════════"
    echo "  Spot Instance Ready"
    echo "  Instance ID : ${INSTANCE_ID}"
    echo "  Public IP   : ${PUBLIC_IP}"
    echo "  SSH         : ssh -i ~/.ssh/${AWS_KEY_NAME}.pem ubuntu@${PUBLIC_IP}"
    echo "════════════════════════════════════════════════════════"
    echo ""
    echo "Next steps on the instance:"
    echo "  1. Clone the repo"
    echo "  2. Run: bash deploy/scripts/run_cosys_cloud.sh"
}

# ── Main actions ─────────────────────────────────────────────
case "$ACTION" in
    provision)
        provision_spot_instance
        ;;
    up)
        check_docker
        echo "Verifying NVIDIA Container Toolkit..."
        check_nvidia_docker
        echo ""
        echo "════════════════════════════════════════════════════════"
        echo "  Launching Cosys-AirSim + Companion Stack (Docker)"
        echo "  Compose file: ${COMPOSE_FILE}"
        echo "════════════════════════════════════════════════════════"
        echo ""
        # Build and launch
        docker compose -f "$COMPOSE_FILE" --project-directory "$PROJECT_DIR" up --build -d
        echo ""
        echo "Stack launched. Streaming logs (Ctrl+C to detach)..."
        echo ""
        docker compose -f "$COMPOSE_FILE" --project-directory "$PROJECT_DIR" logs -f
        ;;
    down)
        check_docker
        echo "Tearing down Cosys-AirSim + Companion Stack..."
        docker compose -f "$COMPOSE_FILE" --project-directory "$PROJECT_DIR" down --remove-orphans
        echo "Stack stopped."
        ;;
    logs)
        check_docker
        docker compose -f "$COMPOSE_FILE" --project-directory "$PROJECT_DIR" logs -f
        ;;
    status)
        check_docker
        docker compose -f "$COMPOSE_FILE" --project-directory "$PROJECT_DIR" ps
        ;;
esac
