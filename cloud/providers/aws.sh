#!/usr/bin/env bash
# ============================================================================
# cloud/providers/aws.sh — AWS EC2 provider for cloud deployment
# ============================================================================
# Implements the provider interface for AWS EC2 GPU spot instances.
# Uses the Deep Learning Base AMI (Ubuntu 22.04) which comes with
# NVIDIA drivers and Docker pre-installed.
#
# Prerequisites: aws CLI configured with appropriate IAM permissions.
#
# Implements:
#   provision_instance      — launch EC2 spot GPU instance
#   get_instance_ip         — get public IP from instance ID
#   get_instance_id_from_ip — reverse lookup instance ID from IP
#   setup_instance          — SSH in, install docker compose, clone repo
#   terminate_instance      — terminate EC2 instance
# ============================================================================

# Default to g5.xlarge (NVIDIA A10G 24GB) — sufficient for UE5 + inference
GPU_INSTANCE_TYPE="${GPU_INSTANCE_TYPE:-g5.xlarge}"

provision_instance() {
    # Find the latest Deep Learning Base AMI with NVIDIA drivers
    local ami_id
    ami_id=$(aws ec2 describe-images \
        --owners amazon \
        --filters \
            "Name=name,Values=Deep Learning Base OSS Nvidia Driver GPU AMI (Ubuntu 22.04)*" \
            "Name=state,Values=available" \
        --query 'Images | sort_by(@, &CreationDate) | [-1].ImageId' \
        --output text \
        --region "$CLOUD_REGION")

    if [[ -z "$ami_id" || "$ami_id" == "None" ]]; then
        echo "ERROR: Could not find Deep Learning Base AMI in ${CLOUD_REGION}" >&2
        return 1
    fi

    local instance_id
    instance_id=$(aws ec2 run-instances \
        --image-id "$ami_id" \
        --instance-type "$GPU_INSTANCE_TYPE" \
        --instance-market-options "MarketType=spot,SpotOptions={MaxPrice=${SPOT_MAX_PRICE},SpotInstanceType=one-time}" \
        --tag-specifications "ResourceType=instance,Tags=[{Key=Name,Value=companion-stack-cosys},{Key=Project,Value=companion-stack},{Key=AutoTerminate,Value=${RUN_DURATION_HOURS}h}]" \
        --block-device-mappings '[{"DeviceName":"/dev/sda1","Ebs":{"VolumeSize":100,"VolumeType":"gp3"}}]' \
        --query 'Instances[0].InstanceId' \
        --output text \
        --region "$CLOUD_REGION")

    if [[ -z "$instance_id" || "$instance_id" == "None" ]]; then
        echo "ERROR: Failed to launch EC2 instance" >&2
        return 1
    fi

    # Wait for instance to be running
    aws ec2 wait instance-running \
        --instance-ids "$instance_id" \
        --region "$CLOUD_REGION"

    echo "$instance_id"
}

get_instance_ip() {
    local instance_id="$1"

    # Wait for public IP to be assigned (can take a few seconds after running)
    local ip=""
    for _ in $(seq 1 12); do
        ip=$(aws ec2 describe-instances \
            --instance-ids "$instance_id" \
            --query 'Reservations[0].Instances[0].PublicIpAddress' \
            --output text \
            --region "$CLOUD_REGION")
        if [[ -n "$ip" && "$ip" != "None" ]]; then
            echo "$ip"
            return 0
        fi
        sleep 5
    done

    echo "ERROR: Timed out waiting for public IP on ${instance_id}" >&2
    return 1
}

get_instance_id_from_ip() {
    local ip="$1"
    aws ec2 describe-instances \
        --filters "Name=ip-address,Values=${ip}" \
        --query 'Reservations[0].Instances[0].InstanceId' \
        --output text \
        --region "$CLOUD_REGION"
}

setup_instance() {
    local ip="$1"

    echo "  Waiting for SSH on ${ip}..."
    for i in $(seq 1 30); do
        if ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "ubuntu@${ip}" true 2>/dev/null; then
            echo "  SSH available after $((i * 10)) seconds."
            break
        fi
        if [[ "$i" -eq 30 ]]; then
            echo "ERROR: SSH timeout after 300 seconds on ${ip}" >&2
            return 1
        fi
        sleep 10
    done

    echo "  Installing prerequisites..."
    # shellcheck disable=SC2087
    ssh -o StrictHostKeyChecking=no "ubuntu@${ip}" << SETUP
set -euo pipefail

# Install Docker Compose plugin (Docker itself is pre-installed on DL AMI)
sudo apt-get update -qq
sudo apt-get install -y -qq docker-compose-plugin

# Verify GPU access
echo "GPU check:"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader

# Verify NVIDIA Container Toolkit (pre-installed on DL AMI)
docker run --rm --gpus all nvidia/cuda:12.6.3-base-ubuntu24.04 nvidia-smi > /dev/null 2>&1 \
    && echo "NVIDIA Container Toolkit: OK" \
    || echo "WARNING: NVIDIA Container Toolkit check failed"

# Clone repository
sudo mkdir -p /opt/companion-stack
sudo chown ubuntu:ubuntu /opt/companion-stack
git clone --branch "${REPO_BRANCH}" --depth 1 "${REPO_URL}" /opt/companion-stack

# Schedule auto-termination for cost safety
nohup bash -c "sleep \$((${RUN_DURATION_HOURS} * 3600)) && sudo shutdown -h now" \
    > /dev/null 2>&1 &

echo "Setup complete."
SETUP
}

terminate_instance() {
    local instance_id="$1"
    aws ec2 terminate-instances \
        --instance-ids "$instance_id" \
        --region "$CLOUD_REGION" \
        > /dev/null
}
