#!/usr/bin/env bash
# ============================================================================
# tests/run_perception_ci.sh — Perception regression CI runner
# ============================================================================
# Provisions a cloud GPU instance (via cloud/scripts/launch.sh), deploys
# containers, runs perception scenarios, collects metrics, compares against
# a baseline, and outputs a JSON results file.
#
# Usage:
#   bash tests/run_perception_ci.sh --baseline tests/perception_baseline.json --output /tmp/results.json
#   bash tests/run_perception_ci.sh --dry-run   # Local testing without cloud
#
# The --dry-run flag generates placeholder results from the baseline (zero
# deltas, all passing) for CI testing without cloud credentials.
# ============================================================================
set -euo pipefail

BASELINE=""
OUTPUT="/tmp/perception-results.json"
DRY_RUN=0
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --baseline) BASELINE="$2"; shift 2 ;;
        --output)   OUTPUT="$2"; shift 2 ;;
        --dry-run)  DRY_RUN=1; shift ;;
        *) echo "ERROR: Unknown option: $1" >&2; exit 1 ;;
    esac
done

BASELINE="${BASELINE:-$SCRIPT_DIR/perception_baseline.json}"

if [[ ! -f "$BASELINE" ]]; then
    echo "ERROR: Baseline file not found: $BASELINE" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Dry-run mode: generate placeholder results without cloud provisioning
# ---------------------------------------------------------------------------
if [[ "$DRY_RUN" == "1" ]]; then
    echo "DRY RUN — generating placeholder results from baseline"
    python3 -c "
import json, sys

baseline_path = '$BASELINE'
output_path = '$OUTPUT'

with open(baseline_path) as f:
    bl = json.load(f)

results = {'metrics': {}, 'cost': 0, 'duration_min': 0, 'dry_run': True}
for key, val in bl['metrics'].items():
    results['metrics'][key] = {
        'baseline': val['value'],
        'current': val['value'],
        'threshold': val['threshold'],
        'passed': True
    }

with open(output_path, 'w') as f:
    json.dump(results, f, indent=2)

passed = sum(1 for m in results['metrics'].values() if m['passed'])
total = len(results['metrics'])
print(f'Dry run: {passed}/{total} metrics passing')
print(f'Results written to {output_path}')
"
    exit 0
fi

# ---------------------------------------------------------------------------
# Live mode: provision cloud instance and run perception scenarios
# ---------------------------------------------------------------------------
echo "=== Perception Regression CI ==="
echo "Baseline: $BASELINE"
echo "Output:   $OUTPUT"
echo ""

# Step 1: Provision cloud GPU instance via provider-neutral launcher
echo "--- Step 1: Provisioning cloud instance ---"
export RUN_DURATION_HOURS=1
bash "$PROJECT_DIR/cloud/scripts/launch.sh" 2>&1 | tee /tmp/launch.log

# Extract instance IP from launch output (launch.sh prints "Public IP: <ip>")
INSTANCE_IP=$(grep "Public IP:" /tmp/launch.log | awk '{print $NF}')
if [[ -z "$INSTANCE_IP" ]]; then
    echo "ERROR: Could not extract instance IP from launch output" >&2
    exit 1
fi
echo "$INSTANCE_IP" > /tmp/instance-ip.txt
echo "Instance IP: $INSTANCE_IP"

# Step 2: Wait for containers to become healthy
echo "--- Step 2: Waiting for containers to be healthy ---"
SSH_OPTS="-o StrictHostKeyChecking=no -o ConnectTimeout=10"
for i in $(seq 1 60); do
    # shellcheck disable=SC2086
    if ssh ${SSH_OPTS} "ubuntu@$INSTANCE_IP" \
        "docker compose -f /opt/companion-stack/docker/docker-compose.cosys.yml ps --format json" 2>/dev/null | \
        python3 -c "import sys,json; data=json.loads(sys.stdin.read()); sys.exit(0 if all(s.get('Health')=='healthy' for s in (data if isinstance(data,list) else [data])) else 1)" 2>/dev/null; then
        echo "  All containers healthy after $((i * 10)) seconds"
        break
    fi
    if [[ "$i" -eq 60 ]]; then
        echo "WARNING: Containers did not become healthy within 600s — continuing anyway"
    fi
    sleep 10
done

# Step 3: Run perception scenarios and collect raw metrics
echo "--- Step 3: Running perception scenarios ---"
# shellcheck disable=SC2086,SC2087
ssh ${SSH_OPTS} "ubuntu@$INSTANCE_IP" << 'SCENARIOS'
set -euo pipefail
cd /opt/companion-stack

# Run perception-focused scenario with 30-min timeout
timeout 1800 bash tests/run_scenario.sh \
    --config config/scenarios/perception_avoidance.json \
    --output /tmp/metrics.json 2>&1 || true

# Also run depth accuracy scenario if available
if [[ -f config/scenarios/perception_depth_accuracy.json ]]; then
    timeout 1800 bash tests/run_scenario.sh \
        --config config/scenarios/perception_depth_accuracy.json \
        --output /tmp/depth_metrics.json 2>&1 || true
fi
SCENARIOS

# Step 4: Download raw metrics from instance
echo "--- Step 4: Collecting metrics ---"
# shellcheck disable=SC2086
scp ${SSH_OPTS} "ubuntu@$INSTANCE_IP:/tmp/metrics.json" /tmp/raw_metrics.json 2>/dev/null || echo '{}' > /tmp/raw_metrics.json
# shellcheck disable=SC2086
scp ${SSH_OPTS} "ubuntu@$INSTANCE_IP:/tmp/depth_metrics.json" /tmp/raw_depth_metrics.json 2>/dev/null || true

# Step 5: Compare against baseline and generate results JSON
echo "--- Step 5: Comparing against baseline ---"
python3 << COMPARE_EOF
import json

with open('$BASELINE') as f:
    bl = json.load(f)

# Load raw metrics (may be empty if scenarios failed)
try:
    with open('/tmp/raw_metrics.json') as f:
        raw = json.load(f)
except (FileNotFoundError, json.JSONDecodeError):
    raw = {}

# Merge depth metrics if available
try:
    with open('/tmp/raw_depth_metrics.json') as f:
        depth_raw = json.load(f)
    raw.update(depth_raw)
except (FileNotFoundError, json.JSONDecodeError):
    pass

results = {'metrics': {}, 'cost': 0.25, 'duration_min': 30}
for key, val in bl['metrics'].items():
    baseline_val = val['value']
    threshold = val['threshold']

    if key not in raw:
        # Missing metric = fail (don't silently fall back to baseline)
        results['metrics'][key] = {
            'baseline': baseline_val,
            'current': None,
            'threshold': threshold,
            'passed': False,
            'error': 'metric not found in scenario output'
        }
        continue

    current = raw[key]

    if val.get('direction') == 'lower_is_better':
        # Current should not exceed baseline + threshold
        passed = current <= baseline_val + threshold
    else:
        # Current should not fall below baseline - threshold
        passed = current >= baseline_val - threshold

    results['metrics'][key] = {
        'baseline': baseline_val,
        'current': current,
        'threshold': threshold,
        'passed': passed
    }

passed_count = sum(1 for m in results['metrics'].values() if m['passed'])
total_count = len(results['metrics'])

with open('$OUTPUT', 'w') as f:
    json.dump(results, f, indent=2)

print(f'Results: {passed_count}/{total_count} metrics within threshold')
COMPARE_EOF

echo "Results written to $OUTPUT"

# Step 6: Collect artifacts (if artifact store is configured)
if [[ -n "${ARTIFACT_STORE_URL:-}" ]]; then
    echo "--- Step 6: Collecting artifacts ---"
    bash "$PROJECT_DIR/cloud/scripts/collect_artifacts.sh" "$INSTANCE_IP" || true
else
    echo "--- Step 6: Skipping artifact collection (ARTIFACT_STORE_URL not set) ---"
fi

echo ""
echo "=== Perception Regression CI complete ==="
