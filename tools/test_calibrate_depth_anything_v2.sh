#!/usr/bin/env bash
# Smoke test for tools/calibrate_depth_anything_v2.py (Issue #616).
#
# Exercises the end-to-end parsing + fitting pipeline using a synthetic
# perception.log sample.  Not wired into ctest — this project does not use
# CMake to drive Python/shell tests — run it manually when touching the
# calibration script:
#
#     bash tools/test_calibrate_depth_anything_v2.sh
#
# Exits non-zero on the first failed assertion.  Keeps the test hermetic:
# all scratch files land in a per-run $TMPDIR that is cleaned up on exit.

set -euo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="${HERE}/calibrate_depth_anything_v2.py"

if [[ ! -x "${SCRIPT}" && ! -f "${SCRIPT}" ]]; then
    echo "ERROR: ${SCRIPT} not found" >&2
    exit 2
fi

WORK="$(mktemp -d -t calib-dav2-test-XXXXXX)"
trap 'rm -rf "${WORK}"' EXIT

fail() {
    echo "FAIL: $*" >&2
    exit 1
}

# ── 1. Happy path: multi-frame log with clear min/max ───────────────────
LOG="${WORK}/perception.log"
cat > "${LOG}" <<'EOF'
2026-04-20 10:00:00 INFO unrelated: boot
[DepthAnythingV2] 518x518 depth map in 42ms (inv_depth range: [0.100, 0.900])
[DepthAnythingV2] 518x518 depth map in 39ms (inv_depth range: [0.110, 0.880])
[DepthAnythingV2] 518x518 depth map in 41ms (inv_depth range: [0.105, 0.910])
2026-04-20 10:00:01 INFO another unrelated line
[DepthAnythingV2] 518x518 depth map in 40ms (inv_depth range: [0.120, 0.895])
EOF

OUT="${WORK}/out.json"
python3 "${SCRIPT}" "${LOG}" --emit-json "${OUT}" >"${WORK}/stdout.txt" 2>"${WORK}/stderr.txt" \
    || fail "happy path exited non-zero"

grep -q 'Parsed 4 frames' "${WORK}/stdout.txt" \
    || fail "expected 'Parsed 4 frames' in stdout"
grep -q '"raw_min_ref": 0.1' "${OUT}" \
    || fail "expected raw_min_ref=0.1 in JSON"
grep -q '"raw_max_ref": 0.91' "${OUT}" \
    || fail "expected raw_max_ref=0.91 in JSON"

# Single-frame warning MUST NOT fire when we have 4 samples.
if grep -q 'only .* sample' "${WORK}/stderr.txt"; then
    fail "single-sample warning fired with 4 samples"
fi

# ── 2. No matching lines → exit 1 + helpful error ───────────────────────
BAD="${WORK}/empty.log"
echo "nothing to parse here" > "${BAD}"

set +e
python3 "${SCRIPT}" "${BAD}" >/dev/null 2>"${WORK}/stderr_bad.txt"
rc=$?
set -e
[[ ${rc} -eq 1 ]] || fail "empty-log path expected rc=1, got ${rc}"
grep -q 'no DA V2 inv_depth range lines' "${WORK}/stderr_bad.txt" \
    || fail "empty-log error message missing"

# ── 3. Missing file → exit 2 ────────────────────────────────────────────
set +e
python3 "${SCRIPT}" "${WORK}/does_not_exist.log" >/dev/null 2>/dev/null
rc=$?
set -e
[[ ${rc} -eq 2 ]] || fail "missing-file path expected rc=2, got ${rc}"

# ── 4. Single-frame log → warning on stderr ─────────────────────────────
SINGLE="${WORK}/single.log"
echo '[DepthAnythingV2] 518x518 depth map in 42ms (inv_depth range: [0.100, 0.900])' \
    > "${SINGLE}"
python3 "${SCRIPT}" "${SINGLE}" >/dev/null 2>"${WORK}/stderr_single.txt" \
    || fail "single-frame path exited non-zero (should still succeed)"
grep -q 'only 1 DA V2 range sample' "${WORK}/stderr_single.txt" \
    || fail "single-frame warning missing on stderr"

# ── 5. Percentile flag shifts refs inward ───────────────────────────────
OUT_P="${WORK}/out_p.json"
python3 "${SCRIPT}" "${LOG}" --percentile 75 --emit-json "${OUT_P}" \
    >/dev/null 2>&1 || fail "--percentile 75 exited non-zero"
# Parse both JSON files and assert the percentile fit has a strictly
# tighter range (higher min, lower max) than the absolute fit.
python3 - <<PY
import json, sys
p100 = json.load(open("${OUT}"))
p75  = json.load(open("${OUT_P}"))
assert p75["raw_min_ref"] > p100["raw_min_ref"], \
    f"p75 min {p75['raw_min_ref']} not > p100 min {p100['raw_min_ref']}"
assert p75["raw_max_ref"] < p100["raw_max_ref"], \
    f"p75 max {p75['raw_max_ref']} not < p100 max {p100['raw_max_ref']}"
PY

echo "OK — all 5 calibration-script checks passed"
