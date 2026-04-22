#!/usr/bin/env bash
# models/download_fastsam.sh
# Fetches FastSAM-s (small variant) and exports to ONNX for OpenCV DNN.
#
# FastSAM is YOLOv8-seg architecture trained on SA-1B — Meta's Segment
# Anything dataset — giving class-agnostic segmentation of every distinct
# object/surface in a single forward pass.  This is what Epic #520 /
# Issue #555 scoped and never actually shipped.  Consumed by
# `hal::FastSamInferenceBackend` (Issue #608 E5.INT) via
# `perception.path_a.sam.backend = "fastsam"`.
#
# Model footprint:
#   FastSAM-s.pt  → ~23 MB (download from Ultralytics)
#   fastsam_s.onnx → ~50 MB (after export, opset=12, imgsz=1024)
#
# Inference cost (~1280×720 frames → downsampled to 1024×1024):
#   ~80 ms CPU (OpenCV DNN, single thread)
#   ~20 ms CUDA (DNN_TARGET_CUDA, if built with it)
#
# Dependencies (installed automatically via pip if missing):
#   ultralytics>=8.0  — provides FastSAM + export
#   onnx, onnxsim   — graph cleanup
#
# Usage:
#   cd <project_root>
#   bash models/download_fastsam.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODEL_PATH="${SCRIPT_DIR}/fastsam_s.onnx"

if [[ -f "$MODEL_PATH" ]]; then
    echo "Model already exists: $MODEL_PATH ($(du -h "$MODEL_PATH" | cut -f1))"
    exit 0
fi

echo "=== FastSAM-s download + ONNX export ==="
echo "Target: $MODEL_PATH"

# ── Ensure Python + ultralytics are available ──
if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 is required but not found." >&2
    exit 1
fi

echo "Installing Python deps (ultralytics, onnx, onnxsim)..."
python3 -m pip install --quiet --user ultralytics onnx onnxsim onnxruntime 1>&2 || {
    echo "WARN: --user install failed, retrying without --user (may need sudo)..."
    python3 -m pip install --quiet ultralytics onnx onnxsim onnxruntime
}

# ── Download + export via ultralytics in a temp dir ──
TMPDIR="$(mktemp -d -t fastsam.XXXXXX)"
trap 'rm -rf "$TMPDIR"' EXIT
cd "$TMPDIR"

# Ultralytics will download FastSAM-s.pt on first instantiation.  Export to
# ONNX with opset=12 (OpenCV DNN 4.5+ compatibility) and imgsz=1024 (FastSAM
# default — the mask prototypes are at 256×256 at this input size, which
# matches the dimensions `FastSamInferenceBackend` expects).
python3 - <<'PY'
from ultralytics import FastSAM
import pathlib
model = FastSAM('FastSAM-s.pt')
exported = model.export(format='onnx', imgsz=1024, opset=12, simplify=True)
src = pathlib.Path(exported)
print(f"Exported: {src} ({src.stat().st_size / 1e6:.1f} MB)")
PY

# Move the exported file into place.  Ultralytics names it "FastSAM-s.onnx".
SRC_ONNX="$(find "$TMPDIR" -maxdepth 2 -name 'FastSAM-s.onnx' -o -name 'fastsam-s.onnx' | head -1)"
if [[ -z "$SRC_ONNX" ]]; then
    echo "ERROR: FastSAM ONNX export did not produce the expected file." >&2
    exit 1
fi

# Ultralytics' built-in simplify leaves shape-inference Floor nodes that
# OpenCV DNN 4.10 chokes on (parse error: Node [Floor@ai.onnx]).
# Run onnxsim as a second pass to constant-fold them away.  This is what
# actually makes the model load into our perception runtime.
echo "Running onnxsim post-pass (folds shape-computation Floor nodes)..."
python3 -m onnxsim "$SRC_ONNX" "$MODEL_PATH"

echo ""
echo "=== done ==="
echo "Model: $MODEL_PATH ($(du -h "$MODEL_PATH" | cut -f1))"
echo ""
echo "Use via:"
echo '  "perception": { "path_a": { "sam": { "backend": "fastsam",'
echo "                                        \"model_path\": \"$MODEL_PATH\","
echo '                                        "input_size": 1024,'
echo '                                        "confidence_threshold": 0.4,'
echo '                                        "mask_channels": 32 } } }'
