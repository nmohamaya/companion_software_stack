#!/usr/bin/env bash
# models/download_yolov8n.sh
# Downloads YOLOv8-nano ONNX model for OpenCV DNN.
#
# Method 1 (preferred): Direct download from Ultralytics GitHub releases.
# Method 2 (fallback):  Export via Python ultralytics package.
#
# Usage:
#   cd <project_root>
#   bash models/download_yolov8n.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODEL_PATH="${SCRIPT_DIR}/yolov8n.onnx"

# Check for existing valid model (not 0-byte)
if [[ -f "$MODEL_PATH" ]] && [[ -s "$MODEL_PATH" ]]; then
    echo "Model already exists: $MODEL_PATH ($(du -h "$MODEL_PATH" | cut -f1))"
    exit 0
fi

# Remove 0-byte files from failed downloads
if [[ -f "$MODEL_PATH" ]] && [[ ! -s "$MODEL_PATH" ]]; then
    echo "Removing empty model file from previous failed download..."
    rm -f "$MODEL_PATH"
fi

echo "Downloading YOLOv8n ONNX model..."

# Method 1: Direct download (no Python deps needed)
# Try multiple known URLs for the pre-exported ONNX model
URLS=(
    "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.onnx"
    "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.onnx"
    "https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.onnx"
)

for url in "${URLS[@]}"; do
    echo "  Trying: $url"
    if wget -q --show-progress -O "$MODEL_PATH" "$url" 2>/dev/null; then
        if [[ -s "$MODEL_PATH" ]]; then
            echo "Downloaded successfully: $MODEL_PATH ($(du -h "$MODEL_PATH" | cut -f1))"
            exit 0
        fi
    fi
    rm -f "$MODEL_PATH"
done

echo "Direct download failed. Trying Python export..."

# Method 2: Export via ultralytics Python package
if python3 -c "import ultralytics" 2>/dev/null; then
    python3 -c "
from ultralytics import YOLO
import shutil, os

model = YOLO('yolov8n.pt')
model.export(format='onnx', imgsz=640, simplify=True, opset=12)

src = 'yolov8n.onnx'
dst = '${MODEL_PATH}'
if os.path.exists(src):
    shutil.move(src, dst)
    print(f'Model exported to {dst} ({os.path.getsize(dst) / 1e6:.1f} MB)')
# Clean up .pt file
if os.path.exists('yolov8n.pt'):
    os.remove('yolov8n.pt')
"
    if [[ -s "$MODEL_PATH" ]]; then
        echo "Done."
        exit 0
    fi
fi

echo "ERROR: Could not download or export YOLOv8n model."
echo "Manual options:"
echo "  1. pip3 install ultralytics && bash models/download_yolov8n.sh"
echo "  2. Download yolov8n.onnx manually from https://github.com/ultralytics/assets/releases"
echo "     and place it at: $MODEL_PATH"
exit 1
