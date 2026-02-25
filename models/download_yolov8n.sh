#!/usr/bin/env bash
# models/download_yolov8n.sh
# Downloads and exports YOLOv8-nano to ONNX format for OpenCV DNN.
# Requires: pip install ultralytics
#
# Usage:
#   cd <project_root>
#   bash models/download_yolov8n.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODEL_PATH="${SCRIPT_DIR}/yolov8n.onnx"

if [[ -f "$MODEL_PATH" ]]; then
    echo "Model already exists: $MODEL_PATH ($(du -h "$MODEL_PATH" | cut -f1))"
    exit 0
fi

echo "Downloading and exporting YOLOv8n to ONNX..."

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

echo "Done."
