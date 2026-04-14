#!/usr/bin/env bash
# models/download_yolov8n_visdrone.sh
# Exports YOLOv8-nano to ONNX format for the VisDrone aerial detection pipeline.
#
# ⚠️  WARNING: This currently exports the BASE COCO-pretrained YOLOv8n weights
# (80 classes), NOT a VisDrone-trained model (10 classes). The output filename
# is yolov8n_visdrone.onnx for pipeline compatibility, but the model output
# shape will be [1, 84, 8400] (4 bbox + 80 COCO classes), NOT [1, 14, 8400].
#
# When using this model with dataset=visdrone config, the num_classes shape
# mismatch warning will fire. Only class IDs 0-9 will be mapped; IDs 10-79
# will map to UNKNOWN.
#
# To use actual VisDrone-trained weights:
#   1. Train: model.train(data='VisDrone.yaml', epochs=100, imgsz=640)
#   2. Export the trained model instead of yolov8n.pt
#   3. Verify output shape: [1, 14, 8400] (4 bbox + 10 VisDrone classes)
#
# Requires: pip install ultralytics
#
# Usage:
#   cd <project_root>
#   bash models/download_yolov8n_visdrone.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODEL_PATH="${SCRIPT_DIR}/yolov8n_visdrone.onnx"

if [[ -f "$MODEL_PATH" ]]; then
    echo "Model already exists: $MODEL_PATH ($(du -h "$MODEL_PATH" | cut -f1))"
    exit 0
fi

echo "⚠️  Exporting BASE YOLOv8n (COCO weights) to ONNX — not VisDrone-trained."
echo "   See script header for instructions on training with VisDrone dataset."

python3 -c "
from ultralytics import YOLO
import shutil, os

# BASE YOLOv8n (COCO 80-class) — placeholder until VisDrone-trained weights
# are available from the training pipeline.
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

echo "Done. Set config: perception.detector.dataset=visdrone, perception.detector.num_classes=10"
echo "Note: Shape mismatch warning expected until VisDrone-trained weights are used."
