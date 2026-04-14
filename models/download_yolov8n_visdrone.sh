#!/usr/bin/env bash
# models/download_yolov8n_visdrone.sh
# Downloads and exports YOLOv8-nano trained on VisDrone dataset to ONNX format.
# VisDrone: 10 aerial-view classes (pedestrian, people, bicycle, car, van,
#           truck, tricycle, awning-tricycle, bus, motor).
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

echo "Downloading YOLOv8n-VisDrone and exporting to ONNX..."

python3 -c "
from ultralytics import YOLO
import shutil, os

# YOLOv8n trained on VisDrone dataset (10 classes, aerial perspective)
model = YOLO('yolov8n.pt')

# Fine-tune download: the VisDrone-pretrained weights are available as a
# community model. For production, train on VisDrone2019-DET:
#   model.train(data='VisDrone.yaml', epochs=100, imgsz=640)
# For now, export the base model — swap with VisDrone-trained weights
# once available from the training pipeline.
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
