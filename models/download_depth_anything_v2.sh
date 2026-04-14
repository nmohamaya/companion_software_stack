#!/usr/bin/env bash
# models/download_depth_anything_v2.sh
# Exports Depth Anything V2 ViT-Small to ONNX (~95MB) compatible with OpenCV DNN.
#
# The ViT-S variant balances accuracy and speed for drone applications:
#   - Input: 518x518 RGB
#   - Output: relative inverse depth map (converted to metric in C++ backend)
#   - CPU inference via OpenCV DNN: ~60-80ms per frame
#
# Why export instead of download?
#   HuggingFace hosts PyTorch weights (safetensors) only — no pre-exported ONNX.
#   The PyTorch->ONNX export produces Resize nodes with 4 inputs (ONNX standard),
#   but OpenCV DNN <=4.10 only supports 1-2 input Resize. We fix this with
#   ONNX graph surgery: replace dynamic-size Resize with fixed-scale Resize.
#
# Requires: pip install torch transformers onnx onnxruntime onnxsim
#
# Usage:
#   cd <project_root>
#   bash models/download_depth_anything_v2.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODEL_PATH="${SCRIPT_DIR}/depth_anything_v2_vits.onnx"

if [[ -f "$MODEL_PATH" ]]; then
    echo "Model already exists: $MODEL_PATH ($(du -h "$MODEL_PATH" | cut -f1))"
    exit 0
fi

echo "Exporting Depth Anything V2 ViT-Small to ONNX (OpenCV DNN compatible)..."

python3 -c "
import torch
import numpy as np
import os, shutil

# --- Step 1: Export PyTorch model to ONNX ---
print('Step 1/4: Loading model from HuggingFace...')
from transformers import AutoModelForDepthEstimation
model = AutoModelForDepthEstimation.from_pretrained(
    'depth-anything/Depth-Anything-V2-Small-hf'
)
model.eval()

print('Step 2/4: Exporting to ONNX (opset 14, bilinear resize)...')
# Monkey-patch bicubic -> bilinear (OpenCV DNN doesn't support bicubic Resize)
import torch.nn.functional as F
_orig_interpolate = F.interpolate
def _bilinear_interpolate(*args, **kwargs):
    if kwargs.get('mode') == 'bicubic':
        kwargs['mode'] = 'bilinear'
        kwargs.pop('align_corners', None)
    return _orig_interpolate(*args, **kwargs)
F.interpolate = _bilinear_interpolate

dummy_input = torch.randn(1, 3, 518, 518)
raw_path = '/tmp/da_v2_raw.onnx'
torch.onnx.export(
    model, {'pixel_values': dummy_input},
    raw_path,
    opset_version=14,
    input_names=['pixel_values'],
    output_names=['predicted_depth'],
    dynamic_axes=None,
    dynamo=False,
)
F.interpolate = _orig_interpolate  # Restore

# --- Step 2: Simplify with onnxsim ---
print('Step 3/4: Simplifying ONNX graph...')
import subprocess
sim_path = '/tmp/da_v2_simplified.onnx'
subprocess.run(['onnxsim', raw_path, sim_path], check=True, capture_output=True)

# --- Step 3: Fix Resize nodes for OpenCV DNN compatibility ---
print('Step 4/4: Fixing Resize nodes for OpenCV DNN...')
import onnx
from onnx import helper, numpy_helper, TensorProto
import onnxruntime as ort

model_onnx = onnx.load(sim_path)
graph = model_onnx.graph

# Find all Resize nodes
resize_nodes = [n for n in graph.node if n.op_type == 'Resize']

# Get input/output shapes by running inference with extra outputs
extra_outputs = []
existing_names = {o.name for o in graph.output}
value_info_map = {vi.name: vi for vi in graph.value_info}

for node in resize_nodes:
    for tname in [node.input[0], node.output[0]]:
        if tname and tname not in existing_names:
            vi = value_info_map.get(tname,
                helper.make_tensor_value_info(tname, TensorProto.FLOAT, None))
            extra_outputs.append(vi)
            existing_names.add(tname)

for eo in extra_outputs:
    graph.output.append(eo)
onnx.save(model_onnx, '/tmp/da_v2_debug.onnx')

sess = ort.InferenceSession('/tmp/da_v2_debug.onnx')
dummy_np = np.random.randn(1, 3, 518, 518).astype(np.float32)
results = sess.run(None, {'pixel_values': dummy_np})
shape_map = {name: r.shape for name, r in zip(
    [o.name for o in sess.get_outputs()], results)}

for eo in extra_outputs:
    graph.output.remove(eo)

# Replace 4-input Resize(X, roi, scales, sizes) with 3-input Resize(X, roi, scales)
new_nodes = []
new_inits = []
for node in graph.node:
    if node.op_type != 'Resize' or len(node.input) < 3:
        new_nodes.append(node)
        continue

    in_shape = shape_map.get(node.input[0])
    out_shape = shape_map.get(node.output[0])
    if in_shape is None or out_shape is None:
        new_nodes.append(node)
        continue

    scales = np.array(out_shape, dtype=np.float32) / np.array(in_shape, dtype=np.float32)
    scales_name = f'{node.name}_fixed_scales'
    roi_name = f'{node.name}_empty_roi'
    new_inits.append(numpy_helper.from_array(scales, name=scales_name))
    new_inits.append(numpy_helper.from_array(np.array([], dtype=np.float32), name=roi_name))

    new_resize = helper.make_node('Resize',
        inputs=[node.input[0], roi_name, scales_name],
        outputs=node.output, name=node.name)
    for attr in node.attribute:
        new_resize.attribute.append(attr)
    new_nodes.append(new_resize)

del graph.node[:]
graph.node.extend(new_nodes)
for init in new_inits:
    graph.initializer.append(init)

onnx.checker.check_model(model_onnx)
onnx.save(model_onnx, '${MODEL_PATH}')

# Verify
sess2 = ort.InferenceSession('${MODEL_PATH}')
out = sess2.run(None, {'pixel_values': dummy_np})
size_mb = os.path.getsize('${MODEL_PATH}') / 1e6
print(f'Model saved to ${MODEL_PATH} ({size_mb:.1f} MB)')
print(f'Output shape: {out[0].shape} — ready for OpenCV DNN')

# Cleanup
for f in [raw_path, sim_path, '/tmp/da_v2_debug.onnx']:
    if os.path.exists(f):
        os.remove(f)
"

echo "Done. Set config: perception.depth_estimator.backend=depth_anything_v2"
