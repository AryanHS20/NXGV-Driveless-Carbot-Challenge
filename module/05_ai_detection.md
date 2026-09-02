# Module 5: BPU AI Object Detection Pipeline

The Horizon RDK X5 developer kit features a dedicated hardware accelerator called the **BPU (Brain Processing Unit)**. The BPU can run neural network models (like YOLOv5) extremely fast and with low power consumption by offloading inference from the CPU.

---

## 1. The BPU Object Detection Pipeline

Converting a custom AI model to run on the BPU hardware involves a specific pipeline:

```text
  [1. Dataset Collection] ──> [2. Train YOLOv5 (PyTorch)] ──> [3. Export to ONNX]
                                                                      │
  [5. Write ROS 2 Node] <── [4. Convert ONNX to .bin Model] <─────────┘
  (Using hobot_dnn library)      (Using Horizon Toolchain)
```

### 1.1. Dataset Collection & Labeling
Collect images of your target objects (e.g., bumper signboards, traffic lights, roundabouts) under various lighting conditions. Annotate the bounding boxes using tools like Roboflow or CVAT in YOLO format.

### 1.2. Model Selection (YOLOv5 Only)
> [!IMPORTANT]
> **Strict Version Constraint:**
> You must strictly use **YOLOv5** for custom vision models. The Horizon toolchain compiler for the RDK X5 has native optimization layers specifically built for YOLOv5 architecture. Trying to compile other model versions (like YOLOv8 or custom PyTorch CNNs) will fail during toolchain conversion.

### 1.3. Train & Export to ONNX
Train your YOLOv5 model on PyTorch. Once training is complete, export the weight file (`.pt`) to the open standard ONNX format:
```bash
python export.py --weights best.pt --include onnx --imgsz 640 640
```

### 1.4. Horizon Toolchain Conversion (ONNX to .bin)
The BPU cannot execute raw `.pt` or `.onnx` files directly. You must convert the ONNX model into a compiled BPU binary (`.bin`) format using the **Horizon Toolchain Compiler**:
- Define a calibration configuration file (`config.yaml`) specifying input node details, model types, and quantization settings (converting FP32 weights to INT8 to run on BPU).
- Compile the model using the toolchain script:
  ```bash
  hb_mapper maketbin --config config.yaml
  ```
- This compiles your model to a hobot_dnn compatible `.bin` file, optimized to execute on the BPU hardware accelerator cores.

---

## 2. Running BPU Inference in Python

On RISA-bot, the BPU inference is implemented in `signage_detector.py`. 

### 2.1. Loading the Model
The node uses `pyeasy_dnn` to load the compiled `.bin` file:
```python
from hobot_dnn import pyeasy_dnn as dnn
# Load model
self.models = dnn.load('/home/sunrise/risabot_signs_640x640_nv12.bin')
self.model = self.models[0]
```

### 2.2. Image Preprocessing (NV12 Layout)
The BPU requires images in the **NV12 YUV color layout** (planar Y component followed by interleaved U and V components) instead of standard RGB/BGR. 

The node resizes the raw camera frame to `640x640` and converts it as follows:
```python
# Convert BGR to YUV I420 format
yuv = cv2.cvtColor(bgr_image_640x640, cv2.COLOR_BGR2YUV_I420)
# Extract planes and construct NV12 layout
y = yuv[0:640, :]
u = yuv[640:800, :]
v = yuv[800:960, :]
uv_planar = np.stack([u.ravel(), v.ravel()], axis=1).ravel().reshape(320, 640)
nv12_image = np.vstack((y, uv_planar))
```

### 2.3. Inference and Post-Processing
The NV12 image array is sent to the BPU core:
```python
# Run hardware forward inference
outputs = self.model.forward([nv12_image])
raw_predictions = outputs[0].buffer
```
The node then processes the output:
1. Calculates scores: `scores = objectness * class_probability`.
2. Filters detections by per-class confidence thresholds.
3. Performs Non-Maximum Suppression (NMS) to eliminate overlapping bounding boxes.

![Signage Detector BPU Debug Stream Overlay](images/signage_detector_debug.png)
<!-- Image Description: Screenshot of the camera debug stream with color-coded bounding boxes around traffic lights and warning signs, showing class labels and confidence percentage text overlays. -->

---
**Previous:** [Module 4 — LiDAR Operations](04_lidar.md)
**Next:** [Module 6 — RISA-bot Features](06_risabot_features.md)
