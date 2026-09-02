# RISAbot BPU Model Training Guide

> **Audience:** Anyone who wants to train, retrain, or extend the RISAbot YOLOv5s object detection model and compile it for hardware-accelerated inference on the RDK X5's Brain Processing Unit (BPU).
>
> **Time estimate:** ~1.5 to 2 hours total (mostly waiting for Colab to train).
>
> **This guide covers everything from scratch**, including Roboflow dataset setup, adding new classes, training on Google Colab, compiling with Docker, and deploying to the robot.

---

## Table of Contents

1. [Overview — The Full Pipeline](#1-overview--the-full-pipeline)
2. [Roboflow — Dataset Setup](#2-roboflow--dataset-setup)
   - [2.1 What Roboflow Is](#21-what-roboflow-is)
   - [2.2 Creating a Project](#22-creating-a-project)
   - [2.3 What a Good Dataset Looks Like](#23-what-a-good-dataset-looks-like)
   - [2.4 Uploading Images](#24-uploading-images)
   - [2.5 Annotating (Labeling) Images](#25-annotating-labeling-images)
   - [2.6 Current RISAbot Classes](#26-current-risabot-classes)
   - [2.7 Adding a New Detection Class](#27-adding-a-new-detection-class)
   - [2.8 Train / Validation Split](#28-train--validation-split)
   - [2.9 Augmentation Settings](#29-augmentation-settings)
   - [2.10 Generating a Version and Getting Your API Key](#210-generating-a-version-and-getting-your-api-key)
3. [Google Colab — Training the Model](#3-google-colab--training-the-model)
   - [3.1 Setting Up the Notebook](#31-setting-up-the-notebook)
   - [3.2 Cell 1 — Clone YOLOv5 v7.0](#32-cell-1--clone-yolov5-v70)
   - [3.3 Cell 2 — Download Dataset from Roboflow](#33-cell-2--download-dataset-from-roboflow)
   - [3.4 Cell 3 — Re-split Dataset (80/20 Fix)](#34-cell-3--re-split-dataset-8020-fix)
   - [3.5 Cell 4 — Write data.yaml](#35-cell-4--write-datayaml)
   - [3.6 Cell 4.5 — PyTorch 2.6+ Patch](#36-cell-45--pytorch-26-patch)
   - [3.7 Cell 5 — Train YOLOv5s](#37-cell-5--train-yolov5s)
   - [3.8 Cell 6 — Export to ONNX](#38-cell-6--export-to-onnx)
   - [3.9 Cell 7 — Generate Calibration Data](#39-cell-7--generate-calibration-data)
   - [3.10 Cell 8 — Package and Download](#310-cell-8--package-and-download)
4. [Docker on Windows — BPU Compilation](#4-docker-on-windows--bpu-compilation)
   - [4.1 Prerequisites](#41-prerequisites)
   - [4.2 Place Files in the Right Location](#42-place-files-in-the-right-location)
   - [4.3 Understanding risabot_bpu_config.yaml](#43-understanding-risabot_bpu_configyaml)
   - [4.4 Run compile_model.bat](#44-run-compile_modelbat)
   - [4.5 If hb_mapper Fails — The ONNX Patch Script](#45-if-hb_mapper-fails--the-onnx-patch-script)
   - [4.6 Output Files Explained](#46-output-files-explained)
5. [Deployment to RDK X5](#5-deployment-to-rdk-x5)
   - [5.1 Copy the Model to the Robot](#51-copy-the-model-to-the-robot)
   - [5.2 Static Verification](#52-static-verification)
   - [5.3 Live Camera Verification](#53-live-camera-verification)
   - [5.4 If You Added a New Class — Update the ROS2 Node](#54-if-you-added-a-new-class--update-the-ros2-node)
6. [Quick Reference](#6-quick-reference)
   - [6.1 Troubleshooting Table](#61-troubleshooting-table)
   - [6.2 All Commands at a Glance](#62-all-commands-at-a-glance)
   - [6.3 Files in tools/bpu_model/ Explained](#63-files-in-toolsbpu_model-explained)

---

## 1. Overview — The Full Pipeline

### What the BPU Is

The RDK X5 contains a dedicated **Brain Processing Unit (BPU)** — a fixed-function neural network accelerator rated at **10 TOPS** (tera operations per second). Unlike a CPU or GPU, the BPU does one job: running compiled AI models as fast and as efficiently as possible.

Running YOLOv5s on the CPU would give ~1–2 FPS and consume most of the processing headroom. The same model on the BPU runs at **30+ FPS** with the CPU nearly idle — leaving full headroom for the rest of the ROS2 stack (lane following, LiDAR, motor control, etc.).

**The BPU only accepts `.bin` files produced by Horizon Robotics' `hb_mapper` compiler.** It cannot execute ONNX, PyTorch `.pt`, or TFLite files directly. The compilation step is mandatory.

### The Pipeline at a Glance

```
┌────────────────┐     ┌──────────────────────────┐     ┌───────────────────────┐     ┌──────────────────────┐
│   ROBOFLOW     │────▶│      GOOGLE COLAB         │────▶│   DOCKER (Windows PC) │────▶│     RDK X5 ROBOT     │
│                │     │                           │     │                       │     │                      │
│ • Upload images│     │ • Clone YOLOv5 v7.0       │     │ • hb_mapper compiler  │     │ • Copy .bin to robot │
│ • Label images │     │ • Download dataset        │     │ • INT8 quantization   │     │ • Run verify_bpu.py  │
│ • Set classes  │     │ • Train YOLOv5s (~30 min) │     │ • Output .bin model   │     │ • Launch ROS2 stack  │
│ • Export v2    │     │ • Export ONNX (opset 11)  │     │                       │     │                      │
│                │     │ • Generate calibration data│     │                       │     │                      │
│                │     │ • Download bpu_package.zip│     │                       │     │                      │
└────────────────┘     └──────────────────────────┘     └───────────────────────┘     └──────────────────────┘
    [Browser]                [Cloud GPU — T4]               [Your Windows PC]              [ARM64 Linux]
    ~30 min                  ~30–60 min                       ~5 min                        ~2 min
```

### Prerequisites Checklist

Before starting, make sure you have:

- [ ] **Google Account** — for Google Colab (free, no GPU subscription needed, free T4 is enough)
- [ ] **Roboflow Account** — free at [app.roboflow.com](https://app.roboflow.com)
- [ ] **Docker Desktop** — installed and running on your Windows PC. Download from [docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop). Requires Windows 10 version 2004 or later with WSL2 (Docker installs WSL2 automatically).
- [ ] **SSH access to the robot** — `ssh sunrise@<ROBOT_IP>` (default password: `sunrise`)
- [ ] The `risabotcar_ws` repository cloned and available on your local PC

---

## 2. Roboflow — Dataset Setup

### 2.1 What Roboflow Is

[Roboflow](https://roboflow.com) is a browser-based platform for managing computer vision datasets. It handles everything in one place:

- Image upload and organization
- Bounding box annotation (labeling)
- Dataset versioning (so you can add more data later without losing old work)
- Exporting in different formats (we need **YOLOv5 PyTorch** format)

The **free tier** allows unlimited public projects with up to 10,000 images — more than enough for this project.

### 2.2 Creating a Project

1. Log in to [app.roboflow.com](https://app.roboflow.com)
2. Click **Create New Project**
3. Set the following:
   - **Project Name:** Something descriptive, e.g., `risabot-signs`
   - **License:** Keep as default
   - **Project Type:** `Object Detection (Bounding Box)` ← important, do not choose classification
   - **Annotation Group:** e.g., `signs` (this is just an internal label grouper, doesn't affect training)
4. Click **Create Public Project**

Your project URL will look like:
`https://app.roboflow.com/your-workspace-name/risabot-signs`

Note your **workspace name** — you will need it in the Colab script (`rf.workspace("your-workspace-name")`).

### 2.3 What a Good Dataset Looks Like

The quality of your dataset is the single biggest factor in detection accuracy. Here are the rules:

#### Minimum Image Count

| Situation | Minimum images per class |
|-----------|--------------------------|
| Very simple object (solid color, consistent shape) | 50–80 |
| Standard sign/object | **100–200** (recommended) |
| Complex object with many variants | 200–500+ |

The current RISAbot model uses ~200 training images across 6 classes.

#### Image Variety — This Is Critical

Collect images that cover the conditions the robot will encounter in the real world:

| Vary This | Examples |
|-----------|----------|
| **Lighting** | Bright room, dim room, outdoor, shadows, backlighting |
| **Angle** | Straight-on, 15° tilt, 30° tilt, slight above/below |
| **Distance** | Very close (fills frame), medium (occupies ~20% of frame), far (small in frame) |
| **Background** | Different walls, floors, clutter behind the object |
| **Orientation** | Slight rotation, imperfectly placed on surface |

> **Common mistake:** Taking 100 photos of a sign against the same white wall, at the same distance, under the same lighting. The model will learn the wall as much as the sign, and fail when the environment changes.

> **Best practice:** Capture images **directly from the robot's camera** during actual test runs. These images already match the exact color profile, resolution, and perspective the model will see at inference time.

#### What to Avoid

- Blurry images (camera motion during capture)
- Images where the object is partially cut off at the frame edge
- Images with the same scene repeated many times (near-duplicates)
- Images that are too dark to see the object clearly

### 2.4 Uploading Images

1. Inside your Roboflow project, click **Upload Data**
2. Drag and drop your images (JPG or PNG, any resolution — Roboflow handles resizing on export)
3. Click **Save and Continue**
4. Roboflow will run duplicate detection — remove any flagged duplicates

> If you are adding images to an **existing project** to expand a class, go to the project → **Dataset** → **Upload** and follow the same process. Your old annotations will not be affected.

### 2.5 Annotating (Labeling) Images

After uploading, you need to draw bounding boxes around every object in every image.

#### Step-by-Step

1. Click on an image in the **Annotate** section
2. Click the **Bounding Box** tool in the left toolbar (keyboard shortcut: `B`)
3. Click and drag to draw a rectangle tightly around the object
4. When you release, a dialog asks for a class name — type the class name and press Enter
5. If the object appears multiple times in one image, draw a separate box for each instance
6. Click the **>** arrow (or press the right arrow key) to move to the next image
7. Repeat until all images are annotated

#### Annotation Quality Rules

- **Draw boxes tight.** The box should touch the edges of the object, not include large amounts of background. A box that includes 50% background teaches the model that background is part of the object.
- **Label every instance.** If a parking sign appears twice in one image, draw two boxes. Unlabeled objects cause false negatives during training (the model is penalized for detecting something that wasn't labeled).
- **Be consistent.** If you label a sign that's 10% occluded in one image, label similar occlusion levels in all images.

#### Class Naming Rules

> **These rules are important.** Incorrect class names will cause a mismatch between Roboflow exports, the `data.yaml` config, and the ROS2 node.

- Use **lowercase** letters only
- Use **underscores** for spaces: `hill_sign` ✅, `Hill Sign` ❌, `hillsign` ❌
- Class names are **case-sensitive** — `Hill_Sign` and `hill_sign` are different classes
- Roboflow **assigns class indices alphabetically** when exporting. This means:
  - `hill_sign` → index 0
  - `parking_sign` → index 1
  - `traffic_light` → index 2
  - ...etc.
  - The order in `data.yaml` must match this alphabetical order exactly

### 2.6 Current RISAbot Classes

The current RISAbot model detects 6 classes:

| Index | Class Name | What It Detects | ROS2 Output Topic |
|-------|------------|-----------------|-------------------|
| 0 | `hill_sign` | The hill/ramp signboard | `/hill_sign_detected` (Bool) |
| 1 | `parking_sign` | The parking signboard | `/parking_signboard_detected` (Bool) |
| 2 | `traffic_light` | Unlit traffic light housing | `/traffic_light_state` ("unknown") |
| 3 | `traffic_light_green` | Green light illuminated | `/traffic_light_state` ("green") |
| 4 | `traffic_light_red` | Red light illuminated | `/traffic_light_state` ("red") |
| 5 | `traffic_light_yellow` | Yellow light illuminated | `/traffic_light_state` ("yellow") |

### 2.7 Adding a New Detection Class

Follow this checklist in order when adding a new class (e.g., `speedbump_sign`):

#### Step 1 — Add the Class in Roboflow

1. Open the Roboflow annotation editor
2. In the **Classes** panel on the right, click **+ Add Class**
3. Type the new class name exactly as it will appear in training: e.g., `speedbump_sign`
4. Go back through existing annotated images — if any of them contain the new object, annotate it now
5. Upload new images specifically showing the new object and annotate them

#### Step 2 — Determine the New Class Index

Roboflow exports classes in **alphabetical order**. Insert your new class name into the alphabetical list:

Current classes sorted alphabetically:
```
hill_sign         → 0
parking_sign      → 1
speedbump_sign    → 2  ← NEW (inserts between parking_sign and traffic_light)
traffic_light     → 3  ← shifted
traffic_light_green → 4  ← shifted
traffic_light_red → 5  ← shifted
traffic_light_yellow → 6  ← shifted
```

> **Warning:** Adding a class in the middle shifts all indices below it. This means the existing `.bin` model is now wrong — you **must** retrain from scratch.

#### Step 3 — Update `colab_training_script.py` (Cell 4)

Edit the `data_yaml_content` dictionary:

```python
data_yaml_content = {
    'train': train_img_dir,
    'val': val_img_dir,
    'nc': 7,                          # ← was 6, now 7
    'names': [
        'hill_sign',                  # Class 0
        'parking_sign',               # Class 1
        'speedbump_sign',             # Class 2  ← NEW
        'traffic_light',              # Class 3
        'traffic_light_green',        # Class 4
        'traffic_light_red',          # Class 5
        'traffic_light_yellow'        # Class 6
    ]
}
```

#### Step 4 — Update the ROS2 Node

After retraining and deploying, update `src/risabot_automode/risabot_automode/signage_detector.py` to handle the new class ID. Find the class ID → topic mapping section and add:

```python
elif class_id == 2:  # speedbump_sign (new index)
    self.speedbump_pub.publish(Bool(data=True))
```

And shift all other class IDs that moved (3, 4, 5, 6 instead of 2, 3, 4, 5).

#### Step 5 — Update the Roboflow Version Number

Generate a new dataset version in Roboflow (e.g., v3) and update Cell 2 of the Colab script:
```python
dataset = project.version(3).download("yolov5")  # ← bump version number
```

#### Step 6 — Retrain, Recompile, Redeploy

Follow the full workflow from Section 3 onwards.

### 2.8 Train / Validation Split

#### What the Validation Split Is For

During training, at the end of every epoch, YOLOv5 runs the current model against the **validation set** and reports mAP (mean Average Precision). This tells you how well the model is learning without overfitting to the training data.

> The validation set must never be seen during training. If training images leak into validation, the mAP score will be artificially high and the model will perform worse in the real world.

#### How to Set the Split in Roboflow

1. Inside your project, go to **Dataset** (left sidebar)
2. At the top you will see the split indicator: `Train | Valid | Test` with percentages
3. Click **Rebalance** to adjust
4. Recommended: **80% Train, 20% Valid, 0% Test**

For a 200-image dataset, this gives ~160 training images and ~40 validation images.

#### The Roboflow Auto-Split Quirk (Important)

When you first add images to a Roboflow project, it may automatically assign almost all images to `train` and only 1 or 2 to `valid`. This causes YOLOv5 to crash during training with a "no labels found" or "division by zero" error.

**The Cell 3 script in `colab_training_script.py` fixes this automatically** by:
1. Merging the Roboflow validation set back into train
2. Re-splitting 20% into validation with a fixed random seed

You do not need to fix the split in Roboflow manually — Cell 3 handles it.

### 2.9 Augmentation Settings

When generating a dataset version in Roboflow, it offers to apply augmentations (flip, brightness, crop, etc.) during export.

> **Do NOT enable Roboflow augmentations for this pipeline.**

YOLOv5's `hyp.VOC.yaml` augmentation preset (used in Cell 5) already applies comprehensive augmentations during training:
- Mosaic (combines 4 images into one)
- HSV color jitter (random hue, saturation, brightness shifts)
- Random horizontal flips
- Random scale, translate, and perspective
- Copy-paste augmentation

Enabling Roboflow augmentations on top of this would inflate the dataset download size, slow down the Colab cells, and produce diminishing returns.

**In Roboflow:** When generating a version, set the augmentation multiplier to `1x` (no augmentation). Leave all augmentation toggles off.

### 2.10 Generating a Version and Getting Your API Key

#### Generate a Dataset Version

1. In your Roboflow project, click **Generate New Version**
2. Skip augmentations (leave at 1x)
3. Click **Generate** → Roboflow will process the images
4. The version number appears in the URL and in the Versions list (e.g., `Version 2`)
5. Note this version number — it goes in the Colab script

#### Get Your Roboflow API Key

1. Click your **profile icon** (top right) → **Settings**
2. Go to **Roboflow API** tab
3. Click **Copy** next to your Private API Key

This is a personal key — keep it private. Do not share it publicly or commit it to Git.

You will paste it into Cell 2 of the Colab script:
```python
rf = Roboflow(api_key="PASTE_YOUR_KEY_HERE")
```

---

## 3. Google Colab — Training the Model

The complete training script is in `tools/bpu_model/colab_training_script.py`. Open this file on your local PC and copy each `# CELL X` block into a separate Colab code cell.

### 3.1 Setting Up the Notebook

1. Go to [colab.research.google.com](https://colab.research.google.com)
2. Click **New Notebook**
3. Set the GPU runtime: **Runtime → Change runtime type → Hardware accelerator → T4 GPU → Save**
4. You should now see a green checkmark and "T4" in the top-right corner
5. Create 8 code cells (one per Cell block in the training script)

> **Important:** Google Colab sessions time out after ~12 hours of inactivity (free tier) and after ~3 hours without a connected browser. Do not close the browser tab during training.

### 3.2 Cell 1 — Clone YOLOv5 v7.0

```python
!git clone --branch v7.0 https://github.com/ultralytics/yolov5.git
%cd yolov5
!pip install -qr requirements.txt
!pip install roboflow numpy pillow opencv-python
```

**Why `--branch v7.0` exactly?**

The BPU compiler (`hb_mapper`) supports a **maximum of ONNX opset 11**. YOLOv5 releases after v7.0 produce opset 17 or 18 by default, which `hb_mapper` rejects immediately with an opset error. YOLOv5 v7.0 exports opset 11 correctly when the `--opset 11` flag is used.

This is a hard constraint — do not use the `main` branch or any release newer than v7.0.

**Expected output:** A stream of pip install messages. Ends with `Successfully installed ...`. This takes about 2 minutes.

### 3.3 Cell 2 — Download Dataset from Roboflow

```python
from roboflow import Roboflow
import os

rf = Roboflow(api_key="PASTE_YOUR_KEY_HERE")   # ← replace with your key

project = rf.workspace("your-workspace-name").project("your-project-name")
dataset = project.version(2).download("yolov5")  # ← use your version number

dataset_dir = dataset.location
print(f"Dataset downloaded to: {dataset_dir}")
```

**What to change:**

| Variable | Where to find it |
|----------|-----------------|
| `api_key` | Roboflow → Settings → Roboflow API |
| `workspace` | Your Roboflow workspace name (in the URL: `app.roboflow.com/WORKSPACE/project`) |
| `project` | Your Roboflow project name (in the URL: `app.roboflow.com/workspace/PROJECT`) |
| `version(2)` | The version number you generated in Section 2.10 |

**Expected output:** A progress bar downloading the dataset ZIP, then the path it was extracted to.

### 3.4 Cell 3 — Re-split Dataset (80/20 Fix)

```python
import glob, os, random, shutil

train_img_dir = os.path.join(dataset_dir, "train", "images")
train_lbl_dir = os.path.join(dataset_dir, "train", "labels")
val_img_dir   = os.path.join(dataset_dir, "valid", "images")
val_lbl_dir   = os.path.join(dataset_dir, "valid", "labels")

os.makedirs(val_img_dir, exist_ok=True)
os.makedirs(val_lbl_dir, exist_ok=True)

# Move everything from valid/ back to train/ first
val_images = glob.glob(os.path.join(val_img_dir, "*"))
for img_path in val_images:
    filename = os.path.basename(img_path)
    label_filename = os.path.splitext(filename)[0] + ".txt"
    lbl_path = os.path.join(val_lbl_dir, label_filename)
    if os.path.exists(img_path):
        shutil.move(img_path, os.path.join(train_img_dir, filename))
    if os.path.exists(lbl_path):
        shutil.move(lbl_path, os.path.join(train_lbl_dir, label_filename))

# Re-split 20% to validation
all_train_images = glob.glob(os.path.join(train_img_dir, "*"))
random.seed(42)
random.shuffle(all_train_images)
val_split_count = int(len(all_train_images) * 0.20)
val_images_to_move = all_train_images[:val_split_count]

print(f"Total merged train images: {len(all_train_images)}")
print(f"Moving {val_split_count} images to validation split...")

for img_path in val_images_to_move:
    filename = os.path.basename(img_path)
    label_filename = os.path.splitext(filename)[0] + ".txt"
    lbl_path = os.path.join(train_lbl_dir, label_filename)
    shutil.move(img_path, os.path.join(val_img_dir, filename))
    if os.path.exists(lbl_path):
        shutil.move(lbl_path, os.path.join(val_lbl_dir, label_filename))

print(f"Split complete. Train: {len(glob.glob(os.path.join(train_img_dir, '*')))}, Valid: {len(glob.glob(os.path.join(val_img_dir, '*')))}")
```

**What this does:**
1. Moves all images from Roboflow's `valid/` folder back into `train/` (starting fresh)
2. Randomly picks 20% of the combined set and moves them to `valid/`
3. The random seed is fixed (`42`) so the split is reproducible

**Expected output:**
```
Total merged train images: 197
Moving 39 images to validation split...
Split complete. Train: 158, Valid: 39
```

If your numbers are very different (e.g., Train: 2, Valid: 1), the dataset download may have failed — re-run Cell 2.

### 3.5 Cell 4 — Write data.yaml

```python
import yaml

data_yaml_content = {
    'train': train_img_dir,
    'val': val_img_dir,
    'nc': 6,
    'names': [
        'hill_sign',             # Class 0
        'parking_sign',          # Class 1
        'traffic_light',         # Class 2
        'traffic_light_green',   # Class 3
        'traffic_light_red',     # Class 4
        'traffic_light_yellow'   # Class 5
    ]
}

data_yaml_path = os.path.join(dataset_dir, "data.yaml")
with open(data_yaml_path, 'w') as f:
    yaml.dump(data_yaml_content, f, default_flow_style=None)

print(f"Updated data.yaml written to: {data_yaml_path}")
```

**What to change if you added a new class:**

- Change `'nc': 6` to `'nc': 7` (or however many classes you now have)
- Add the new class name to the `names` list in **alphabetical order**
- Make sure the order matches what Roboflow exports (alphabetical)

**Why we overwrite Roboflow's data.yaml:** Roboflow's exported `data.yaml` uses relative paths that don't work inside Colab's directory structure. This cell writes an absolute-path version.

### 3.6 Cell 4.5 — PyTorch 2.6+ Patch

```python
patch_code = '''import torch
_org_load = torch.load
def _patched_load(*args, **kwargs):
    if 'weights_only' not in kwargs:
        kwargs['weights_only'] = False
    return _org_load(*args, **kwargs)
torch.load = _patched_load
'''

import os
if os.path.basename(os.getcwd()) != 'yolov5':
    if os.path.exists('yolov5'):
        os.chdir('yolov5')

for filename in ['train.py', 'export.py']:
    if os.path.exists(filename):
        with open(filename, 'r') as f:
            code = f.read()
        if 'patched_load' not in code:
            print(f"Patching {filename}...")
            with open(filename, 'w') as f:
                f.write(patch_code + "\n" + code)
```

**Why this patch is needed:**

PyTorch 2.6 (released early 2025) changed the default value of the `weights_only` parameter in `torch.load()` from `False` to `True`. YOLOv5 v7.0 was written before this change and calls `torch.load()` without specifying this parameter. On modern Colab environments (which use PyTorch 2.6+), this causes an immediate crash when loading the pretrained `yolov5s.pt` weights:

```
_pickle.UnpicklingError: Weights only load failed.
```

This patch prepends a monkey-patch to `train.py` and `export.py` that forces `weights_only=False` on every `torch.load()` call — matching the original intended behavior.

**Expected output:**
```
Patching train.py...
Patching export.py...
```

If it prints nothing, the files were already patched (from a previous run) — that's fine.

### 3.7 Cell 5 — Train YOLOv5s

```python
!python train.py --img 640 --batch-size 8 --epochs 150 --data {data_yaml_path} \
  --weights yolov5s.pt --hyp data/hyps/hyp.VOC.yaml --patience 30 --name risabot_signs
```

**Every flag explained:**

| Flag | Value | Why |
|------|-------|-----|
| `--img` | `640` | Input resolution. Must match the BPU config (`input_shape: 1x3x640x640`). Larger = slower training but better small-object detection. |
| `--batch-size` | `8` | Number of images processed per gradient update. Small datasets need small batch sizes to avoid unstable gradients. |
| `--epochs` | `150` | Maximum number of full passes through the training data. Early stopping will trigger before this if the model converges. |
| `--weights` | `yolov5s.pt` | Start from pretrained COCO weights (transfer learning). This is much faster than training from random initialization and achieves better results with small datasets. |
| `--hyp` | `hyp.VOC.yaml` | Augmentation and hyperparameter preset designed for small datasets. Applies mosaic, HSV color jitter, random flips, perspective distortion. |
| `--patience` | `30` | Early stopping: if mAP@0.5 does not improve for 30 consecutive epochs, training stops automatically. |
| `--name` | `risabot_signs` | Folder name for results: `runs/train/risabot_signs/` |

**Expected training time:** 20–40 minutes on a Colab T4 GPU.

**What to watch in the training log:**

The training log prints a table every epoch. The important columns are:

```
  Epoch    GPU_mem   box_loss   obj_loss   cls_loss  Instances       Size
  0/149      3.52G      0.089      0.057      0.052         98        640:
               Class     Images  Instances      P          R     mAP50  mAP50-95
                 all        39        142      0.741     0.680      0.72     0.421
```

- `mAP50` — main metric. Aim for **> 0.85** (> 0.90 is excellent)
- `box_loss` — should decrease steadily over epochs
- If `mAP50` is stuck below 0.5 after 50 epochs, check your annotations and dataset quality

**Where results are saved:** `runs/train/risabot_signs/weights/best.pt` (the checkpoint with the highest mAP)

### 3.8 Cell 6 — Export to ONNX

```python
!python export.py --weights runs/train/risabot_signs/weights/best.pt \
  --img 640 --include onnx --opset 11
```

**Why `--opset 11`?**

ONNX opset is the version of the ONNX operator set used. The Horizon Robotics `hb_mapper` compiler (v1.24.3) supports a **maximum of opset 11**. If you omit this flag or use a higher value, the compiler will reject the model with an error like:

```
ValueError: Opset 17 is not supported. Supported opsets: ≤ 11
```

The export will produce `runs/train/risabot_signs/weights/best.onnx` (~28 MB).

### 3.9 Cell 7 — Generate Calibration Data

```python
import cv2
import numpy as np

calibration_dir = "/content/calibration_data"
os.makedirs(calibration_dir, exist_ok=True)

train_images = glob.glob(os.path.join(train_img_dir, "*"))
random.shuffle(train_images)
selected_cal_images = train_images[:50]

print(f"Generating {len(selected_cal_images)} calibration files in {calibration_dir}...")

for idx, img_path in enumerate(selected_cal_images):
    img = cv2.imread(img_path)
    if img is None:
        continue
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)          # BGR → RGB
    img_resized = cv2.resize(img_rgb, (640, 640),
                             interpolation=cv2.INTER_LINEAR)  # resize to model input
    img_chw = np.transpose(img_resized, (2, 0, 1))           # HWC → CHW
    img_chw = img_chw.astype(np.uint8)                       # ensure uint8

    bin_path = os.path.join(calibration_dir, f"calib_{idx:03d}.bin")
    img_chw.tofile(bin_path)                                 # write raw binary

print("Calibration data generation complete.")
```

**What calibration data is for:**

When `hb_mapper` compiles the model, it performs **INT8 quantization** — converting the model's 32-bit floating point weights and activations into 8-bit integers. This reduces the model size by ~4x and is required for BPU execution.

To quantize accurately, the compiler needs to measure the activation range (min/max values) of each layer using real input images. These 50 images are the "calibration set."

**The exact format required:**

Each `.bin` file must be:
- Raw binary pixel data (no file header, no compression)
- Shape: `(3, 640, 640)` in **CHW order** (Channel first, then Height, then Width)
- Data type: `uint8` (values 0–255)
- File size: `3 × 640 × 640 = 1,228,800 bytes` per file — exactly

> **Critical:** Do NOT save as `.npy`, `.jpg`, or `.png`. `hb_mapper` reads raw binary only. If the file size is wrong, the compiler will fail with a reshape error.

**Expected output:** 50 files named `calib_000.bin` through `calib_049.bin`, each exactly 1,228,800 bytes.

### 3.10 Cell 8 — Package and Download

```python
!zip -q -r /content/bpu_package.zip \
  /content/yolov5/runs/train/risabot_signs/weights/best.onnx \
  /content/calibration_data

print("---------------------------------------------------------------")
print("Done! Download /content/bpu_package.zip from Colab.")
print("---------------------------------------------------------------")
```

**To download the zip:**

1. In Colab, click the **folder icon** in the left sidebar to open the Files panel
2. Navigate to the root (`/content/`)
3. Right-click `bpu_package.zip` → **Download**
4. Save it somewhere easy to find on your Windows PC (e.g., `Downloads/`)

The zip contains:
- `best.onnx` — the trained ONNX model
- `calibration_data/` — the 50 `.bin` calibration files

---

## 4. Docker on Windows — BPU Compilation

### 4.1 Prerequisites

1. **Docker Desktop is open and running.** Open it from the Start menu and wait for the bottom-left status indicator to show **"Engine running"** (green dot). Do not proceed until you see this — Docker commands will fail if the engine is not ready.

2. **Pull the Horizon Robotics toolchain image** (one-time, ~4 GB download). Open PowerShell and run:
   ```powershell
   docker pull openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8
   ```
   Wait for the pull to complete. Once pulled, the image stays on your machine permanently and this step does not need to be repeated.

### 4.2 Place Files in the Right Location

Extract `bpu_package.zip` and place the following inside `tools/bpu_model/` (in the repository):

```
tools/bpu_model/
├── best.onnx                  ← from Colab output
├── calibration_data/          ← from Colab output
│   ├── calib_000.bin
│   ├── calib_001.bin
│   └── ... (50 files total)
├── risabot_bpu_config.yaml    ← already in repo, do not modify
└── compile_model.bat          ← already in repo
```

> If `best.onnx` already exists in this folder from a previous run, overwrite it with the new one.

### 4.3 Understanding risabot_bpu_config.yaml

The file `tools/bpu_model/risabot_bpu_config.yaml` tells `hb_mapper` how to compile the model. Here is what each setting means:

```yaml
model_parameters:
  onnx_model: './best.onnx'          # Path to the ONNX model (relative to working dir)
  march: 'bayes-e'                   # BPU architecture. RDK X5 uses Bayes-E. DO NOT CHANGE.
  layer_out_dump: False              # Disable per-layer output dumping (slow, debug only)
  working_dir: 'model_output'        # Folder where all output files are written
  output_model_file_prefix: 'risabot_signs_640x640_nv12'  # Prefix for all output file names

input_parameters:
  input_name: 'images'               # Must match the ONNX model's input tensor name
  input_type_rt: 'nv12'             # Format at RUNTIME: NV12 (what the camera produces)
  input_type_train: 'rgb'            # Format during TRAINING: RGB (how images were loaded in Colab)
  input_layout_train: 'NCHW'        # PyTorch standard layout: Batch × Channel × Height × Width
  input_shape: '1x3x640x640'        # Input tensor shape: 1 image, 3 channels, 640×640 pixels
  norm_type: 'data_scale'           # Normalization method: multiply by scale_value
  scale_value: 0.003921568627451    # = 1/255. Converts pixels from [0,255] to [0,1] at runtime.
                                    # This is baked into the BPU model — no normalization needed
                                    # in your inference code at runtime.

calibration_parameters:
  cal_data_dir: './calibration_data' # Path to the 50 .bin calibration files
  cal_data_type: 'uint8'            # Must match the dtype used when generating calib files
  calibration_type: 'default'       # Standard calibration algorithm

compiler_parameters:
  compile_mode: 'latency'           # Optimize for inference speed (vs. 'throughput' for batch)
  optimize_level: 'O3'              # Maximum optimization. O3 = most aggressive fusion and pruning.
  debug: False                      # Disable debug output
```

> **You do not need to modify this file** unless you change the model name prefix or input resolution.

### 4.4 Run compile_model.bat

1. Open Windows Explorer and navigate to `tools/bpu_model/`
2. **Double-click `compile_model.bat`**

The script will:
1. Check that `best.onnx` exists in the folder
2. Check that `calibration_data/` exists in the folder
3. Start Docker and mount the current folder as `/workspace` inside the container
4. Run `hb_mapper makertbin --config risabot_bpu_config.yaml --model-type onnx`
5. The compiler reads the ONNX model, runs INT8 calibration on the 50 images, and produces the binary

**What you will see:**

- First: Docker output as the container starts
- Then: `hb_mapper` log — it reads the config, validates the model, loads each calibration file, runs quantization, and compiles

**Calibration takes ~1–2 minutes.** You will see each `.bin` file being read:
```
Read raw file: /workspace/calibration_data/calib_000.bin
Read raw file: /workspace/calibration_data/calib_001.bin
...
```

**Compilation takes ~2–5 minutes total.**

**Success output:**
```
==============================================================
[SUCCESS] BPU Model Compilation Completed!
==============================================================

Output files are located inside: 'model_output/'
Copy 'risabot_signs_640x640_nv12.bin' to your RDK X5 board at:
'/home/sunrise/risabot_signs_640x640_nv12.bin'
```

### 4.5 If hb_mapper Fails — The ONNX Patch Script

Some ONNX exports (even with `--opset 11`) produce node configurations that `hb_mapper` cannot handle. The most common errors are:

- `Unsupported attribute 'antialias' in node Resize`
- `Unsupported attribute 'keep_aspect_ratio_policy' in node Resize`
- `Reshape node has unsupported attribute 'allowzero'`
- `Split node has 2 inputs but expects 1`

The script `tools/bpu_model/patch_onnx_resize.py` fixes all of these. To run it:

1. Copy `best.onnx` into `tools/bpu_model/` (if it isn't there already)
2. Open PowerShell in that folder
3. Install the `onnx` library if you haven't already:
   ```powershell
   pip install onnx
   ```
4. Run the patch:
   ```powershell
   python patch_onnx_resize.py
   ```
5. The script modifies `best.onnx` in place

**What the patch does:**

| Problem | Fix Applied |
|---------|-------------|
| ONNX IR version > 9 | Downgrades `model.ir_version` to 9 |
| Opset > 11 | Sets all opset domains to version 11 |
| Resize node missing `roi` input | Inserts a dummy empty tensor as the `roi` input |
| Resize node has unsupported attributes | Removes attributes not in the opset 11 Resize spec |
| Reshape node has `allowzero` attribute | Removes `allowzero` (opset 11 Reshape doesn't support it) |
| Split node has 2 inputs (opset 13 style) | Converts second input to `split` attribute (opset 11 style) |

**After patching:** Re-run `compile_model.bat`.

### 4.6 Output Files Explained

After successful compilation, `model_output/` contains:

| File | What It Is |
|------|-----------|
| `risabot_signs_640x640_nv12.bin` | ✅ **The final BPU model.** This is the only file needed on the robot. |
| `*_original_float_model.onnx` | The ONNX model as parsed by `hb_mapper` (FP32) |
| `*_optimized_float_model.onnx` | After graph optimization (layer fusion, dead node removal) |
| `*_calibrated_model.onnx` | After INT8 calibration ranges are applied to each layer |
| `*_quantized_model.onnx` | The fully 8-bit quantized model |
| `*_quant_info.json` | Per-layer quantization info — useful if you suspect accuracy loss |
| `main_graph_subgraph_0.html` | Visual graph of the compiled subgraph (open in browser to inspect) |

---

## 5. Deployment to RDK X5

### 5.1 Copy the Model to the Robot

From your Windows PC (PowerShell or CMD), run:

```powershell
scp tools\bpu_model\model_output\risabot_signs_640x640_nv12.bin sunrise@<ROBOT_IP>:/home/sunrise/
```

Replace `<ROBOT_IP>` with the robot's IP address (e.g., `192.168.1.105`).

> The model **must** be at `/home/sunrise/risabot_signs_640x640_nv12.bin`. This path is hardcoded in the `signage_detector` node. If you change the filename, you must also update `src/risabot_automode/risabot_automode/signage_detector.py` and rebuild.

After copying, SSH into the robot to verify the file is there:
```bash
ssh sunrise@<ROBOT_IP>
ls -lh /home/sunrise/risabot_signs_640x640_nv12.bin
```
Expected: a file ~7.5 MB in size.

### 5.2 Static Verification

Run the static verification script to confirm the BPU hardware can load the model:

```bash
# On the robot (via SSH)
cd ~/risabotcar_ws
python3 tools/bpu_model/verify_bpu.py
```

**Expected output:**

```
=== BPU Model Verification Script ===
Successfully imported pyeasy_dnn from hobot_dnn_rdkx5
Attempting to load BPU model from: /home/sunrise/risabot_signs_640x640_nv12.bin
SUCCESS: BPU model loaded successfully.

--- Model Inputs ---
  Input[0]: name='images', shape=[1, 3, 640, 640], layout=NCHW, type=IMG_TYPE_NV12

--- Model Outputs ---
  Output[0]: name='output', shape=[1, 25200, 11], ...

Created dummy NV12 input tensor of shape: (960, 640)
Running forward inference pass on BPU...
SUCCESS: Forward inference completed.
Output raw prediction buffer shape: (1, 25200, 11)

=== Verification Successful! BPU model is fully functional. ===
```

> **If you get `import pyeasy_dnn failed`:** This script must run **on the robot** (via SSH), not on your local PC. The BPU libraries are only installed on the RDK X5 OS image.

> **If you get `Failed to load BPU model`:** The `.bin` file path is wrong or the file is corrupted. Check that it is at `/home/sunrise/risabot_signs_640x640_nv12.bin` and re-copy if needed.

### 5.3 Live Camera Verification

Run this test to confirm the model produces real detections using the live camera feed:

**Terminal 1 — start the camera:**
```bash
ros2 launch astra_camera astra_mini.launch.py
```

**Terminal 2 — run the live test:**
```bash
cd ~/risabotcar_ws
python3 tools/bpu_model/verify_live.py
```

Hold a sign (e.g., the parking signboard) in front of the camera within ~60 cm.

**Expected output:**
```
=== Live BPU Model Diagnostic Script ===
Successfully imported pyeasy_dnn.
Successfully loaded BPU model.
Subscribed to /camera/color/image_raw. Waiting for a frame...
Received a live frame (640x480). Running BPU inference...

--- Diagnostic Results ---
Maximum absolute score in the frame: 0.842341

  Anchors with score >= 0.01: 312
  Anchors with score >= 0.05: 87
  Anchors with score >= 0.10: 34
  Anchors with score >= 0.20: 12
  Anchors with score >= 0.30: 5
  Anchors with score >= 0.40: 2

--- Top 5 Highest Scoring Anchors ---
Rank 1: Anchor #12482 | Score=0.8423
  Box   : [x=318.4, y=241.2, w=145.6, h=198.3]
  Obj   : 0.9102
  Classes probabilities: [0.0012, 0.9254, 0.0002, 0.0001, 0.0001, 0.0001]
  Best Class ID: 1
```

A `Best Class ID: 1` with a score > 0.4 confirms **class 1 (`parking_sign`) was detected** — the model is working correctly.

> **If the max score is < 0.05 even with a sign directly in front of the camera:** The model may have trained on the wrong dataset version, or the class indices in `data.yaml` may not match the annotations. Retrain with a verified dataset.

### 5.4 If You Added a New Class — Update the ROS2 Node

After retraining with a new class, you must update the signage detector node to handle the new class ID.

1. Open `src/risabot_automode/risabot_automode/signage_detector.py`
2. Find the section that maps class IDs to publishers and topics
3. Add your new class:
   ```python
   elif class_id == 2:   # speedbump_sign (new class at index 2)
       self.speedbump_pub.publish(Bool(data=True))
   ```
4. Add a publisher for the new topic in `__init__`:
   ```python
   self.speedbump_pub = self.create_publisher(Bool, '/speedbump_detected', 10)
   ```
5. Rebuild the workspace on the robot:
   ```bash
   cd ~/risabotcar_ws
   colcon build --packages-select risabot_automode
   source install/setup.bash
   ```

---

## 6. Quick Reference

### 6.1 Troubleshooting Table

| Symptom | Cause | Fix |
|---------|-------|-----|
| Colab crashes immediately on Cell 5 with `UnpicklingError` or `weights_only` error | PyTorch 2.6+ changed `torch.load()` default | Run Cell 4.5 (PyTorch patch) before Cell 5 |
| Cell 5 crashes with `No labels found in .../valid/labels` | Roboflow gave only 1 image to validation | Run Cell 3 (re-split script) before Cell 5 |
| `hb_mapper` fails with `unsupported attribute` in Resize/Reshape node | ONNX export produced opset 13 style nodes despite `--opset 11` | Run `patch_onnx_resize.py` on `best.onnx`, then recompile |
| `hb_mapper` fails with `reshape error` or `calibration data shape mismatch` | Calibration files are wrong format (`.npy`, `.jpg`, wrong dtype) | Verify each `.bin` file is exactly 1,228,800 bytes. Re-run Cell 7 if needed. |
| `hb_mapper` fails with `Opset N is not supported` | Wrong opset version on ONNX export | Re-run Cell 6 with `--opset 11` explicitly |
| `verify_bpu.py` — `Failed to import BPU library` | Script run on local PC instead of robot | SSH into the robot first, then run the script |
| `verify_bpu.py` — `Failed to load BPU model` | Wrong file path or corrupted file | Check file exists at `/home/sunrise/risabot_signs_640x640_nv12.bin`. Re-SCP if needed. |
| `verify_live.py` hangs on `Waiting for a frame` | Camera driver not running | Open Terminal 1 and run `ros2 launch astra_camera astra_mini.launch.py` first |
| `verify_live.py` shows max score < 0.05 with sign in front of camera | Wrong dataset version, wrong class indices, or model trained on the wrong data | Check `data.yaml` class order matches Roboflow export order. Retrain if needed. |
| mAP50 stuck below 0.5 after 60 epochs | Too few images, incorrect annotations, or class imbalance | Add more images. Review annotations for tight bounding boxes. Check for label index mismatches. |
| mAP50 drops sharply on live robot even though Colab mAP was high | Domain mismatch — training images don't match robot camera conditions | Re-collect images using the robot's actual camera in the competition environment |

### 6.2 All Commands at a Glance

**Local PC (PowerShell):**
```powershell
# Pull Docker image (one-time)
docker pull openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8

# Copy compiled model to robot
scp tools\bpu_model\model_output\risabot_signs_640x640_nv12.bin sunrise@<ROBOT_IP>:/home/sunrise/

# Run ONNX patch (if hb_mapper fails)
cd tools\bpu_model
pip install onnx
python patch_onnx_resize.py
```

**Robot (via SSH):**
```bash
# Static BPU verification
python3 ~/risabotcar_ws/tools/bpu_model/verify_bpu.py

# Live camera verification (Terminal 1)
ros2 launch astra_camera astra_mini.launch.py

# Live camera verification (Terminal 2)
python3 ~/risabotcar_ws/tools/bpu_model/verify_live.py

# Rebuild after code changes
cd ~/risabotcar_ws
colcon build --packages-select risabot_automode
source install/setup.bash

# Adjust detection threshold at runtime (no rebuild needed)
ros2 param set /signage_detector conf_threshold 0.35
ros2 param set /signage_detector min_parking_sign_width 80
```

### 6.3 Files in tools/bpu_model/ Explained

| File / Folder | What It Is | When You Use It |
|---------------|-----------|-----------------|
| `colab_training_script.py` | All 8 Colab training cells in one file | Copy cells into Colab notebook |
| `risabot_bpu_config.yaml` | `hb_mapper` compiler configuration | Already configured — no changes needed |
| `compile_model.bat` | Windows batch script that runs Docker and `hb_mapper` | Double-click to compile after Colab training |
| `patch_onnx_resize.py` | Fixes incompatible ONNX node attributes | Run only if `hb_mapper` fails with attribute errors |
| `verify_bpu.py` | Loads the `.bin` model and runs a dummy inference pass | Run on robot to verify model loaded correctly |
| `verify_live.py` | Runs live BPU inference using the camera feed | Run on robot to verify real detections |
| `best.onnx` | The trained ONNX model (downloaded from Colab) | Input to `compile_model.bat` |
| `calibration_data/` | 50 raw binary calibration images | Input to `compile_model.bat` |
| `model_output/` | All compiler output files | Source of the `.bin` to SCP to the robot |
| `hb_mapper_makertbin.log` | Detailed compiler log from the last run | Check here if compilation fails for detailed errors |

---

*Guide written for RISAbot `risabotcar_ws` — based on the [Cytron RDK X5 BPU Tutorial](https://my.cytron.io/tutorial/malaysia-car-number-plate-detection-on-the-rdk-x5-bpu).*
