# ASR-D501 YOLO Drone Demo

**Real-time object detection controlling drone motors using CPU and NPU inference on the Qualcomm QCS6490.**

Built for Japan IT Week 2026 at the Advantech booth to demonstrate the difference between running YOLOv8 on a general-purpose CPU versus the integrated Hexagon NPU.

---

## What It Does

A camera watches the scene. When the YOLOv8 model detects a **clock**, the drone motors speed up. Move the clock left or right and the drone yaws to follow. Remove the clock and the motors drop back to idle.

Two versions of the same demo:
- **CPU version** - YOLOv8 runs on the ARM Cortex-A78 cores (~3-5 FPS, noticeable lag)
- **NPU version** - YOLOv8 runs on the Hexagon HTP NPU (~30+ FPS, real-time response)

Same model, same camera, same drone. The only difference is where the inference runs.

---

## Hardware

| Component | Details |
|-----------|---------|
| **Companion Computer** | [Advantech ASR-D501](https://www.advantech.com/) - Qualcomm QCS6490 SoC, 8GB LPDDR5, 128GB UFS, Ubuntu 24.04 LTS |
| **Flight Controller** | Pixhawk 1 running ArduCopter V4.6.x |
| **Drone Frame** | [DJI F450 frame kit](https://www.hawks-work.com/pages/f450-drone) - assembly instructions at the link |
| **Camera (Option A)** | AnkerWork C310 - USB webcam, outputs MJPEG |
| **Camera (Option B)** | Intel RealSense D435i - USB depth camera, RGB stream outputs YUYV |

---

## Repository Structure

```
├── README.md                          # This file
├── SETUP-GUIDE.md                     # How to run the demo (start here)
├── docs/
│   ├── ROS2-and-MAVROS-install.md     # Installing ROS 2 Jazzy + MAVROS on Ubuntu 24.04
│   ├── NPU-deployment-guide.md        # Adapting the GStreamer pipeline from Yocto to Ubuntu
│   └── RTABMAP-mapping.md             # 3D mapping with Intel RealSense (optional)
├── scripts/
│   ├── ankerCAM_cpu_demo.py           # CPU inference - AnkerWork C310
│   ├── ankerCAM_npu_demo.py           # NPU inference - AnkerWork C310
│   ├── intelCAM_cpu_demo.py           # CPU inference - Intel RealSense D435i
│   ├── intelCAM_npu_demo.py           # NPU inference - Intel RealSense D435i
│   ├── anker_MJPEG_yolo_cam_ai_hub.sh # GStreamer shell script - AnkerWork (MJPEG)
│   ├── intel_YUY2_yolo_cam_ai_hub.sh  # GStreamer shell script - Intel RealSense (YUYV)
│   └── labels.txt                     # COCO class labels (80 classes)
└── model/
    └── yolov8_det.tflite              # Quantized YOLOv8n model for NPU (see below)
```

---

## Quick Start

> Full step-by-step instructions: **[SETUP-GUIDE.md](SETUP-GUIDE.md)**

```bash
# Terminal 1 - Start MAVROS
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200

# Terminal 2 - Run the demo
cd scripts/

# CPU version (AnkerWork camera):
python3 ankerCAM_cpu_demo.py

# NPU version (AnkerWork camera):
python3 ankerCAM_npu_demo.py
```

Point the camera at a clock. Motors react.

---

## Model Quantization

The NPU requires a quantized TFLite model. The original YOLOv8n PyTorch model (.pt) is converted through:

```
.pt (PyTorch) → ONNX → .tflite (INT8 quantized for Hexagon HTP)
```

Follow the Advantech quantization guide:
https://github.com/ADVANTECH-Corp/EdgeAI_Workflow/blob/main/ai_system/qualcomm/aom-dk2721/linux/object_detection_demo-using-qc_ai_hub.md

> **Note:** The shell scripts in that guide are written for Yocto Linux with a MIPI camera. If you are running Ubuntu with a USB camera (AnkerWork or RealSense), use the adapted shell scripts in `scripts/` instead. See [NPU Deployment Guide](docs/NPU-deployment-guide.md) for details on what changed and why.

---

## Measured Performance

| Metric | CPU | NPU |
|--------|-----|-----|
| Inference FPS | ~3-5 | ~30+ |
| Board power draw | ~8.5W | ~7.6W |
| Response latency | ~200-300ms (noticeable) | ~33ms (instant) |
| Model size | ~12 MB (FP32) | ~3.3 MB (INT8) |

Idle board power: 5.8W. Board maximum rated: 11W.

---

## Prerequisites

- Ubuntu 24.04 LTS on the ASR-D501
- ROS 2 Jazzy + MAVROS installed ([installation guide](docs/ROS2-and-MAVROS-install.md))
- Pixhawk flight controller with ArduCopter firmware
- Camera connected via USB

---

## Camera Support

| Camera | Format | Device | Decode Stage |
|--------|--------|--------|-------------|
| AnkerWork C310 | MJPEG | `/dev/video2` | jpegparse → jpegdec → videoconvert |
| Intel RealSense D435i | YUYV | `/dev/video6` (RGB) | videoconvert only |

Both cameras produce NV12 after conversion. The NPU inference path is identical from that point on.

To find the correct device for the RealSense:
```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/videoN --list-formats-ext  # look for YUYV
```

---

## Flight Controller Parameters

These must be set in Mission Planner before running the demo:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| RC_OVERRIDE_TIME | 3 | RC override timeout (seconds) |
| FS_THR_ENABLE | 0 | Disable throttle failsafe |
| SYSID_MYGCS | 1 | Must match MAVROS system_id |
| BRD_SAFETY_MASK | 0 | Bypass safety switch |
| MOT_SPIN_ARM | 0.10 | Motor spin when armed (10%) |
| MOT_SPIN_MIN | 0.15 | Minimum motor spin (15%) |

---

## Author

**Szymon Dudek** - Internship at Advantech Co., Ltd., Tokyo, Japan (2025-2026)

Supervisor: **Jack Tsao** 