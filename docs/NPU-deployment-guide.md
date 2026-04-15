# NPU Deployment Guide — Yocto to Ubuntu

Running YOLOv8 on the Qualcomm QCS6490 NPU with Ubuntu 24.04 LTS.

> The official Advantech/Qualcomm reference scripts are written for Yocto Linux. This document explains what needs to change to run the same pipeline on Ubuntu.

---

## 1. Quantizing the YOLOv8 Model

The Qualcomm HTP NPU requires models in TensorFlow Lite format with INT8 quantization. A standard YOLOv8 PyTorch model (.pt) cannot run on the NPU directly.

### Process Overview

1. Export the YOLOv8 PyTorch model (.pt) to ONNX format.
2. Upload the ONNX model to Qualcomm AI Hub for quantization and compilation.
3. AI Hub quantizes the model to INT8 and compiles it for the Hexagon HTP backend.
4. Download the resulting `.tflite` file and deploy it to the board.

Full quantization guide: [Advantech EdgeAI Workflow](https://github.com/ADVANTECH-Corp/EdgeAI_Workflow/blob/main/ai_system/qualcomm/aom-dk2721/linux/object_detection_demo-using-qc_ai_hub.md)

> **Shortcut:** A pre-quantized `yolov8_det.tflite` and `labels.txt` are included in the `model/` directory of this repository. Copy them to the directory the scripts expect:
> ```bash
> cp model/yolov8_det.tflite model/labels.txt ~/ai-hub/EdgeAI_Workflow/ai_system/qualcomm/aom-dk2721/linux/script/
> ```

The model file is placed on the board alongside a `labels.txt` file containing the 80 COCO class names (included in `scripts/`).

---

## 2. How the AI Model Gets from Training to the NPU

An AI model goes through three format conversions before it can run on the Qualcomm NPU. Each step makes the model smaller, faster, and more suited to the target hardware.

### Step 1: PyTorch (.pt) — The Training Format

YOLOv8 is trained using PyTorch. The resulting `.pt` file contains millions of numerical values (weights) that represent what the model has learned. This format is designed for training and experimentation on powerful computers, not for running efficiently on small embedded boards. The file is large (~12 MB) and requires the full PyTorch library to run.

### Step 2: ONNX — The Universal Exchange Format

ONNX (Open Neural Network Exchange) is a standardized format that acts as a bridge between different AI tools. Qualcomm's tools do not understand PyTorch's format directly, but they can read ONNX. Exporting to ONNX does not change what the model does — it rewrites the same model in a format that hardware vendor compilers can work with.

### Step 3: TensorFlow Lite (.tflite) — The Deployment Format

Qualcomm AI Hub takes the ONNX model and does two things: it compresses the model through quantization (reducing each value from 32 bits down to 8 bits, making the model roughly 4x smaller), and it compiles the operations into instructions that the Hexagon NPU hardware can execute natively. The result is a compact `.tflite` file (~3.3 MB) that runs directly on the dedicated AI silicon.

Despite the 4x compression, the detection accuracy remains nearly identical. Neural networks are highly tolerant of this kind of precision reduction, and the quantization process is carefully calibrated to preserve accuracy.

### How It Runs on the Board

On the ASR-D501, the GStreamer media pipeline handles the entire flow automatically. Each camera frame is fed into the TFLite runtime, which passes it to the QNN delegate — a bridge that routes all computation to the Hexagon NPU hardware instead of the CPU. The NPU processes the frame in dedicated AI silicon, and the results (object class, position, confidence) are returned for the application to act on. The CPU is free to handle other tasks like motor control, communication, and display.

This is the key advantage over boards without an NPU, such as the Raspberry Pi 5. On a Raspberry Pi, the same YOLOv8 model must run entirely on the CPU. On the QCS6490, the 12.3 TOPS NPU handles the AI workload in parallel, delivering real-time detection at full camera frame rate while consuming less power.

---

## 3. Adapting the GStreamer Pipeline from Yocto to Ubuntu

The reference GStreamer pipeline ([yolov8_cam_ai_hub.sh](https://github.com/ADVANTECH-Corp/EdgeAI_Workflow/blob/main/ai_system/qualcomm/aom-dk2721/linux/script/yolov8_cam_ai_hub.sh)) provided by Advantech is written for the Yocto Linux image. Running on Ubuntu 24.04 requires several modifications. The core inference path (camera to NPU to bounding boxes) remains the same. The changes are all in the plumbing around it.

### Key Differences

| Area | Yocto (Reference Script) | Ubuntu (Modified Script) |
|------|--------------------------|--------------------------|
| **Wayland Display** | `XDG_RUNTIME_DIR=/dev/socket/weston` / `wayland-1` | `XDG_RUNTIME_DIR=/run/user/1000` / `wayland-0` |
| **SDK Setup** | `source /opt/qcom/qirp-sdk/qirp-setup.sh` (required) | Not needed. Libraries installed system-wide via apt. |
| **Camera Input** | Raw video from v4l2src. Camera outputs NV12 natively (MIPI or UVC raw). | Camera outputs MJPEG or YUYV. Requires decode/convert stage before NV12. |
| **Memory Model** | `video/x-raw(memory:GBM)` — GPU buffer memory. Zero-copy between camera and NPU. | `video/x-raw` (system memory). GBM not available through standard Ubuntu packages. |
| **Post-Processing** | `qtimlvdetection` with manual quantization constants (q-offsets, q-scales). | `qtimlpostprocess` with `module=yolov8`. Handles dequantization internally. No manual constants needed. |
| **Display Compositor** | `qtivcomposer` — Qualcomm hardware compositor with alpha blending. | `videomixer` — standard GStreamer element. Works on any platform. |
| **Queue Tuning** | Default queue settings. | `max-size-buffers` and `leaky=downstream` set explicitly to prevent pipeline stalls. |
| **Output Resolution** | Detection branch outputs 640x360 (16:9). Aspect ratio mismatch with 4:3 camera. | Everything stays at 640x480 consistently. No aspect ratio issues. |

---

## 4. Camera Format Handling

The biggest practical difference is how the camera frame reaches the NPU. The Yocto reference assumes the camera outputs raw uncompressed NV12 video. Most USB cameras do not.

### MJPEG Camera (AnkerWork C310)

MJPEG is compressed on the camera chip, so it needs to be decoded before the NPU can use it:

```
v4l2src device="/dev/video2"
! image/jpeg,width=640,height=480       # Request MJPEG from camera
! jpegparse                              # Parse JPEG stream
! jpegdec                                # Decode JPEG to raw pixels
! videoconvert                           # Convert color format
! video/x-raw,format=NV12,width=640,height=480   # NV12 for NPU
```

### YUYV Camera (Intel RealSense D435i)

The RealSense outputs raw uncompressed YUYV. No decoding needed — just a color format conversion:

```
v4l2src device="/dev/video6"
! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1   # Raw YUYV
! videoconvert                           # Convert color format
! video/x-raw,format=NV12,width=640,height=480   # NV12 for NPU
```

### NV12 Camera (MIPI CSI, e.g. Raspberry Pi Camera Module)

Cameras that output NV12 natively need no conversion at all. This is the case the Yocto reference script was designed for.

From the NV12 stage forward, the rest of the pipeline (`qtivtransform`, `qtimlvconverter`, `qtimltflite`) is identical regardless of which camera is used.

### Camera Comparison

| | AnkerWork C310 | Intel RealSense D435i |
|---|---|---|
| **Device** | `/dev/video2` | `/dev/video6` (RGB stream) |
| **Output format** | MJPEG (compressed) | YUYV (uncompressed) |
| **Input caps** | `image/jpeg,width=640,height=480` | `video/x-raw,format=YUY2,width=640,height=480,framerate=30/1` |
| **Decode stage** | `jpegparse` + `jpegdec` + `videoconvert` | `videoconvert` only |

### Finding the RealSense RGB Device

The RealSense D435i registers six `/dev/video` devices for its different sensors (depth, infrared, RGB). The device number may vary. To find the correct RGB device:

```bash
v4l2-ctl --list-devices

# Then check which device outputs YUYV:
v4l2-ctl -d /dev/videoN --list-formats-ext

# Depth stream shows 'Z16', IR shows 'GREY', RGB shows 'YUYV'.
```

---

## 5. Reference Scripts

### Original Yocto Script (from Advantech GitHub)

```bash
export XDG_RUNTIME_DIR=/dev/socket/weston
export WAYLAND_DISPLAY=wayland-1
source /opt/qcom/qirp-sdk/qirp-setup.sh

gst-launch-1.0 -e v4l2src device="/dev/video2" ! queue ! tee name=split \
  split. ! queue ! qtivcomposer name=mixer \
    sink_1::dimensions="<640,480>" sink_1::alpha=0.5 \
    ! queue ! waylandsink fullscreen=false width=1280 height=720 \
  split. ! queue ! qtivtransform \
    ! video/x-raw(memory:GBM),format=NV12,width=640,height=480 \
    ! qtimlvconverter ! queue ! qtimltflite delegate=external \
    external-delegate-path=libQnnTFLiteDelegate.so \
    external-delegate-options="QNNExternalDelegate,backend_type=htp;" \
    model=yolov8_det_quantized.tflite ! queue \
    ! qtimlvdetection threshold=10.0 results=10 module=yolov8 \
    labels=coco_labels.txt \
    constants="YOLOv8,q-offsets=<20.0,0.0,0.0>,q-scales=<3.14,0.004,1.0>;" \
    ! video/x-raw,format=BGRA,width=640,height=360 ! queue ! mixer.
```

### Modified Ubuntu Script — AnkerWork C310 (MJPEG)

```bash
#!/bin/bash
export XDG_RUNTIME_DIR=/run/user/1000
export WAYLAND_DISPLAY=wayland-0

gst-launch-1.0 -e \
  v4l2src device="/dev/video2" \
  ! image/jpeg,width=640,height=480 \
  ! jpegparse ! jpegdec ! videoconvert \
  ! "video/x-raw,format=NV12,width=640,height=480" \
  ! qtivtransform \
  ! "video/x-raw,format=NV12,width=640,height=480" \
  ! tee name=split \
  split. ! queue max-size-buffers=20 leaky=no \
    ! videoconvert \
    ! "video/x-raw,format=BGRA,width=640,height=480" \
    ! videomixer name=mix \
    ! queue ! waylandsink fullscreen=false sync=false \
  split. ! queue max-size-buffers=2 leaky=downstream \
    ! qtimlvconverter \
    ! qtimltflite delegate=external \
      external-delegate-path=libQnnTFLiteDelegate.so \
      external-delegate-options="QNNExternalDelegate,backend_type=htp;" \
      model=yolov8_det.tflite \
    ! qtimlpostprocess \
      module=yolov8 \
      labels=labels.txt \
      results=10 \
      bbox-stabilization=true \
    ! "video/x-raw,format=BGRA,width=640,height=480" \
    ! queue max-size-buffers=2 leaky=downstream \
    ! mix.
```

### Modified Ubuntu Script — Intel RealSense D435i (YUYV)

```bash
#!/bin/bash
# RGB stream is on /dev/video6, outputs YUYV at 640x480@30fps
export XDG_RUNTIME_DIR=/run/user/1000
export WAYLAND_DISPLAY=wayland-0

gst-launch-1.0 -e \
  v4l2src device="/dev/video6" \
  ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 \
  ! videoconvert \
  ! "video/x-raw,format=NV12,width=640,height=480" \
  ! qtivtransform \
  ! "video/x-raw,format=NV12,width=640,height=480" \
  ! tee name=split \
  split. ! queue max-size-buffers=20 leaky=no \
    ! videoconvert \
    ! "video/x-raw,format=BGRA,width=640,height=480" \
    ! videomixer name=mix \
    ! queue ! waylandsink fullscreen=false sync=false \
  split. ! queue max-size-buffers=2 leaky=downstream \
    ! qtimlvconverter \
    ! qtimltflite delegate=external \
      external-delegate-path=libQnnTFLiteDelegate.so \
      external-delegate-options="QNNExternalDelegate,backend_type=htp;" \
      model=yolov8_det.tflite \
    ! qtimlpostprocess \
      module=yolov8 \
      labels=labels.txt \
      results=10 \
      bbox-stabilization=true \
    ! "video/x-raw,format=BGRA,width=640,height=480" \
    ! queue max-size-buffers=2 leaky=downstream \
    ! mix.
```

---

## 6. What Stays the Same

The core inference path is identical between Yocto and Ubuntu. The differences are only in the surrounding I/O plumbing.

- Same TFLite model file (`yolov8_det.tflite`)
- Same QNN HTP delegate (`libQnnTFLiteDelegate.so`)
- Same NPU hardware (Hexagon 770, 12.3 TOPS)
- Same GStreamer inference elements (`qtimlvconverter`, `qtimltflite`)
- Same real-time inference performance at full camera frame rate

---

## 7. Summary

Customers deploying on Ubuntu instead of Yocto need to make three categories of changes:

1. **Wayland environment variables** must be updated to match Ubuntu's display server paths.
2. **Camera input handling** must be adapted for the camera's output format. USB webcams typically output MJPEG and need a decode stage. Cameras outputting raw YUYV (such as the Intel RealSense D435i) only need a color format conversion. MIPI cameras outputting raw NV12 do not need any conversion.
3. **Display and compositing elements** should use standard GStreamer elements (`videomixer`, `waylandsink`) instead of Qualcomm-specific elements (`qtivcomposer`) that may not be fully supported on Ubuntu.

The NPU inference itself requires zero changes. The same model, same delegate, and same performance carry over directly from Yocto to Ubuntu.
