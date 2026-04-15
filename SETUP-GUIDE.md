# Setup Guide - ASR-D501 YOLO Drone Demo

Step-by-step instructions for running the CPU and NPU object detection demo.

---

## 1. Hardware Setup

### 1.1 Assemble the Drone

If starting from scratch, follow the F450 assembly guide:
https://www.hawks-work.com/pages/f450-drone

### 1.2 Connect Everything

1. **Plug the battery** into the drone.
2. **Connect the USB cable** from the Pixhawk flight controller to the ASR-D501 companion computer.
3. **Connect the camera** to the ASR-D501 via USB:
   - AnkerWork C310 → will appear as `/dev/video2`
   - Intel RealSense D435i → RGB stream on `/dev/video6`
4. **Connect a monitor** to the ASR-D501 Mini DisplayPort (for bounding box display).

### 1.3 Safety Switch

Hold the **RED SAFETY SWITCH** on the Pixhawk for 3 seconds until it turns **SOLID RED**.

> **IMPORTANT:** The flight controller re-engages the safety switch after some time. When you hear the motors start **BEEPING**, that means the safety switch turned itself back on. To fix it:
>
> 1. Hold the safety switch for ~2 seconds until it starts **FLASHING**
> 2. Release, press again and keep holding until it goes **SOLID RED** - the motors will make a synchronized **BEEP** sound
> 3. Now you're good to go

---

## 2. Software Prerequisites

Make sure the following are installed on the ASR-D501. If not, follow the [ROS 2 and MAVROS installation guide](docs/ROS2-and-MAVROS-install.md).

- Ubuntu 24.04 LTS
- ROS 2 Jazzy (`ros-jazzy-ros-base`)
- MAVROS (`ros-jazzy-mavros`, `ros-jazzy-mavros-extras`)
- GeographicLib datasets (MAVROS crashes without these)
- User added to `dialout` group (for serial port access)
- Python 3.12 with `numpy` and `opencv-python` (for CPU version also `ultralytics`)

Verify:
```bash
ros2 --version
ros2 pkg list | grep mavros
groups | grep dialout
```

### NPU Prerequisites (for NPU version only)

The quantized model and labels must be in place:
```bash
ls ~/ai-hub/EdgeAI_Workflow/ai_system/qualcomm/aom-dk2721/linux/script/
# Should contain: yolov8_det.tflite, labels.txt
```

If the model is missing, follow the [Advantech quantization guide](https://github.com/ADVANTECH-Corp/EdgeAI_Workflow/blob/main/ai_system/qualcomm/aom-dk2721/linux/object_detection_demo-using-qc_ai_hub.md).

---

## 3. Running the Demo

Open **two terminals** on the ASR-D501.

### Terminal 1 - Start MAVROS

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

Wait until you see **"GF: Mission received"** in the output before continuing.

If the Pixhawk is on a different port, check:
```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

### Terminal 2 - Start the Demo Script

Navigate to the scripts directory:
```bash
cd scripts/
```

Pick the version matching your camera and inference mode:

#### AnkerWork C310 Camera

```bash
# CPU version (slow, for comparison):
python3 ankerCAM_cpu_demo.py

# NPU version (fast, real-time):
python3 ankerCAM_npu_demo.py
```

#### Intel RealSense D435i Camera

```bash
# CPU version (slow, for comparison):
python3 intelCAM_cpu_demo.py

# NPU version (fast, real-time):
python3 intelCAM_npu_demo.py
```

#### NPU Test Mode (no motors, detection only)

If you want to test NPU detection without arming the motors:
```bash
python3 ankerCAM_npu_demo.py --test
python3 intelCAM_npu_demo.py --test
```

#### NPU Headless Mode (no display, SSH)

If running over SSH without a monitor:
```bash
python3 ankerCAM_npu_demo.py --no-display
python3 intelCAM_npu_demo.py --no-display
```

---

## 4. What the Script Does

After launch, the script will:
1. Connect to the flight controller via MAVROS
2. Set STABILIZE flight mode
3. Arm the motors (you'll hear them start spinning at idle)
4. Begin detection - **point the camera at a clock**

### Detection Behavior

- **Clock detected** → motors speed up (1750 us throttle)
- **Clock removed** → motors drop to idle (1150 us throttle)
- **Clock moves left** → yaw left (1150 us)
- **Clock moves right** → yaw right (1850 us)
- **Clock centered** → yaw neutral (1500 us)
- **Person, book, or other objects** → ignored (only class 74 "clock" triggers)

### Terminal Output (NPU version)

```
Pipeline PLAYING - NPU inference active
Target: clock (class 74)  conf>0.55
Deadzone: left<243px  right>396px
PWM: idle=1150  active=1750  yawL=1150  yawR=1850
CLOCK DETECTED (conf=95%) -> SPOOL UP (1750 us)!
CLOCK at cx=320/640  === CENTERED ===  thr=1750 yaw=1500
CLOCK at cx=120/640  <<< YAW LEFT  thr=1750 yaw=1150
CLOCK LOST -> IDLE (1150 us)
```

### Terminal Output (CPU version)

The CPU version also shows a live OpenCV window with:
- Bounding box around detected clock
- Direction status (YAW LEFT / CENTERED / YAW RIGHT)
- Pixel position and confidence percentage
- PWM values being sent

---

## 5. Stopping the Demo

**CPU version:**
- Press **Q** on the OpenCV window, or press **Ctrl+C** in the terminal

**NPU version:**
- Press **Ctrl+C** in the terminal (motors disarm automatically)

**Then:**
- Press **Ctrl+C** in Terminal 1 (MAVROS)
- Disconnect the battery

---

## 6. GStreamer Shell Scripts (NPU display only, no motor control)

If you just want to see the NPU detection running with bounding boxes on screen (no motor control, no ROS 2 needed):

```bash
cd scripts/

# AnkerWork C310:
bash anker_MJPEG_yolo_cam_ai_hub.sh

# Intel RealSense D435i:
bash intel_YUY2_yolo_cam_ai_hub.sh
```

These run the full GStreamer pipeline with the NPU and display bounding boxes on the monitor. Useful for testing that the camera and model work before connecting the drone.

---

## 7. Troubleshooting

| Problem | Solution |
|---------|----------|
| **"FCU not connected after 15s"** | Check USB cable between Pixhawk and ASR-D501. Make sure Terminal 1 (MAVROS) is running and shows heartbeat messages. |
| **"ARM REJECTED"** | Safety switch is not solid red - hold it again. Check Mission Planner for pre-arm errors. |
| **Motors beeping** | Safety switch re-engaged. Hold it until solid red again. |
| **No bounding boxes on NPU display** | Make sure you are in the correct directory with `yolov8_det.tflite` and `labels.txt`. |
| **"Permission denied" on /dev/ttyACM0** | Add user to dialout group: `sudo usermod -a -G dialout $USER` then log out and back in. |
| **Camera not found** | Run `v4l2-ctl --list-devices` to check device paths. The RealSense has multiple devices - look for the YUYV one. |
| **NPU script detects but no display** | You might be running with `--no-display`. Run without the flag and make sure a monitor is connected. |
| **Pipeline stuck in PAUSED** | Wrong camera device or format. Check the camera is plugged in and verify the device path with `v4l2-ctl`. |
| **False detections on blank wall** | The confidence threshold is set to 0.55 to prevent this. If it still happens, raise `CONF_THRESHOLD` in the script. |

---

## 8. Flight Controller Parameters

These are pre-configured via Mission Planner. Do not change them unless you know what you are doing.

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `RC_OVERRIDE_TIME` | 3 | Seconds before RC override expires |
| `ARMING_CHECK` | 0 | Disable pre-arm safety checks |
| `FS_THR_ENABLE` | 0 | Throttle failsafe disabled |
| `SYSID_MYGCS` | 1 | Must match MAVROS `system_id` (verify: `ros2 param get /mavros system_id`) |
| `BRD_SAFETY_MASK` | 0 | Bypass safety switch requirement |
| `MOT_SPIN_ARM` | 0.15 | Motor spin when armed (15%) |
| `MOT_SPIN_MIN` | 0.15 | Minimum motor spin in flight (15%) |

---

## 9. Performance Reference

| | CPU | NPU |
|---|---|---|
| FPS | ~3-5 | ~30+ |
| Power draw | ~8.5W | ~7.6W |
| Latency | ~200-300ms | ~33ms |
| Model | YOLOv8n FP32 (12 MB) | YOLOv8n INT8 (3.3 MB) |

Idle board power: 5.8W | Board max: 11W