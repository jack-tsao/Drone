# Setup Guide - ASR-D501 YOLO Drone Demo

Step-by-step instructions for running the CPU and NPU object detection demo.

> **What's new:** The `ankerCAM_npu_demo.py` script has been substantially
> upgraded. It no longer drives the motors through RC override in STABILIZE
> mode; instead it uses the flight controller's **motor-test** command for
> direct, reliable motor-speed (pitch) control, and adds distance-scaled
> pitch, a lock-on countdown, voice announcements, an on-screen HUD, idle
> search sweep, multi-class tracking, session logging, and a live telemetry
> web page. See **Section 10 – NPU Script Features** for full details.

---

## 1. Hardware Setup

### 1.1 Assemble the Drone

If starting from scratch, follow the F450 assembly guide:
https://www.hawks-work.com/pages/f450-drone

### 1.2 Connect Everything

1. **Plug the battery** into the drone.
2. **Connect the USB cable** from the Pixhawk / MiniPix flight controller to the ASR-D501 companion computer.
3. **Connect the camera** to the ASR-D501 via USB:
   - AnkerWork C310 → will appear as `/dev/video2`
   - Intel RealSense D435i → RGB stream on `/dev/video6`
4. **Connect a monitor** to the ASR-D501 Mini DisplayPort (for bounding box + HUD display).
5. **(Optional) Connect a speaker** to the ASR-D501 (3.5mm jack, USB audio, or HDMI audio) if you want the new **voice announcements**. Test it with `speaker-test -t sine -f 440 -l 1`.

### 1.3 Safety Switch

Hold the **RED SAFETY SWITCH** on the flight controller for 3 seconds until it turns **SOLID RED**.

> **IMPORTANT:** The flight controller re-engages the safety switch after some time. When you hear the motors start **BEEPING**, that means the safety switch turned itself back on. To fix it:
>
> 1. Hold the safety switch for ~2 seconds until it starts **FLASHING**
> 2. Release, press again and keep holding until it goes **SOLID RED** - the motors will make a synchronized **BEEP** sound
> 3. Now you're good to go

> **NOTE (NPU script):** Motor test runs while **disarmed**, so the new NPU
> script does not arm the aircraft. The safety switch and `BRD_SAFETY_MASK`
> parameter still govern whether the board will spin motors at all — keep the
> switch solid red.

---

## 2. Software Prerequisites

Make sure the following are installed on the ASR-D501. If not, follow the [ROS 2 and MAVROS installation guide](docs/ROS2-and-MAVROS-install.md).

- Ubuntu 24.04 LTS
- ROS 2 Jazzy (`ros-jazzy-ros-base`)
- MAVROS (`ros-jazzy-mavros`, `ros-jazzy-mavros-extras`)
- GeographicLib datasets (MAVROS crashes without these)
- User added to `dialout` group (for serial port access)
- Python 3.12 with `numpy` and `opencv-python` (for CPU version also `ultralytics`)
- **`espeak-ng`** (optional, for NPU voice announcements): `sudo apt install espeak-ng`
- **`matplotlib`** (optional, for the session-log plotting script): `pip install matplotlib --break-system-packages`

Verify:
```bash
ros2 --version
ros2 pkg list | grep mavros
groups | grep dialout
espeak-ng "test"   # optional: should speak if audio is set up
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

If the flight controller is on a different port, check:
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

#### NPU Command-Line Flags

The NPU script accepts these flags (combine as needed):

```bash
python3 ankerCAM_npu_demo.py --test         # No motor commands (detection only)
python3 ankerCAM_npu_demo.py --no-display    # Headless / SSH (no monitor, no HUD)
python3 ankerCAM_npu_demo.py --web           # Enable live telemetry web page (port 8088)
```

---

## 4. What the Script Does

> **NPU vs CPU — important difference.** The **CPU** script still uses RC
> override in STABILIZE mode (arms the aircraft, sends throttle/yaw on RC
> channels). The **NPU** script has been rewritten to use the flight
> controller's **motor-test** command, which spins the motors at a direct
> throttle percentage. It does **not** set a flight mode and does **not** arm.
> This is why the NPU pitch behavior is smooth and reliable while the motors
> stay disarmed.

### NPU script flow

After launch, the NPU script will:
1. Connect to the flight controller via MAVROS
2. Wait for the motor-test command service
3. Start the motors at a low idle / search-sweep pulse
4. Begin detection - **point the camera at the target (default: clock)**

### Detection Behavior (NPU script)

- **Target detected** → enters a **lock-on countdown** (default 3s), pitch
  rises during the count, then locks on at full pitch
- **Target close (large in frame)** → higher pitch; **far (small)** → lower
  pitch (distance-scaled)
- **Target removed** → motors drop to an idle **search sweep** (gentle pulsing)
- **Target left of frame** → "YAW LEFT" (HUD + voice)
- **Target right of frame** → "YAW RIGHT"
- **Target centered** → "CENTERED"
- **Other classes** → ignored unless selected (see multi-class tracking)

### Detection Behavior (CPU script — unchanged)

- **Clock detected** → motors speed up (1750 us throttle)
- **Clock removed** → motors drop to idle (1150 us throttle)
- **Clock moves left** → yaw left (1150 us)
- **Clock moves right** → yaw right (1850 us)
- **Clock centered** → yaw neutral (1500 us)

### Terminal Output (NPU version)

```
Pipeline PLAYING - NPU inference active
Target: clock (class 74)  conf>0.55
Motor pitch: SCALED 25%-85% by clock size (far->near)  idle=1%  motors=4
Yaw zones: LEFT<243px  CENTER  RIGHT>396px  (frame 640px)
Multi-class: tracking 'clock'. Cycle with:  kill -USR1 <pid>
Voice announcements on (espeak-ng)
CLOCK DETECTED (95%) -> ACQUIRING
CLOCK LOCKED
clock at cx=120/640  YAW LEFT
clock at cx=320/640  CENTERED
CLOCK LOST -> IDLE
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
- Press **Ctrl+C** in the terminal (motors stop automatically; session log is saved)

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
| **"FCU not connected after 15s"** | Check USB cable between flight controller and ASR-D501. Make sure Terminal 1 (MAVROS) is running and shows heartbeat messages. |
| **"ARM REJECTED" (CPU script)** | Safety switch is not solid red - hold it again. Check Mission Planner for pre-arm errors. |
| **"PreArm: Motors: MOT_SPIN_ARM > MOT_SPIN_MIN"** | `MOT_SPIN_ARM` must be ≤ `MOT_SPIN_MIN`. Set `MOT_SPIN_ARM 0.10` and `MOT_SPIN_MIN 0.15` (see Section 8). |
| **NPU motors don't spin** | Motor test requires the vehicle **disarmed** and the safety switch off/solid. Confirm `BRD_SAFETY_MASK` and check only one motor spins → see next row. |
| **Only one motor spins (NPU)** | Motor test runs motors one at a time; the script sends a separate command per motor. Confirm `NUM_MOTORS` matches your airframe (default 4). |
| **Motors beeping** | Safety switch re-engaged. Hold it until solid red again. |
| **No bounding boxes on NPU display** | Make sure you are in the correct directory with `yolov8_det.tflite` and `labels.txt`. |
| **Pango-WARNING / HUD not updating** | Fixed: direction strings no longer contain `< >` characters and HUD text is sanitized. Update to the latest script if you still see this. |
| **No voice / "espeak not found"** | Install with `sudo apt install espeak-ng` and connect a speaker. Voice degrades silently if missing — the rest of the demo still works. |
| **"Permission denied" on /dev/ttyACM0** | Add user to dialout group: `sudo usermod -a -G dialout $USER` then log out and back in. |
| **Camera not found** | Run `v4l2-ctl --list-devices` to check device paths. The RealSense has multiple devices - look for the YUYV one. |
| **NPU script detects but no display** | You might be running with `--no-display`. Run without the flag and make sure a monitor is connected. |
| **Pipeline stuck in PAUSED** | Wrong camera device or format. Check the camera is plugged in and verify the device path with `v4l2-ctl`. |
| **Web page won't load (`--web`)** | Confirm the device IP (`hostname -I`), use `http://<ip>:8088`, and check the port isn't blocked by the network/firewall. |
| **False detections on blank wall** | The confidence threshold is set to 0.55 to prevent this. If it still happens, raise `CONF_THRESHOLD` in the script. |

---

## 8. Flight Controller Parameters

These are pre-configured via Mission Planner. Do not change them unless you know what you are doing.

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `RC_OVERRIDE_TIME` | 3 | Seconds before RC override expires (used by CPU script) |
| `ARMING_CHECK` | 0 | Disable pre-arm safety checks (needed by CPU script to arm) |
| `FS_THR_ENABLE` | 0 | Throttle failsafe disabled |
| `SYSID_MYGCS` | 1 | Must match MAVROS `system_id` (verify: `ros2 param get /mavros system_id`) |
| `BRD_SAFETY_MASK` | 0 | Bypass safety switch requirement |
| `MOT_SPIN_ARM` | 0.15 | Motor spin when armed (15%). **Must be ≤ `MOT_SPIN_MIN`** or arming/motor-test is refused |
| `MOT_SPIN_MIN` | 0.15 | Minimum motor spin (15%) |

> **NPU note:** The NPU script uses **motor test**, which bypasses arming and
> the flight-mode throttle mixer. It does **not** rely on `ARMING_CHECK`,
> `FS_THR_ENABLE`, or `RC_OVERRIDE_TIME`. The only motor-related parameters
> that affect it are `MOT_SPIN_*` (minimum spin thresholds) and the safety
> switch / `BRD_SAFETY_MASK`.

---

## 9. Performance Reference

| | CPU | NPU |
|---|---|---|
| FPS | ~3-5 | ~30+ |
| Power draw | ~8.5W | ~7.6W |
| Latency | ~200-300ms | ~33ms |
| Model | YOLOv8n FP32 (12 MB) | YOLOv8n INT8 (3.3 MB) |

Idle board power: 5.8W | Board max: 11W

---

## 10. NPU Script Features (new)

All of the following are in `ankerCAM_npu_demo.py`. Each is controlled by a
constant near the top of the file, so behavior can be tuned without touching
the loop logic.

### 10.1 Motor control via motor test (core change)

The script sends `MAV_CMD_DO_MOTOR_TEST` (through `/mavros/cmd/command`) to spin
the motors at a throttle **percentage**. This bypasses STABILIZE, the attitude
mixer, arming, and the radio failsafe — which is why motor speed (pitch) now
tracks detection reliably and the motors stay disarmed. A separate command is
sent per motor (`NUM_MOTORS`) and re-issued periodically so all motors stay
spinning continuously.

```python
NUM_MOTORS       = 4      # number of motors on the airframe
MOTOR_TEST_SECS  = 2.0    # each command spins this long before auto-stop
REISSUE_HZ       = 2      # re-send rate to keep motors spinning
```

### 10.2 Distance-scaled pitch

Pitch rises as the target gets closer, using the detection's bounding-box
height as a distance proxy.

```python
SCALE_PITCH      = True   # False = simple idle/active jump
THROTTLE_MIN_PCT = 25     # pitch when target is far (small in frame)
THROTTLE_MAX_PCT = 85     # pitch when target is close (fills frame)
BOX_FRAC_FAR     = 0.10   # box-height fraction treated as "far"
BOX_FRAC_NEAR    = 0.60   # box-height fraction treated as "near"
```

### 10.3 Lock-on countdown

When a target first appears it must stay detected for `LOCK_SECONDS` before
locking. During the countdown the pitch rises from `LOCK_ACQUIRE_PCT` toward
full and a spoken countdown plays ("detected… 3… 2… 1… locked").

```python
LOCK_ON          = True
LOCK_SECONDS     = 3.0    # persist time before full lock
LOCK_ACQUIRE_PCT = 20     # pitch floor during the countdown
```

### 10.4 Smooth pitch ramping

Pitch glides between values instead of jumping, so the motor tone slides
smoothly.

```python
SMOOTH_RAMP      = True
RAMP_RATE        = 120.0  # %/sec; higher = snappier
```

### 10.5 Voice announcements

Spoken status via `espeak-ng`, played one phrase at a time from a background
queue (no overlapping audio). Announces the actual class name ("person
detected", "clock locked"), the countdown, loss of target, and direction
(left / right / centered) once locked. Degrades silently if `espeak-ng` is not
installed.

```python
VOICE            = True
VOICE_CMD        = "espeak-ng"   # falls back to "espeak"
```

### 10.6 On-screen HUD overlay

A `textoverlay` element shows the target, direction, pitch %, confidence, box
size and FPS on the video, color-coded: **blue** while locking on, **green**
when locked & centered, **amber** when off-center, **red** while searching.
(Skipped automatically in `--no-display` mode.)

### 10.7 Idle search sweep

When no target is visible the motors gently pulse as if scanning, instead of a
flat idle.

```python
SEARCH_SWEEP     = True
SWEEP_LOW_PCT    = 3
SWEEP_HIGH_PCT   = 12
SWEEP_PERIOD_S   = 2.0
```

### 10.8 Multi-class tracking

The script can track any class in `TRACK_CLASSES`, each with its own pitch
multiplier so different objects sound different. Cycle the active target at
runtime by sending the process **SIGUSR1**:

```bash
kill -USR1 <pid>     # the PID is printed at startup
```

```python
TRACK_CLASSES = [
    {"id": 74, "name": "clock",      "pitch_mult": 1.00},
    {"id": 0,  "name": "person",     "pitch_mult": 0.70},
    {"id": 39, "name": "bottle",     "pitch_mult": 1.20},
    {"id": 67, "name": "cell phone", "pitch_mult": 0.85},
]
```

### 10.9 Session logging + plot

Every frame is logged to a CSV. After a run, plot it with the companion
`plot_session.py` script.

```python
LOG_CSV   = True
CSV_PATH  = "~/Drone/session_log.csv"
```

```bash
python3 plot_session.py                  # opens the plot
python3 plot_session.py --save run.png   # save to file (headless)
```

> **Note:** Each run overwrites the CSV. Copy/rename it before the next run if
> you want to keep a session.

### 10.10 Telemetry web page (`--web`)

With `--web`, a small built-in HTTP server serves a live dashboard (direction,
target, pitch, confidence, size, FPS) that updates several times a second.

```bash
python3 ankerCAM_npu_demo.py --web
# then open http://<device-ip>:8088 on a phone/laptop on the same network
```

```python
WEB_TELEMETRY = "--web" in sys.argv
WEB_PORT      = 8088
```

> The web server binds `0.0.0.0` and is unauthenticated — fine for a local
> demo network; do not expose it to the open internet.

---

## 11. Safety Notes

- The demo is intended to be run **without propellers** for motor-noise/HUD
  testing. Keep props off and the area clear of the motors.
- Motor test spins the motors under power with no flight logic — treat it like
  any live-motor bench test.
- If this airframe will ever fly again, restore the default flight-controller
  parameters (especially `ARMING_CHECK`, `FS_THR_ENABLE`, `MOT_SPIN_*`) first;
  several are set permissively for this ground demo.
