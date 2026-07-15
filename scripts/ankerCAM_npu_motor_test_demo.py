#!/usr/bin/env python3
"""
yolo_npu_drone.py — QCS6490 NPU YOLO Tracker + Motor Control
==============================================================
ASR-D501 / Qualcomm QCS6490 HTP NPU / ROS 2 Jazzy / MAVROS

Reads raw tensor output from NPU, filters by class ID (clock=74).
Display shows bounding boxes via qtimlpostprocess + videomixer.
Same logic as the CPU version but running on the 12 TOPS NPU.

USAGE:
  ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200   # Terminal 1
  python3 yolo_npu_drone.py                                      # Terminal 2
  python3 yolo_npu_drone.py --test                               # No motors
  python3 yolo_npu_drone.py --no-display                         # Headless/SSH
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import numpy as np
import threading
import time
import os
import sys
import signal
import subprocess

# ─────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────

MODEL_FILE = "yolov8n_det.tflite"

CAMERA_DEVICE    = "/dev/video2"
FRAME_W, FRAME_H = 640, 480
MODEL_INPUT_SIZE = 640

MODEL_DIR = os.path.expanduser(
    "~/ai-hub/EdgeAI_Workflow/ai_system/qualcomm/aom-dk2721/linux/script"
)

# ── Motor-test "pitch" settings (throttle PERCENT, 0-100) ──
THROTTLE_IDLE    = 1     # low hum when no clock
THROTTLE_ACTIVE  = 60     # higher pitch when clock detected
NUM_MOTORS       = 4      # how many motors on the airframe
MOTOR_TEST_SECS  = 2.0    # each command spins this long before auto-stop
REISSUE_HZ       = 2      # how often to re-send the command (keeps sound continuous)

DEADZONE_L_FRAC = 0.38
DEADZONE_R_FRAC = 0.62

TARGET_CLASS   = 74     # "clock" in labels.txt (0-indexed)
CONF_THRESHOLD = 0.55
HOLD_FRAMES    = 30

TEST_MODE      = "--test" in sys.argv
NO_DISPLAY     = "--no-display" in sys.argv

# ─────────────────────────────────────────────────────────
#  ROS 2 IMPORTS
# ─────────────────────────────────────────────────────────
if not TEST_MODE:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.callback_groups import (
        ReentrantCallbackGroup,
        MutuallyExclusiveCallbackGroup,
    )
    from mavros_msgs.msg import OverrideRCIn, State
    from mavros_msgs.srv import CommandBool, SetMode, CommandLong


# ─────────────────────────────────────────────────────────
#  DRONE CONTROLLER
# ─────────────────────────────────────────────────────────
if not TEST_MODE:
    class DroneNode(Node):
        """
        Motor-noise demo controller.

        Uses MAV_CMD_DO_MOTOR_TEST to spin motors at a chosen throttle
        PERCENT, bypassing STABILIZE, the attitude mixer, and the
        arming/radio failsafe gating. Motors MUST be disarmed for this.
        No flight, no stabilization — just controllable motor RPM (pitch).
        """
        def __init__(self):
            super().__init__('npu_drone_tracker')
            self._svc_grp = MutuallyExclusiveCallbackGroup()

            self.state = None
            self.create_subscription(
                State, '/mavros/state', self._on_state,
                qos_profile_sensor_data, callback_group=self._svc_grp,
            )
            # /mavros/cmd/command sends a raw COMMAND_LONG (MAV_CMD_DO_MOTOR_TEST)
            self.cmd_cli = self.create_client(
                CommandLong, '/mavros/cmd/command', callback_group=self._svc_grp
            )

            self._last_pct = None   # avoid log spam

        def _on_state(self, msg):
            self.state = msg

        def _call(self, client, req, timeout=4.0):
            future = client.call_async(req)
            deadline = time.time() + timeout
            while not future.done():
                if time.time() > deadline:
                    self.get_logger().error("Service call timed out")
                    return None
                time.sleep(0.05)
            return future.result()

        def spin_motors(self, throttle_pct):
            """Spin ALL motors at throttle_pct (0-100). Higher = higher pitch."""
            throttle_pct = max(0.0, min(100.0, float(throttle_pct)))
            for motor in range(1, NUM_MOTORS + 1):
                req = CommandLong.Request()
                req.broadcast = False
                req.command = 209          # MAV_CMD_DO_MOTOR_TEST
                req.confirmation = 0
                req.param1 = float(motor)      # this specific motor
                req.param2 = 0.0               # 0 = throttle PERCENT
                req.param3 = throttle_pct
                req.param4 = float(MOTOR_TEST_SECS)
                req.param5 = 1.0               # test 1 motor (this one)
                req.param6 = 0.0
                req.param7 = 0.0
                self._call(self.cmd_cli, req)
            if throttle_pct != self._last_pct:
                self.get_logger().info(f"MOTORS -> {throttle_pct:.0f}%")
                self._last_pct = throttle_pct

        def setup(self):
            self.get_logger().info("Waiting for FCU connection...")
            deadline = time.time() + 15.0
            while time.time() < deadline:
                if self.state and self.state.connected:
                    break
                time.sleep(0.2)
            else:
                self.get_logger().error(
                    "FCU not connected after 15 s. "
                    "Run: ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200"
                )
                return False

            self.get_logger().info("FCU connected")
            while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("  waiting for command service...")
            self.get_logger().info("Ready (motor-test mode, DISARMED)")
            self.spin_motors(THROTTLE_IDLE)   # start at idle hum
            return True

        def disarm(self):
            # Stop the motors (0% explicitly stops the motor test)
            self.get_logger().info("Stopping motors...")
            self.spin_motors(0)
            time.sleep(0.5)


# ─────────────────────────────────────────────────────────
#  TENSOR POST-PROCESSING
# ─────────────────────────────────────────────────────────
def parse_yolov8_tensors(raw_bytes):
    """
    Parse 3-tensor YOLOv8 output: boxes[8400,4], scores[8400], classes[8400].
    Returns list of dicts: {class_id, conf, cx, cy, w, h}
    """
    floats = np.frombuffer(raw_bytes, dtype=np.float32)
    if len(floats) != 50400:
        return []

    boxes = floats[:8400*4].reshape(8400, 4)
    scores = floats[8400*4 : 8400*4 + 8400]
    classes = floats[8400*4 + 8400 :]

    mask = scores > CONF_THRESHOLD
    indices = np.where(mask)[0]

    detections = []
    for idx in indices:
        cx, cy, w, h = boxes[idx]
        detections.append(dict(
            class_id=int(round(classes[idx])),
            conf=float(scores[idx]),
            cx=float(cx), cy=float(cy),
            w=float(w), h=float(h),
        ))
    return detections


# ─────────────────────────────────────────────────────────
#  APPSINK HELPER
# ─────────────────────────────────────────────────────────
def pull_sample(sink, timeout_ns):
    try:
        return sink.try_pull_sample(timeout_ns)
    except AttributeError:
        pass
    try:
        return sink.emit("try-pull-sample", timeout_ns)
    except Exception:
        pass
    return None


# ─────────────────────────────────────────────────────────
#  LOGGER
# ─────────────────────────────────────────────────────────
class SimpleLogger:
    def info(self, msg):  print(f"[INFO]  {msg}")
    def warn(self, msg):  print(f"[WARN]  {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")

# ─────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────
def main():
    Gst.init(None)
    log = SimpleLogger()

    if TEST_MODE:
        log.info("=== TEST MODE — no motors, just detection output ===")
        node = None
    else:
        rclpy.init()
        node = DroneNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()
        log = node.get_logger()

    os.chdir(MODEL_DIR)
    log.info(f"Working directory: {os.getcwd()}")

    # ── Pipeline ──────────────────────────────────────────────
    # Structure:
    #   camera → NV12 → tee(split)
    #     ├── clean branch → BGRA → videomixer(mix) → waylandsink (display)
    #     └── inference → qtimltflite → tee(tensor_tee)
    #           ├── appsink (Python reads raw tensors for class filtering)
    #           └── qtimlpostprocess → BGRA overlay → mix (draws bounding boxes)
    #
    # The clean branch MUST define videomixer before the inference
    # branch references mix. — gst_parse_launch processes left to right.

    if NO_DISPLAY:
        # No display: just tensor appsink + fakesink for inference branch
        pipeline_str = (
            f'v4l2src device="{CAMERA_DEVICE}" '
            '! image/jpeg,width=640,height=480 '
            '! jpegparse ! jpegdec ! videoconvert '
            '! video/x-raw,format=NV12,width=640,height=480 '
            '! qtivtransform '
            '! video/x-raw,format=NV12,width=640,height=480 '
            '! queue max-size-buffers=2 leaky=downstream '
            '! qtimlvconverter '
            '! qtimltflite delegate=external '
            '  external-delegate-path=libQnnTFLiteDelegate.so '
            '  external-delegate-options="QNNExternalDelegate,backend_type=htp;" '
            f'  model={MODEL_FILE} '
            '! appsink name=tensor_sink max-buffers=2 drop=true sync=false emit-signals=false '
        )
    else:
        # Display: original shell script pipeline + tensor appsink tapped off
        pipeline_str = (
            f'v4l2src device="{CAMERA_DEVICE}" '
            '! image/jpeg,width=640,height=480 '
            '! jpegparse ! jpegdec ! videoconvert '
            '! video/x-raw,format=NV12,width=640,height=480 '
            '! qtivtransform '
            '! video/x-raw,format=NV12,width=640,height=480 '
            '! tee name=split '

            # ── Clean branch: defines videomixer, feeds display ──
            'split. ! queue max-size-buffers=20 leaky=no '
            '! videoconvert '
            '! video/x-raw,format=BGRA,width=640,height=480 '
            '! videomixer name=mix '
            '! queue ! waylandsink fullscreen=false sync=false '

            # ── Inference branch → tee for tensor + display ──
            'split. ! queue max-size-buffers=2 leaky=downstream '
            '! qtimlvconverter '
            '! qtimltflite delegate=external '
            '  external-delegate-path=libQnnTFLiteDelegate.so '
            '  external-delegate-options="QNNExternalDelegate,backend_type=htp;" '
            f'  model={MODEL_FILE} '
            '! tee name=tensor_tee '

            # ── Tensor appsink (Python reads raw detections) ──
            'tensor_tee. ! queue max-size-buffers=2 leaky=downstream '
            '! appsink name=tensor_sink max-buffers=2 drop=true sync=false emit-signals=false '

            # ── Overlay: postprocess draws boxes → videomixer ──
            'tensor_tee. ! queue max-size-buffers=2 leaky=downstream '
            '! qtimlpostprocess '
            '  module=yolov8 '
            '  labels=labels.txt '
            '  results=10 '
            '  bbox-stabilization=true '
            '! video/x-raw,format=BGRA,width=640,height=480 '
            '! queue max-size-buffers=2 leaky=downstream '
            '! mix. '
        )

    log.info("Launching pipeline...")
    try:
        pipeline = Gst.parse_launch(pipeline_str)
    except GLib.GError as e:
        log.error(f"Pipeline parse failed: {e}")
        if not TEST_MODE:
            rclpy.shutdown()
        return

    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        log.error("Pipeline failed to start")
        pipeline.set_state(Gst.State.NULL)
        if not TEST_MODE:
            rclpy.shutdown()
        return

    time.sleep(3.0)
    _, state, _ = pipeline.get_state(Gst.CLOCK_TIME_NONE)
    if state != Gst.State.PLAYING:
        log.error(f"Pipeline stuck in {state}")
        pipeline.set_state(Gst.State.NULL)
        if not TEST_MODE:
            rclpy.shutdown()
        return

    log.info("Pipeline PLAYING — NPU inference active")

    tensor_sink = pipeline.get_by_name('tensor_sink')
    if tensor_sink is None:
        log.error("Could not find tensor_sink!")
        pipeline.set_state(Gst.State.NULL)
        if not TEST_MODE:
            rclpy.shutdown()
        return

    # ── Arm drone ─────────────────────────────────────────────
    if not TEST_MODE:
        if not node.setup():
            log.error("Drone setup failed")
            pipeline.set_state(Gst.State.NULL)
            rclpy.shutdown()
            return

    # ── Detection loop ────────────────────────────────────────

    log.info(f"Target: clock (class {TARGET_CLASS})  conf>{CONF_THRESHOLD}")
    log.info(f"Motor pitch: idle={THROTTLE_IDLE}%  active={THROTTLE_ACTIVE}%  "
             f"motors={NUM_MOTORS}")
    log.info("Press Ctrl+C to stop")

    is_active = False
    hold_counter = HOLD_FRAMES  # start idle
    last_cx = FRAME_W / 2.0
    last_direction = ""
    running = True
    frame_count = 0
    STATUS_LOG_INTERVAL = 10

    last_cmd_time = 0.0
    REISSUE_INTERVAL = 1.0 / REISSUE_HZ   # re-send so motor test doesn't lapse

    def on_sigint(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, on_sigint)

    while running:
        sample = pull_sample(tensor_sink, Gst.SECOND)
        if sample is None:
            _, state, _ = pipeline.get_state(0)
            if state not in (Gst.State.PLAYING, Gst.State.PAUSED):
                log.error("Pipeline stopped")
                break
            continue

        buf = sample.get_buffer()
        if buf is None:
            continue

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            continue

        raw_bytes = bytes(mapinfo.data)
        buf.unmap(mapinfo)

        frame_count += 1

        # Parse tensors and filter for clock
        all_dets = parse_yolov8_tensors(raw_bytes)
        clock_dets = [d for d in all_dets if d['class_id'] == TARGET_CLASS]

        # Find best clock detection
        detected = bool(clock_dets)

        # Hold timer (keeps sound steady through brief detection dropouts)
        if detected:
            hold_counter = 0
        else:
            hold_counter += 1

        effectively_detected = (hold_counter < HOLD_FRAMES)

        # State transitions
        if effectively_detected and not is_active:
            best = max(clock_dets, key=lambda d: d['conf']) if clock_dets else None
            conf = f"{best['conf']:.0%}" if best else "held"
            log.info(f"CLOCK DETECTED ({conf}) -> PITCH UP ({THROTTLE_ACTIVE}%)")
            is_active = True
        elif not effectively_detected and is_active:
            log.info(f"CLOCK LOST -> IDLE ({THROTTLE_IDLE}%)")
            is_active = False

        # Choose pitch (throttle percent) based on detection state
        target_pct = THROTTLE_ACTIVE if is_active else THROTTLE_IDLE

        # Re-issue the motor-test command periodically so it stays alive
        now = time.time()
        if now - last_cmd_time >= REISSUE_INTERVAL:
            if not TEST_MODE:
                node.spin_motors(target_pct)
            last_cmd_time = now

    # ── Cleanup ───────────────────────────────────────────────
    log.info("Shutting down...")
    if not TEST_MODE:
        node.disarm()

    pipeline.set_state(Gst.State.NULL)

    if not TEST_MODE:
        executor.shutdown()
        ros_thread.join(timeout=2.0)
        rclpy.shutdown()

    log.info("Done.")


if __name__ == '__main__':
    main()
