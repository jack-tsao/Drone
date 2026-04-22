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

# ─────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────

MODEL_FILE = "yolov8n_det.tflite"

CAMERA_DEVICE    = "/dev/video6"
FRAME_W, FRAME_H = 640, 480
MODEL_INPUT_SIZE = 640

MODEL_DIR = os.path.expanduser(
    "~/ai-hub/EdgeAI_Workflow/ai_system/qualcomm/aom-dk2721/linux/script"
)

PWM_IDLE     = 1150
PWM_ACTIVE   = 1750
PWM_YAW_C    = 1500
PWM_YAW_L    = 1150
PWM_YAW_R    = 1850
RC_HZ        = 25

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
    from mavros_msgs.srv import CommandBool, SetMode


# ─────────────────────────────────────────────────────────
#  DRONE CONTROLLER
# ─────────────────────────────────────────────────────────
if not TEST_MODE:
    class DroneNode(Node):
        def __init__(self):
            super().__init__('npu_drone_tracker')
            self._sensor_grp = ReentrantCallbackGroup()
            self._svc_grp = MutuallyExclusiveCallbackGroup()

            self.rc_pub = self.create_publisher(
                OverrideRCIn, '/mavros/rc/override', 10
            )
            self.state = None
            self.create_subscription(
                State, '/mavros/state', self._on_state,
                qos_profile_sensor_data, callback_group=self._sensor_grp,
            )
            self.mode_cli = self.create_client(
                SetMode, '/mavros/set_mode', callback_group=self._svc_grp
            )
            self.arm_cli = self.create_client(
                CommandBool, '/mavros/cmd/arming', callback_group=self._svc_grp
            )

            self._lock = threading.Lock()
            self._throttle = 1000
            self._yaw = PWM_YAW_C

            self.create_timer(
                1.0 / RC_HZ, self._rc_tick, callback_group=self._sensor_grp
            )

        def _on_state(self, msg):
            self.state = msg

        def _rc_tick(self):
            with self._lock:
                t, y = self._throttle, self._yaw
            msg = OverrideRCIn()
            msg.channels = [65535] * 18
            msg.channels[0] = 1500
            msg.channels[1] = 1500
            msg.channels[2] = t
            msg.channels[3] = y
            self.rc_pub.publish(msg)

        def set_cmd(self, throttle, yaw):
            throttle = max(1000, min(2000, int(throttle)))
            yaw = max(1000, min(2000, int(yaw)))
            with self._lock:
                self._throttle = throttle
                self._yaw = yaw

        def _call(self, client, req, timeout=4.0):
            future = client.call_async(req)
            deadline = time.time() + timeout
            while not future.done():
                if time.time() > deadline:
                    self.get_logger().error("Service call timed out")
                    return None
                time.sleep(0.05)
            return future.result()

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
            self.set_cmd(1000, PWM_YAW_C)
            time.sleep(2.0)

            self.get_logger().info("Setting STABILIZE mode...")
            while not self.mode_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("  waiting for mode service...")
            req = SetMode.Request()
            req.custom_mode = "STABILIZE"
            res = self._call(self.mode_cli, req)
            if not res or not res.mode_sent:
                self.get_logger().error("Failed to set STABILIZE")
                return False
            self.get_logger().info("STABILIZE OK")

            self.get_logger().info("Arming...")
            while not self.arm_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("  waiting for arm service...")
            req = CommandBool.Request()
            req.value = True
            res = self._call(self.arm_cli, req)
            if not res or not res.success:
                self.get_logger().error("ARM REJECTED")
                return False
            self.get_logger().info("ARMED")

            self.set_cmd(PWM_IDLE, PWM_YAW_C)
            time.sleep(1.5)
            return True

        def disarm(self):
            self.get_logger().info("Disarming...")
            self.set_cmd(1000, PWM_YAW_C)
            time.sleep(1.5)
            req = CommandBool.Request()
            req.value = False
            self._call(self.arm_cli, req)
            self.get_logger().info("DISARMED")


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
            '! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 '
            '! videoconvert '
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
            '! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 '
            '! videoconvert '
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
    left_b = int(FRAME_W * DEADZONE_L_FRAC)
    right_b = int(FRAME_W * DEADZONE_R_FRAC)

    log.info(f"Target: clock (class {TARGET_CLASS})  conf>{CONF_THRESHOLD}")
    log.info(f"Deadzone: left<{left_b}px  right>{right_b}px")
    log.info(f"PWM: idle={PWM_IDLE}  active={PWM_ACTIVE}  "
             f"yawL={PWM_YAW_L}  yawR={PWM_YAW_R}")
    log.info("Press Ctrl+C to stop")

    is_active = False
    hold_counter = HOLD_FRAMES  # start idle
    last_cx = FRAME_W / 2.0
    last_direction = ""
    running = True
    frame_count = 0
    STATUS_LOG_INTERVAL = 10

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
        detected = False
        center_x = None

        if clock_dets:
            best = max(clock_dets, key=lambda d: d['conf'])
            center_x = best['cx'] * (FRAME_W / MODEL_INPUT_SIZE)
            detected = True

        # Hold timer
        if detected:
            hold_counter = 0
            last_cx = center_x
        else:
            hold_counter += 1

        effectively_detected = (hold_counter < HOLD_FRAMES)
        cx = last_cx if effectively_detected else FRAME_W / 2.0

        # State transitions
        if effectively_detected and not is_active:
            if clock_dets:
                best = max(clock_dets, key=lambda d: d['conf'])
                log.info(f"CLOCK DETECTED (conf={best['conf']:.0%}) "
                         f"-> SPOOL UP ({PWM_ACTIVE} us)!")
            is_active = True

        elif not effectively_detected and is_active:
            log.info(f"CLOCK LOST -> IDLE ({PWM_IDLE} us)")
            is_active = False

        # Motor commands + logging
        if is_active:
            if cx < left_b:
                yaw = PWM_YAW_L
                direction = "<<< YAW LEFT"
            elif cx > right_b:
                yaw = PWM_YAW_R
                direction = "YAW RIGHT >>>"
            else:
                yaw = PWM_YAW_C
                direction = "=== CENTERED ==="

            if not TEST_MODE:
                node.set_cmd(PWM_ACTIVE, yaw)

            if direction != last_direction:
                log.info(f"CLOCK at cx={cx:.0f}/{FRAME_W}  {direction}  "
                         f"thr={PWM_ACTIVE} yaw={yaw}")
                last_direction = direction
            elif frame_count % STATUS_LOG_INTERVAL == 0:
                log.info(f"CLOCK at cx={cx:.0f}/{FRAME_W}  {direction}  "
                         f"thr={PWM_ACTIVE} yaw={yaw}")
        else:
            if not TEST_MODE:
                node.set_cmd(PWM_IDLE, PWM_YAW_C)
            if last_direction != "IDLE":
                last_direction = "IDLE"

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