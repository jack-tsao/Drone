#!/usr/bin/env python3
"""
YOLO Clock Tracker — Executor Fix
====================================
ArduCopter V4.6.x / Pixhawk1 / ROS 2 Jazzy / MAVROS

FIX: "Executor is already spinning" error.

Root cause: rclpy.spin() in background thread locks the executor.
            Calling spin_once() or spin_until_future_complete() from
            setup_drone() on the same node then crashes.

Solution:
  1. MultiThreadedExecutor — allows concurrent callbacks safely.
  2. _call_and_wait() now polls future.done() in a sleep loop instead
     of calling any spin function — safe to call from any thread.
  3. setup_drone() uses time.sleep() instead of spin_for() so it never
     touches the executor directly.

BEFORE RUNNING — set in Mission Planner:
  RC_OVERRIDE_TIME = 3     (already set per your screenshot)
  FS_THR_ENABLE    = 0     (disable throttle failsafe)
  SYSID_MYGCS      = 1     (MAVROS ROS 2 defaults to system_id=1, NOT 255.
                             Verify with: ros2 param get /mavros system_id
                             Set this to match whatever that command returns.)
  BRD_SAFETY_MASK  = 0     (bypass safety switch)
  MOT_SPIN_ARM     = 0.10
  MOT_SPIN_MIN     = 0.15

LAUNCH MAVROS first:
  ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import threading
import time
import sys
import signal

from mavros_msgs.msg import OverrideRCIn, RCOut, State
from mavros_msgs.srv import CommandBool, SetMode

import cv2
from ultralytics import YOLO


# ─────────────────────────────────────────────────────────
#  CAMERA
# ─────────────────────────────────────────────────────────
CAMERA_INDEX  = 2
CAMERA_WIDTH  = 640    # Force down from 4K — YOLO only needs 640px anyway
CAMERA_HEIGHT = 480    # Lower res = less data per frame = less lag
CAMERA_FPS    = 30     # Cap FPS — 4K webcams default to lower FPS anyway
CAMERA_BUFFER = 1      # OpenCV frame buffer size — 1 = always get latest frame,
                       # never process stale queued frames (key fix for lag)
# V4L2 autofocus/exposure — applied via cv2.CAP_PROP_* after open.
# AnkerWork supports these over UVC. If they have no effect, install
# v4l2-utils and run:
#   v4l2-ctl -d /dev/video2 --set-ctrl=focus_automatic_continuous=0
#   v4l2-ctl -d /dev/video2 --set-ctrl=focus_absolute=30
DISABLE_AUTOFOCUS = True   # True = manual focus (stops hunting during demo)
FOCUS_VALUE       = 30     # Manual focus distance (0-255, 30 ≈ ~1 metre)

# ─────────────────────────────────────────────────────────
#  PWM VALUES
# ─────────────────────────────────────────────────────────
PWM_IDLE_ARMED  = 1150   # Just above MOT_SPIN_ARM — barely spinning
PWM_CLOCK_SEEN  = 1750   # Loud spool-up on clock detection
PWM_YAW_CENTER  = 1500
PWM_YAW_LEFT    = 1150   # Strong yaw — audible motor difference
PWM_YAW_RIGHT   = 1850

OVERRIDE_RATE_HZ = 25    # RC override publish frequency
FRAME_READ_TIMEOUT_SEC = 0.5

class YoloTrackerNode(Node):

    def __init__(self):
        super().__init__('yolo_drone_tracker')

        # ── Callback groups ──────────────────────────────────────────
        # ReentrantCallbackGroup  — timer + subscribers can fire in parallel.
        # MutuallyExclusiveCallbackGroup — service clients are isolated so a
        # slow service call never blocks the RC timer or state callbacks.
        self._sensor_group  = ReentrantCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()

        # ── Publisher ────────────────────────────────────────────────
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # ── Subscribers ──────────────────────────────────────────────
        self.state  = None
        self.rc_out = None
        # MAVROS publishes State and RCOut with Best Effort QoS (sensor data profile).
        # Using the default Reliable QoS here causes a silent mismatch — ROS 2
        # refuses to connect them and you get "FCU not connected after 15s".
        self.create_subscription(State, '/mavros/state',  self._cb_state,
                                 qos_profile_sensor_data,
                                 callback_group=self._sensor_group)
        self.create_subscription(RCOut, '/mavros/rc/out', self._cb_rc_out,
                                 qos_profile_sensor_data,
                                 callback_group=self._sensor_group)

        # ── Service clients ──────────────────────────────────────────
        self.mode_client = self.create_client(SetMode,     '/mavros/set_mode',
                                              callback_group=self._service_group)
        self.arm_client  = self.create_client(CommandBool, '/mavros/cmd/arming',
                                              callback_group=self._service_group)

        # ── Shared RC command (YOLO thread writes, timer reads) ──────
        self._cmd_lock     = threading.Lock()
        self._cmd_throttle = 1000
        self._cmd_yaw      = PWM_YAW_CENTER

        # ── 25Hz timer — fires independently of YOLO loop speed ──────
        self.create_timer(1.0 / OVERRIDE_RATE_HZ, self._rc_timer_cb,
                          callback_group=self._sensor_group)

    # ── Callbacks ─────────────────────────────────────────────────────
    def _cb_state(self, msg):  self.state = msg
    def _cb_rc_out(self, msg): self.rc_out = msg

    def _rc_timer_cb(self):
        """Publishes RC override at 25Hz — never misses ArduPilot's timeout."""
        with self._cmd_lock:
            thr = self._cmd_throttle
            yaw = self._cmd_yaw

        msg = OverrideRCIn()
        msg.channels = [65535] * 18
        msg.channels[0] = 1500  # Roll  centered
        msg.channels[1] = 1500  # Pitch centered
        msg.channels[2] = thr   # Throttle
        msg.channels[3] = yaw   # Yaw
        self.rc_pub.publish(msg)

    # ── Thread-safe command setter with clamp ─────────────────────────
    def set_command(self, throttle: int, yaw: int):
        """
        Clamps both values to valid ArduPilot PWM range before storing.
        Prevents a future refactor from accidentally sending garbage PWM.
        """
        throttle = max(1000, min(2000, int(throttle)))
        yaw      = max(1000, min(2000, int(yaw)))
        with self._cmd_lock:
            self._cmd_throttle = throttle
            self._cmd_yaw      = yaw

    # ── Service call helper ───────────────────────────────────────────
    def _call_and_wait(self, client, request, timeout_sec=4.0) -> object:
        """
        Calls a service and waits for the result WITHOUT touching the executor.
        Safe to call from any thread while MultiThreadedExecutor is spinning.
        Uses a polling sleep loop instead of spin_until_future_complete().
        """
        future: Future = client.call_async(request)
        deadline = time.time() + timeout_sec

        while not future.done():
            if time.time() > deadline:
                self.get_logger().error("Service call timed out.")
                return None
            time.sleep(0.05)   # yield — executor in background thread handles it

        return future.result()

    # ── Drone setup ────────────────────────────────────────────────────
    def setup_drone(self) -> bool:
        self.get_logger().info("Waiting for FCU connection...")

        # Poll state until connected — plain sleep, no spin calls
        deadline = time.time() + 15.0
        while time.time() < deadline:
            if self.state and self.state.connected:
                break
            time.sleep(0.2)
        else:
            self.get_logger().error("FCU not connected after 15s.")
            self.get_logger().error(
                "Run: ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200"
            )
            self.get_logger().error(
                "If MAVROS is running but state never arrives: QoS mismatch or "
                "SYSID_MYGCS mismatch. Check: ros2 param get /mavros system_id "
                "and make sure Mission Planner SYSID_MYGCS matches that value."
            )
            return False

        self.get_logger().info("FCU connected.")

        # Send minimum throttle — timer is already publishing at 25Hz
        self.set_command(throttle=1000, yaw=PWM_YAW_CENTER)
        time.sleep(2.0)   # let FC register the zero-throttle signal

        # ── Set STABILIZE mode ────────────────────────────────────
        self.get_logger().info("Setting STABILIZE mode...")
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for mode service...")

        mode_req = SetMode.Request()
        mode_req.custom_mode = "STABILIZE"
        res = self._call_and_wait(self.mode_client, mode_req)

        if res is None or not res.mode_sent:
            self.get_logger().error("Failed to set STABILIZE mode.")
            return False
        self.get_logger().info("Mode: STABILIZE ✓")

        # ── Arm ───────────────────────────────────────────────────
        self.get_logger().info("Arming...")
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")

        arm_req = CommandBool.Request()
        arm_req.value = True
        res = self._call_and_wait(self.arm_client, arm_req)

        if res is None or not res.success:
            self.get_logger().error(
                "Arming REJECTED. Check Mission Planner for pre-arm errors."
            )
            return False

        self.get_logger().info("ARMED ✓")

        # Ramp to idle — timer keeps publishing
        self.set_command(throttle=PWM_IDLE_ARMED, yaw=PWM_YAW_CENTER)
        time.sleep(1.5)

        return True

    def disarm(self):
        self.get_logger().info("Disarming...")
        self.set_command(throttle=1000, yaw=PWM_YAW_CENTER)
        time.sleep(1.5)

        disarm_req = CommandBool.Request()
        disarm_req.value = False
        self._call_and_wait(self.arm_client, disarm_req)
        self.get_logger().info("DISARMED.")


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackerNode()

    # ── MultiThreadedExecutor ─────────────────────────────────────────
    # Allows the 25Hz RC timer and service call callbacks to run
    # concurrently without blocking each other or setup_drone().
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin executor in background — this is the ONLY place spin is called
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # ── Load YOLO ─────────────────────────────────────────────────────
    node.get_logger().info("Loading YOLO model...")
    model = YOLO('yolov8n.pt')

    # ── Camera ────────────────────────────────────────────────────────
    if sys.platform.startswith('win'):
        camera_backend = cv2.CAP_DSHOW
        backend_name = 'CAP_DSHOW'
    else:
        camera_backend = cv2.CAP_V4L2
        backend_name = 'CAP_V4L2'

    cap = cv2.VideoCapture(CAMERA_INDEX, camera_backend)
    if not cap.isOpened():
        node.get_logger().error("Camera not found! Change CAMERA_INDEX.")
        executor.shutdown()
        rclpy.shutdown()
        return

    node.get_logger().info(f"Camera backend: {backend_name}")

    # MJPG MUST be set before resolution. Without it the AnkerWork defaults
    # to YUYV (raw uncompressed) which saturates USB 3 bandwidth at 4K and
    # caps you at ~5fps regardless of what you request below.
    # MJPG compresses on the camera chip — same cable, full 30fps.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   CAMERA_BUFFER)

    if DISABLE_AUTOFOCUS:
        af_ok = cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FOCUS, FOCUS_VALUE)
        if not af_ok:
            # Many UVC cameras ignore OpenCV's autofocus property entirely.
            # Use v4l2-ctl directly in that case — it talks to the driver layer.
            node.get_logger().warn(
                "CAP_PROP_AUTOFOCUS was ignored by the driver (AnkerWork does this). "
                "Fix autofocus manually with these two commands:\n"
                f"  v4l2-ctl -d /dev/video{CAMERA_INDEX} --set-ctrl=focus_automatic_continuous=0\n"
                f"  v4l2-ctl -d /dev/video{CAMERA_INDEX} --set-ctrl=focus_absolute={FOCUS_VALUE}\n"
                "See all available controls with:  v4l2-ctl --list-ctrls"
            )
        else:
            node.get_logger().info(f"Autofocus disabled. Focus fixed at {FOCUS_VALUE}.")

    actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    node.get_logger().info(
        f"Camera opened: {actual_w}x{actual_h} @ {actual_fps:.0f}fps  "
        f"(requested {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps)"
    )
    if actual_w > CAMERA_WIDTH or actual_h > CAMERA_HEIGHT:
        node.get_logger().warn(
            f"Driver kept resolution at {actual_w}x{actual_h} despite request. "
            "MJPG may not have applied — YOLO will resize explicitly before inference "
            "so speed is unaffected, but check FOURCC support with: "
            f"v4l2-ctl -d /dev/video{CAMERA_INDEX} --list-formats-ext"
        )

    # ── Arm (runs in main thread — safe because executor is in bg) ────
    if not node.setup_drone():
        node.get_logger().error("Setup failed. Exiting.")
        cap.release()
        executor.shutdown()
        rclpy.shutdown()
        return

    # ── Frame geometry ─────────────────────────────────────────────────
    width          = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    left_boundary  = int(width * 0.38)
    right_boundary = int(width * 0.62)

    node.get_logger().info(
        f"Tracking started. "
        f"IDLE={PWM_IDLE_ARMED}us  ACTIVE={PWM_CLOCK_SEEN}us  "
        f"YAW L={PWM_YAW_LEFT}us R={PWM_YAW_RIGHT}us"
    )
    node.get_logger().info("Press 'q' or Ctrl+C to quit.")

    clock_was_detected = False
    running = True

    def on_sigint(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, on_sigint)

    # ── Main YOLO + display loop ───────────────────────────────────────
    while running and cap.isOpened():
        
        # 1. EMPTY THE BUFFER: Rapid-fire grab to throw away stale frames
        for _ in range(5): 
            cap.grab()
            
        # 2. DECODE THE NEWEST FRAME:
        ret, frame = cap.retrieve()
        
        if not ret:
            continue

        h, w = frame.shape[:2]

        # Deadzone lines
        cv2.line(frame, (left_boundary, 0),  (left_boundary, h),  (255, 128, 0), 2)
        cv2.line(frame, (right_boundary, 0), (right_boundary, h), (255, 128, 0), 2)

        # Skip resize when the camera already delivers target size to avoid
        # an unnecessary per-frame copy that adds latency.
        if w == CAMERA_WIDTH and h == CAMERA_HEIGHT:
            inference_frame = frame
        else:
            # Hard guarantee against driver-ignored resolution settings.
            inference_frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))

        # Plain model() — faster than model.track() on CPU because it skips
        # the BoTSORT ReID matching step. 
        results = model(inference_frame, classes=74, conf=0.45, verbose=False)
        clock_detected = len(results[0].boxes) > 0

        # Scale factor — maps inference coords back to display frame coords
        scale_x = w / CAMERA_WIDTH
        scale_y = h / CAMERA_HEIGHT

        if not clock_detected:
            node.set_command(throttle=PWM_IDLE_ARMED, yaw=PWM_YAW_CENTER)
            if clock_was_detected:
                node.get_logger().info(f"Clock LOST -> IDLE ({PWM_IDLE_ARMED}us)")
            status = "IDLE SEARCHING"
            color  = (0, 0, 255)
            hud_thr = PWM_IDLE_ARMED
            hud_yaw = PWM_YAW_CENTER
            hud_detail = "No clock detected"

        else:
            box = results[0].boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0].cpu().numpy())
            
            # Scale inference coords → display frame coords
            x1, x2 = int(x1 * scale_x), int(x2 * scale_x)
            y1, y2 = int(y1 * scale_y), int(y2 * scale_y)
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 8, (0, 0, 255), -1)

            if not clock_was_detected:
                node.get_logger().info(f"Clock DETECTED -> SPOOL UP ({PWM_CLOCK_SEEN}us)!")

            if center_x < left_boundary:
                yaw_cmd = PWM_YAW_LEFT
                status  = "<<< YAW LEFT"
                color   = (0, 165, 255)
            elif center_x > right_boundary:
                yaw_cmd = PWM_YAW_RIGHT
                status  = "YAW RIGHT >>>"
                color   = (0, 165, 255)
            else:
                yaw_cmd = PWM_YAW_CENTER
                status  = "=== CENTERED ==="
                color   = (0, 255, 0)

            node.set_command(throttle=PWM_CLOCK_SEEN, yaw=yaw_cmd)

            hud_thr = PWM_CLOCK_SEEN
            hud_yaw = yaw_cmd
            hud_detail = f"cx={center_x}/{w}  conf={conf:.0%}"

        clock_was_detected = clock_detected

        # ── HUD ───────────────────────────────────────────────────
        cv2.putText(frame, status, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, color, 2)
        cv2.putText(frame, hud_detail, (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(frame, f"THR={hud_thr}us  YAW={hud_yaw}us", (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(frame, f"L={left_boundary}px", (left_boundary - 50, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 128, 0), 1)
        cv2.putText(frame, f"R={right_boundary}px", (right_boundary + 5, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 128, 0), 1)

        cv2.imshow("Drone Target Lock Demo", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # ── Cleanup ────────────────────────────────────────────────────────
    node.get_logger().info("Shutting down...")
    node.disarm()
    cap.release()
    cv2.destroyAllWindows()
    executor.shutdown()
    ros_thread.join(timeout=1.0)   # wait for bg thread to exit cleanly
    rclpy.shutdown()


if __name__ == '__main__':
    main()