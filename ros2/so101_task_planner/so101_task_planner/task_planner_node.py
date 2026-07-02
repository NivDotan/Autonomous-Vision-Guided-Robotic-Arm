"""
task_planner_node — autonomous pick-and-place state machine.

Coordinates perception, calibration, and driver to execute a full pick-and-place
task. Implements the same state machine as the v2 app's approach pipeline, but
split across proper ROS2 topics/services.

Subscribes:
  /joint_states             sensor_msgs/JointState   (current ticks)
  /target_pixel             so101_interfaces/PixelPoint  (tracked object center)
  /drop_zone_pixel          so101_interfaces/PixelPoint  (place target center)
  /range                    sensor_msgs/Range         (VL53 distance)
  /gripper_state            std_msgs/Float32MultiArray  [load, current]

Publishes:
  /joint_command            sensor_msgs/JointState   (tick targets → so101_driver)
  /task_state               so101_interfaces/TaskState

Services (provided):
  /start_task               so101_interfaces/StartTask
  /abort_task               std_srvs/Trigger

Service clients (called):
  /detect_object            so101_interfaces/DetectObject
  /initialize_tracking      so101_interfaces/InitializeTracking
  /emergency_stop_srv       std_srvs/Trigger

Parameters:
  dry_run           bool   true   true = publish /joint_command but driver is also dry_run
  max_retries       int    3
  tick_rate         float  25.0   Hz (state machine + command rate)
  frame_w           float  640.0
  frame_h           float  480.0
  grip_check_frames int    25     frames after close before declaring miss
  current_threshold float  50.0   CURRENT_GRIP_THRESHOLD
  current_stable_w  float  20.0   CURRENT_STABLE_WINDOW
  current_stable_n  int    5      CURRENT_STABLE_COUNT
  home_ticks_*      int    2048   per-joint home positions (mirrored from config)
"""
from __future__ import annotations

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

try:
    from so101_interfaces.msg import PixelPoint, TaskState as TaskStateMsg
    from so101_interfaces.srv import DetectObject, InitializeTracking, StartTask
    _IFACES = True
except ImportError:
    _IFACES = False

from .state_machine import S, PlannerState, StateMachine, MOTOR_NAMES, DEFAULT_TICKS

GRIP_STABLE_WINDOW = 5   # readings for current stability check


class TaskPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('task_planner')

        self.declare_parameter('dry_run',           True)
        self.declare_parameter('max_retries',        3)
        self.declare_parameter('tick_rate',          25.0)
        self.declare_parameter('frame_w',            640.0)
        self.declare_parameter('frame_h',            480.0)
        self.declare_parameter('grip_check_frames',  25)
        self.declare_parameter('current_threshold',  50.0)
        self.declare_parameter('current_stable_w',   20.0)
        self.declare_parameter('current_stable_n',   5)
        # Home positions (ticks) — mirrored from DEFAULT_TICKS / config.py
        for n, v in DEFAULT_TICKS.items():
            self.declare_parameter(f'home_{n}', v)

        if not _IFACES:
            self.get_logger().error('so101_interfaces not built — planner disabled.')
            return

        # ── state machine ──────────────────────────────────────────────────────
        ps               = PlannerState()
        ps.dry_run       = self.get_parameter('dry_run').value
        ps.max_retries   = self.get_parameter('max_retries').value
        ps.frame_w       = self.get_parameter('frame_w').value
        ps.frame_h       = self.get_parameter('frame_h').value
        ps.home_ticks    = {n: self.get_parameter(f'home_{n}').value for n in MOTOR_NAMES}
        self._ps         = ps
        self._sm         = StateMachine(ps, log=self.get_logger().info)

        self._grip_check_frames = self.get_parameter('grip_check_frames').value
        self._curr_thresh       = self.get_parameter('current_threshold').value
        self._curr_stable_w     = self.get_parameter('current_stable_w').value
        self._curr_stable_n     = self.get_parameter('current_stable_n').value
        self._current_buf: deque[float] = deque(maxlen=10)
        self._detect_pending    = False
        self._track_pending     = False
        self._pending_bbox      = None

        # ── publishers ─────────────────────────────────────────────────────────
        self._cmd_pub   = self.create_publisher(JointState, '/joint_command', 10)
        self._state_pub = self.create_publisher(TaskStateMsg, '/task_state',   10)

        # ── subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(JointState,         '/joint_states',    self._js_cb,    10)
        self.create_subscription(PixelPoint,         '/target_pixel',    self._target_cb, 10)
        self.create_subscription(PixelPoint,         '/drop_zone_pixel', self._dz_cb,     10)
        self.create_subscription(Range,              '/range',           self._range_cb,  10)
        self.create_subscription(Float32MultiArray,  '/gripper_state',   self._grip_cb,   10)

        # ── service clients ────────────────────────────────────────────────────
        self._detect_cli = self.create_client(DetectObject,        '/detect_object')
        self._track_cli  = self.create_client(InitializeTracking,  '/initialize_tracking')
        self._estop_cli  = self.create_client(Trigger,             '/emergency_stop_srv')

        # ── services (provided) ────────────────────────────────────────────────
        self.create_service(StartTask, '/start_task', self._start_srv)
        self.create_service(Trigger,   '/abort_task', self._abort_srv)

        # ── main tick timer ────────────────────────────────────────────────────
        rate = self.get_parameter('tick_rate').value
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'task_planner ready (dry_run={ps.dry_run}).')

    # ── subscriptions ──────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self._ps.curr:
                self._ps.curr[name] = int(pos)

    def _target_cb(self, msg: PixelPoint) -> None:
        self._ps.target_x  = msg.x
        self._ps.target_y  = msg.y
        self._ps.has_target = True

    def _dz_cb(self, msg: PixelPoint) -> None:
        # Drop zone pixel feeds into target during PLACE state
        if self._ps.state == S.PLACE:
            self._ps.target_x  = msg.x
            self._ps.target_y  = msg.y
            self._ps.has_target = True

    def _range_cb(self, msg: Range) -> None:
        self._ps.range_mm = msg.range * 1000.0   # metres → mm

    def _grip_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 2:
            self._ps.gripper_load    = msg.data[0]
            self._ps.gripper_current = msg.data[1]
            self._current_buf.append(msg.data[1])

    # ── services (provided) ────────────────────────────────────────────────────

    def _start_srv(self, req: StartTask.Request, resp: StartTask.Response):
        if self._sm.start_task(req.pick_query, req.place_query):
            self._detect_pending = False
            self._track_pending = False
            self._pending_bbox = None
            self._trigger_detection(req.pick_query)
            resp.success = True
            resp.message = f"Task started: pick='{req.pick_query}' place='{req.place_query}'"
        else:
            resp.success = False
            resp.message = f'Cannot start: current state is {self._ps.state.value}'
        return resp

    def _abort_srv(self, _req, resp: Trigger.Response):
        self._sm.abort()
        resp.success = True
        resp.message = 'Task aborted'
        return resp

    # ── async service calls ────────────────────────────────────────────────────

    def _trigger_detection(self, query: str) -> None:
        if self._detect_pending:
            return
        if not self._detect_cli.service_is_ready():
            self.get_logger().debug('/detect_object service not available yet.')
            return
        req       = DetectObject.Request()
        req.query = query
        future    = self._detect_cli.call_async(req)
        self._detect_pending = True
        future.add_done_callback(self._detection_done_cb)

    def _detection_done_cb(self, future) -> None:
        self._detect_pending = False
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'DetectObject call failed: {exc}')
            return
        if not resp.success:
            self.get_logger().warning(f'Detection failed: {resp.message}')
            self._sm.transition(S.RETRY_OR_DONE)
            return
        self.get_logger().info(f'Detected: {resp.message}')
        if self._ps.range_mm is None:
            self.get_logger().info('[VL53 after detection] no /range sample yet')
        else:
            self.get_logger().info(f'[VL53 after detection] range={self._ps.range_mm:.0f}mm')
        self._ps.target_x = (resp.x_min + resp.x_max) / 2.0
        self._ps.target_y = (resp.y_min + resp.y_max) / 2.0
        self._ps.has_target = True
        self._sm.on_detection(resp.x_min, resp.y_min, resp.x_max, resp.y_max)
        self._pending_bbox = (resp.x_min, resp.y_min, resp.x_max, resp.y_max)
        self._trigger_tracking()

    def _trigger_tracking(self) -> None:
        if self._track_pending or self._pending_bbox is None:
            return
        if self._track_cli.service_is_ready():
            x_min, y_min, x_max, y_max = self._pending_bbox
            treq          = InitializeTracking.Request()
            treq.x_min    = x_min
            treq.y_min    = y_min
            treq.x_max    = x_max
            treq.y_max    = y_max
            treq.camera   = 'gripper'
            tfuture       = self._track_cli.call_async(treq)
            self._track_pending = True
            tfuture.add_done_callback(self._tracking_done_cb)

    def _tracking_done_cb(self, future) -> None:
        self._track_pending = False
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'InitializeTracking call failed: {exc}')
            return
        if resp.success:
            self._pending_bbox = None
            self._sm.on_tracking_ready()
        else:
            self.get_logger().warning(
                f'Tracking init failed: {resp.message}; using detector bbox center as fixed target'
            )
            self._pending_bbox = None
            self._sm.on_tracking_ready()

    # ── main tick ──────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if not _IFACES:
            return
        ps = self._ps

        if ps.state == S.SCAN_OBJECTS and ps.pick_query:
            self._trigger_detection(ps.pick_query)
        elif ps.state == S.SELECT_OBJECT:
            self._trigger_tracking()

        # ── gripper catch detection (in GRASP / VERIFY_GRASP) ─────────────────
        if ps.state in (S.GRASP, S.VERIFY_GRASP) and ps.gripper_closed:
            if ps.grip_frames >= self._grip_check_frames:
                if self._grip_detected():
                    self._sm.on_grip_success()
                else:
                    self._sm.on_grip_miss()
                    self._current_buf.clear()

        # ── state machine step ─────────────────────────────────────────────────
        target_ticks = self._sm.tick()

        # ── publish /joint_command ─────────────────────────────────────────────
        msg             = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name        = list(MOTOR_NAMES)
        msg.position    = [float(target_ticks.get(n, 2048)) for n in MOTOR_NAMES]
        self._cmd_pub.publish(msg)

        # ── publish /task_state ────────────────────────────────────────────────
        ts               = TaskStateMsg()
        ts.header.stamp  = self.get_clock().now().to_msg()
        ts.state         = ps.state.value
        ts.previous_state = ps.previous_state.value
        ts.attempt       = ps.attempt
        ts.dry_run       = ps.dry_run
        ts.message       = f'range={ps.range_mm:.0f}mm' if ps.range_mm else 'no range'
        self._state_pub.publish(ts)

    def _grip_detected(self) -> bool:
        buf = list(self._current_buf)
        if len(buf) < self._curr_stable_n:
            return False
        last = buf[-self._curr_stable_n:]
        spread = max(last) - min(last)
        return spread <= self._curr_stable_w and min(last) > self._curr_thresh


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
