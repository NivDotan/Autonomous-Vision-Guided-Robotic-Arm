"""
so101_driver — driver_node

Wraps robot_sam2_app_v2/.../hardware.py's make_hardware() as a ROS2 node.

Publishes:
  /joint_states       sensor_msgs/JointState   @ publish_rate Hz
  /gripper_state      std_msgs/Float32MultiArray  [load, current] on each read

Subscribes:
  /joint_command      sensor_msgs/JointState   → write_ticks() with joint-limit clamping
  /emergency_stop     std_msgs/Bool            → disable torque (True = stop)

Services:
  /emergency_stop_srv  std_srvs/Trigger        → immediate torque off

Parameters:
  backend          string  "daemon"             "daemon" or "feetech"
  serial_port      string  "/dev/ttyACM0"       feetech backend only (was COM4)
  baudrate         int     1_000_000            feetech baud
  dry_run          bool    true                 skip hardware connect entirely
  daemon_endpoint  string  "tcp://localhost:5555"
  publish_rate     float   20.0 Hz
  speed_limit      int     25                   max ticks delta per command (0=off)

Joint limits (from config.py tick ranges) — clamped on every /joint_command:
  shoulder / shoulder_lift  [1000, 3000]
  elbow / elbow_flex        [ 400, 3000]
  palm / wrist_flex         [1000, 3500]
  base/shoulder_pan, wrist/wrist_roll, gripper — no static limit enforced here

Joint name remapping (joint_name_map parameter):
  The official SO-101 URDF (legalaspro/so101-ros-physical-ai) uses different
  joint names from the v2 app's MOTOR_NAMES. The driver auto-remaps on both
  publish (/joint_states) and subscribe (/joint_command).
  Default map (official → internal):
    shoulder_pan  → base       shoulder_lift → shoulder
    elbow_flex    → elbow      wrist_flex    → palm
    wrist_roll    → wrist      gripper       → gripper
  Set use_official_names:=true to publish /joint_states with official names
  (so RViz/MoveIt using the official URDF sees matching joint names).

Note:
  The daemon backend requires motor_daemon to be rebuilt for Linux.
  TODO(hardware): compile motor_daemon/src/motor_daemon.cpp with CMake on Ubuntu.
  Meanwhile, feetech backend works directly via lerobot on /dev/ttyACM0.

Build with --symlink-install for the v2 path-finding to work from source:
  colcon build --symlink-install --packages-select so101_driver
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger

TICKS_CENTER   = 2048
TICKS_PER_REV  = 4096
RAD_PER_TICK   = 2.0 * math.pi / TICKS_PER_REV   # ≈ 0.001534 rad/tick
TICK_PER_RAD   = TICKS_PER_REV / (2.0 * math.pi)  # ≈ 651.9 ticks/rad

def ticks_to_rad(tick: int) -> float:
    # TODO(hardware): add per-joint calibration offset once measured on physical arm.
    return (tick - TICKS_CENTER) * RAD_PER_TICK

def rad_to_ticks(rad: float) -> int:
    return int(round(rad * TICK_PER_RAD + TICKS_CENTER))

# ── Find robot_sam2_app_v2 by walking up from this file ───────────────────────
# Works when built with --symlink-install (Path.resolve() follows symlinks back
# to the source tree). Falls back to dry-run if v2 cannot be located.
def _add_v2_to_path() -> bool:
    for parent in Path(__file__).resolve().parents:
        v2 = parent / 'robot_sam2_app_v2'
        if v2.is_dir():
            sys.path.insert(0, str(v2))
            return True
    return False

_V2_FOUND = _add_v2_to_path()

MOTOR_NAMES = ('base', 'shoulder', 'elbow', 'palm', 'wrist', 'gripper')

# Official SO-101 joint names (legalaspro/so101-ros-physical-ai, TheRobotStudio URDF).
# Motor IDs are identical — only the names differ.
OFFICIAL_NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')

# official name → internal MOTOR_NAMES
OFFICIAL_TO_INTERNAL: dict[str, str] = dict(zip(OFFICIAL_NAMES, MOTOR_NAMES))
# internal → official  (for publishing)
INTERNAL_TO_OFFICIAL: dict[str, str] = dict(zip(MOTOR_NAMES, OFFICIAL_NAMES))

# Tick limits from config.py — applied by internal name. None = unconstrained.
JOINT_LIMITS: dict[str, tuple[int | None, int | None]] = {
    'base':     (None, None),   # TODO(hardware): add base tick range if known
    'shoulder': (1000, 3000),
    'elbow':    (400,  3000),
    'palm':     (1000, 3500),
    'wrist':    (None, None),   # TODO(hardware): wrist tick range not in config.py
    'gripper':  (2100, 3000),
}


def _clamp(value: int, lo: int | None, hi: int | None) -> int:
    if lo is not None and value < lo:
        return lo
    if hi is not None and value > hi:
        return hi
    return value


class DriverNode(Node):
    def __init__(self) -> None:
        super().__init__('so101_driver')

        self.declare_parameter('backend',            'daemon')
        self.declare_parameter('serial_port',        '/dev/ttyACM0')
        self.declare_parameter('baudrate',           1_000_000)
        self.declare_parameter('dry_run',            True)
        self.declare_parameter('daemon_endpoint',    'tcp://localhost:5555')
        self.declare_parameter('daemon_pub_endpoint','tcp://localhost:5556')
        self.declare_parameter('publish_rate',       20.0)
        self.declare_parameter('speed_limit',        25)
        self.declare_parameter('use_official_names', False)

        self._dry_run       = self.get_parameter('dry_run').value
        self._speed         = self.get_parameter('speed_limit').value
        self._use_official  = self.get_parameter('use_official_names').value
        self._stopped       = False
        self._hw            = None
        self._connected     = False
        self._sub_sock      = None   # ZMQ SUB for daemon state broadcast
        self._sub_ticks     = None   # latest ticks from broadcast
        self._pub_names = list(OFFICIAL_NAMES) if self._use_official else list(MOTOR_NAMES)

        if not _V2_FOUND:
            self.get_logger().warning(
                'robot_sam2_app_v2 not found in workspace tree. '
                'Build with --symlink-install and ensure the v2 directory exists. '
                'Falling back to dry-run mode.'
            )

        if not self._dry_run and _V2_FOUND:
            self._connect()
        else:
            self.get_logger().info('Dry-run: hardware not connected (no commands sent to motors).')

        self._pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self._pub_grip = self.create_publisher(Float32MultiArray, '/gripper_state', 10)
        self.create_subscription(JointState, '/joint_command', self._cmd_cb, 10)
        self.create_subscription(Bool, '/emergency_stop', self._estop_cb, 10)
        self.create_service(Trigger, '/emergency_stop_srv', self._estop_srv_cb)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish_state)
        self.get_logger().info('so101_driver ready.')

    # ── hardware connect ──────────────────────────────────────────────────────

    def _connect(self) -> None:
        backend  = self.get_parameter('backend').value
        endpoint = self.get_parameter('daemon_endpoint').value
        port     = self.get_parameter('serial_port').value

        if backend == 'daemon':
            pub_ep = self.get_parameter('daemon_pub_endpoint').value
            # Subscribe to the daemon's PUB broadcast for reading joint state.
            # This lets the ROS2 driver coexist with the v2 app on the same daemon
            # (v2 app uses REQ/REP for commands; we only subscribe to PUB for state).
            try:
                import zmq, msgpack as _mp
                self._zmq_mp = _mp
                ctx = zmq.Context()
                self._sub_sock = ctx.socket(zmq.SUB)
                self._sub_sock.setsockopt(zmq.RCVTIMEO, 50)
                self._sub_sock.setsockopt_string(zmq.SUBSCRIBE, '')
                self._sub_sock.connect(pub_ep)
                self.get_logger().info(
                    f'Daemon backend: subscribed to state broadcast at {pub_ep}. '
                    f'Commands via {endpoint}.'
                )
                self._connected = True
            except Exception as exc:
                self.get_logger().error(f'Cannot connect to daemon PUB: {exc}')
                return
            # Also connect REQ/REP for sending commands
            if _V2_FOUND:
                try:
                    from robot_sam2_app.hardware import make_hardware
                    self._hw = make_hardware(use_daemon=True, endpoint=endpoint)
                    self._hw.connect()
                except Exception:
                    pass   # command path optional — read-only mode still works
            return   # skip feetech fallback below
        else:
            # feetech backend: try lerobot first, fall back to our direct pyserial driver
            self._hw = self._make_feetech(port)
            if self._hw is None:
                return

        self._connected = self._hw.connect()
        if self._connected:
            self.get_logger().info(f'Hardware connected (backend={backend})')
        else:
            self.get_logger().warning(
                f'Hardware not available (backend={backend}, '
                f'{"endpoint=" + endpoint if backend == "daemon" else "port=" + port}). '
                'Node keeps running; /joint_command dropped until hardware reconnects.'
            )

    def _make_feetech(self, port: str):
        """Try lerobot FeetechHardware; fall back to direct pyserial driver."""
        # Only use lerobot if it is actually importable (it is NOT on fresh Linux).
        # We test the actual sub-module that does the serial work, not just the v2 wrapper.
        if _V2_FOUND:
            try:
                from lerobot.motors.feetech import FeetechMotorsBus  # noqa: F401
                from robot_sam2_app.hardware import make_hardware
                self.get_logger().info(f'Feetech: lerobot FeetechHardware on {port}')
                return make_hardware(use_daemon=False, port=port)
            except ImportError:
                self.get_logger().info('lerobot not importable — using direct pyserial driver')
            except Exception as exc:
                self.get_logger().warning(f'lerobot init error ({exc}) — using direct pyserial driver')

        # Direct pyserial driver: works on Linux without any extra pip installs
        # (pyserial is already installed on this machine).
        try:
            from .feetech_direct import FeetechDirectHardware
            self.get_logger().info(f'Feetech: direct pyserial driver on {port}')
            return FeetechDirectHardware(port=port)
        except ImportError as exc:
            self.get_logger().error(f'No Feetech driver: {exc} — pip install pyserial')
            return None

    # ── publish state ─────────────────────────────────────────────────────────

    def _publish_state(self) -> None:
        if not self._connected:
            return
        # Daemon mode: read from the PUB broadcast (non-blocking)
        if self._sub_sock is not None:
            try:
                raw = self._sub_sock.recv()
                msg = self._zmq_mp.unpackb(raw, raw=False)
                tl  = msg.get('ticks') or msg.get(b'ticks')
                if tl:
                    ticks = {n: int(tl[i]) for i, n in enumerate(MOTOR_NAMES)}
                    self._sub_ticks = ticks
            except Exception:
                pass   # timeout or no data yet
            ticks = self._sub_ticks
            if ticks is None:
                return
        elif self._hw is not None:
            ticks = self._hw.read_ticks()
            if ticks is None:
                return
        else:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = self._pub_names
        # Convert ticks → radians so robot_state_publisher can drive TF/RViz.
        # TODO(hardware): add per-joint zero-offset calibration once measured.
        msg.position = [ticks_to_rad(ticks.get(n, TICKS_CENTER)) for n in MOTOR_NAMES]
        self._pub_js.publish(msg)

        # Gripper current + load (one ZMQ call for daemon, separate reads for feetech)
        try:
            load, current = self._hw.read_gripper_state()
            if load is not None or current is not None:
                g = Float32MultiArray()
                g.data = [float(load or 0), float(current or 0)]
                self._pub_grip.publish(g)
        except Exception:
            pass

    # ── command subscriber ────────────────────────────────────────────────────

    def _cmd_cb(self, msg: JointState) -> None:
        if self._stopped:
            self.get_logger().debug('Emergency stop active — ignoring /joint_command')
            return
        if self._hw is None or not self._connected:
            return
        ticks: dict[str, int] = {}
        for name, pos in zip(msg.name, msg.position):
            internal = OFFICIAL_TO_INTERNAL.get(name, name)
            # Accept both radians (|pos| < 20) and raw ticks (pos > 100).
            # Radians: task planner / joint_state_publisher_gui output.
            # Ticks: legacy commands from the v2 app style.
            if abs(pos) < 20.0:
                raw = rad_to_ticks(pos)
            else:
                raw = int(pos)
            lo, hi = JOINT_LIMITS.get(internal, (None, None))
            ticks[internal] = _clamp(raw, lo, hi)
        self._hw.write_ticks(ticks)

    # ── emergency stop ────────────────────────────────────────────────────────

    def _estop_cb(self, msg: Bool) -> None:
        self._stopped = bool(msg.data)
        if self._stopped:
            self.get_logger().warning('Emergency stop ACTIVATED — torque off.')
            if self._hw is not None and self._connected:
                self._hw.set_torque(False)
        else:
            self.get_logger().info('Emergency stop released.')

    def _estop_srv_cb(self, _req: Trigger.Request, resp: Trigger.Response):
        self._estop_cb(Bool(data=True))
        resp.success = True
        resp.message = 'Emergency stop activated.'
        return resp

    def destroy_node(self) -> None:
        if self._hw is not None and self._connected:
            self._hw.disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
