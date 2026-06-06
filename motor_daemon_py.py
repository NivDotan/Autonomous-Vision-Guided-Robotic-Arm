#!/usr/bin/env python3
"""
motor_daemon_py.py — Linux replacement for motor_daemon.exe

Implements the exact same ZMQ REQ/REP + msgpack protocol that
robot_sam2_app_v2 expects, so the v2 app runs unchanged on Linux.

Protocol (identical to motor_daemon.exe):
  cmd 0xFF — STATUS         → {status: 0}
  cmd 0x01 — WRITE_TICKS    {ticks: [6×int]} → {status: 0}
  cmd 0x02 — READ_TICKS     → {ticks: [6×int]}
  cmd 0x03 — GRIPPER_LOAD   → {status, load: int16, current: int16, detected: bool}

Usage:
  python3 motor_daemon_py.py
  python3 motor_daemon_py.py --port /dev/ttyACM0 --zmq-port 5555
"""
import argparse
import sys
import time
import threading

# ── deps ──────────────────────────────────────────────────────────────────────
try:
    import zmq, msgpack
except ImportError:
    print("Missing deps.  Run:  pip install pyzmq msgpack")
    sys.exit(1)

try:
    import serial
except ImportError:
    print("Missing pyserial.  Run:  pip install pyserial")
    sys.exit(1)

# ── Feetech STS3215 protocol (same as feetech_direct.py) ─────────────────────
MOTOR_NAMES = ('base', 'shoulder', 'elbow', 'palm', 'wrist', 'gripper')
MOTOR_IDS   = (1, 2, 3, 4, 5, 6)

REG_TORQUE_ENABLE    = 40
REG_GOAL_POSITION    = 42
REG_PRESENT_POSITION = 56
REG_PRESENT_LOAD     = 60
REG_PRESENT_CURRENT  = 69


def _chk(body):
    return (~sum(body)) & 0xFF


def _read(ser, mid, reg, n):
    body = [mid, 4, 0x02, reg, n]
    body.append(_chk(body))
    ser.reset_input_buffer()
    ser.write(bytes([0xFF, 0xFF]) + bytes(body))
    ser.flush()
    resp = ser.read(n + 6)
    if len(resp) < n + 6 or resp[0] != 0xFF or resp[4] != 0:
        return None
    raw = resp[5:5 + n]
    return raw[0] | (raw[1] << 8) if n == 2 else raw[0]


def _write(ser, mid, reg, value, n=2):
    data = [value & 0xFF, (value >> 8) & 0xFF] if n == 2 else [value & 0xFF]
    body = [mid, 3 + n, 0x03, reg] + data
    body.append(_chk(body))
    ser.write(bytes([0xFF, 0xFF]) + bytes(body))
    ser.flush()
    ser.read(6)   # consume ack


def _sync_write(ser, reg, pairs):
    """Write goal position to multiple motors in one packet."""
    params = [reg, 2]
    for mid, val in pairs:
        params += [mid, val & 0xFF, (val >> 8) & 0xFF]
    body = [0xFE, len(params) + 2, 0x83] + params
    body.append(_chk(body))
    ser.write(bytes([0xFF, 0xFF]) + bytes(body))
    ser.flush()


# ── daemon ────────────────────────────────────────────────────────────────────

class MotorDaemon:
    def __init__(self, port: str, zmq_port: int, pub_port: int | None = None):
        self._port     = port
        self._zmq_port = zmq_port
        self._pub_port = pub_port   # optional broadcast port for RViz/ROS2
        self._ser      = None
        self._curr_ticks = [2048] * 6
        self._lock     = threading.Lock()
        self._pub_sock = None

    def connect(self) -> bool:
        try:
            self._ser = serial.Serial(self._port, 1_000_000, timeout=0.1)
            time.sleep(0.2)
            # Verify: read motor 1
            pos = _read(self._ser, 1, REG_PRESENT_POSITION, 2)
            if pos is None:
                print(f"[daemon] No response from motor 1 on {self._port}")
                return False
            self._curr_ticks = self._read_all()
            print(f"[daemon] Connected on {self._port}  motor1={pos}")
            print(f"[daemon] Current ticks: {dict(zip(MOTOR_NAMES, self._curr_ticks))}")
            return True
        except Exception as e:
            print(f"[daemon] Serial connect error: {e}")
            return False

    def _read_all(self):
        ticks = []
        for mid in MOTOR_IDS:
            v = _read(self._ser, mid, REG_PRESENT_POSITION, 2)
            ticks.append(v if v is not None else 2048)
        with self._lock:
            self._curr_ticks = list(ticks)
        return ticks

    def _gripper_state(self):
        load = _read(self._ser, 6, REG_PRESENT_LOAD,    2)
        curr = _read(self._ser, 6, REG_PRESENT_CURRENT, 2)
        if load is not None and load > 32767:
            load -= 65536
        if curr is not None and curr > 32767:
            curr -= 65536
        return int(load or 0), int(curr or 0)

    def _pub_loop(self, ctx):
        """Background thread: broadcast joint state at 20 Hz on the PUB socket.
        ROS2 driver subscribes here — no conflict with v2 app's REQ/REP."""
        self._pub_sock = ctx.socket(zmq.PUB)
        self._pub_sock.bind(f"tcp://*:{self._pub_port}")
        print(f"[daemon] State broadcast on tcp://localhost:{self._pub_port} (for RViz/ROS2)")
        while True:
            with self._lock:
                ticks = list(self._curr_ticks)
            self._pub_sock.send(msgpack.packb({'ticks': ticks}))
            time.sleep(0.05)   # 20 Hz

    def run(self):
        ctx  = zmq.Context()
        sock = ctx.socket(zmq.REP)
        sock.bind(f"tcp://*:{self._zmq_port}")
        print(f"[daemon] ZMQ REQ/REP on tcp://localhost:{self._zmq_port}")

        if self._pub_port:
            t = threading.Thread(target=self._pub_loop, args=(ctx,), daemon=True)
            t.start()

        print("[daemon] Ready — start the robot app now.")

        while True:
            raw = sock.recv()
            req = msgpack.unpackb(raw, raw=False)
            cmd = req.get('cmd') or req.get(b'cmd')

            if cmd == 0xFF:   # STATUS
                sock.send(msgpack.packb({
                    'status': 0, 'loop_hz': 100,
                    'trajectory_active': False,
                    'current_ticks': self._curr_ticks,
                    'target_ticks':  self._curr_ticks,
                }))

            elif cmd == 0x01:  # WRITE_TICKS
                tl = req.get('ticks') or req.get(b'ticks', [])
                if tl and self._ser:
                    pairs = list(zip(MOTOR_IDS, (int(t) for t in tl)))
                    _sync_write(self._ser, REG_GOAL_POSITION, pairs)
                    with self._lock:
                        self._curr_ticks = [int(t) for t in tl]
                sock.send(msgpack.packb({'status': 0}))

            elif cmd == 0x02:  # READ_TICKS
                ticks = self._read_all() if self._ser else self._curr_ticks
                sock.send(msgpack.packb({'ticks': ticks}))

            elif cmd == 0x03:  # GRIPPER_LOAD
                load, curr = self._gripper_state() if self._ser else (0, 0)
                sock.send(msgpack.packb({
                    'status': 0,
                    'load':     load,
                    'current':  curr,
                    'detected': abs(curr) > 150,
                }))

            else:
                sock.send(msgpack.packb({'status': 1}))


def main():
    ap = argparse.ArgumentParser(description='Python motor daemon for SO-101 on Linux')
    ap.add_argument('--port',     default='/dev/ttyACM0')
    ap.add_argument('--zmq-port', type=int, default=5555)
    ap.add_argument('--pub-port', type=int, default=5556,
                    help='ZMQ PUB port for ROS2/RViz state broadcast (0 = disabled)')
    args = ap.parse_args()

    d = MotorDaemon(args.port, args.zmq_port,
                    pub_port=args.pub_port if args.pub_port else None)
    if not d.connect():
        print(f"[daemon] Failed to connect on {args.port}")
        print("  Check: is the arm powered on? is /dev/ttyACM0 the right port?")
        print("  Run:   ls /dev/ttyACM* /dev/ttyUSB*")
        sys.exit(1)
    d.run()


if __name__ == '__main__':
    main()
