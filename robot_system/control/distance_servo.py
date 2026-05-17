"""
Step 10 — Distance-controlled final approach using a VL53L1X sensor.

After the visual servo centres the object (object_reached=True), this module
takes over elbow control to close the remaining gap, then validates stable
close-range readings before signalling "grip now".

Architecture
------------
DistanceSensor          — protocol (read() -> mm | None)
MockDistanceSensor      — injects a scripted sequence for tests / dry-run
SerialDistanceSensor    — reads from ESP32 over serial ("Distance: NNN mm")
DistanceServo           — rolling-buffer stability validator + P-controller

Integration with VisualServo
----------------------------
    # After visual_servo.step() returns state.object_reached=True:
    dist_state = dist_servo.step(adapter)
    if dist_state.should_grip:
        adapter.close_gripper()

Usage (dry-run, no hardware):
    from control.distance_servo import DistanceServo, MockDistanceSensor, DistanceConfig

    sensor = MockDistanceSensor([400, 300, 200, 100, 80, 70, 68, 66, 65])
    servo  = DistanceServo(sensor)

    from control.lerobot_adapter import LeRobotAdapter
    adapter = LeRobotAdapter(dry_run=True)

    while True:
        state = servo.step(adapter)
        print(state)
        if state.should_grip:
            adapter.close_gripper()
            break
"""

from __future__ import annotations

import time
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, Protocol, Sequence

from control.lerobot_adapter import LeRobotAdapter


# ------------------------------------------------------------------
# Sensor protocol + implementations
# ------------------------------------------------------------------

class DistanceSensor(Protocol):
    """Anything that returns a distance in millimetres (or None on error)."""

    def read(self) -> Optional[int]:
        ...

    def close(self) -> None:
        ...


class MockDistanceSensor:
    """
    Replay a scripted sequence of readings.

    Useful for unit tests and offline demos.  After the sequence is exhausted,
    read() returns the last value indefinitely (simulates holding still).
    """

    def __init__(self, readings: Sequence[int], delay_s: float = 0.0) -> None:
        self._readings = list(readings)
        self._idx      = 0
        self._delay    = delay_s

    def read(self) -> Optional[int]:
        if self._delay:
            time.sleep(self._delay)
        if not self._readings:
            return None
        val = self._readings[min(self._idx, len(self._readings) - 1)]
        self._idx += 1
        return val

    def close(self) -> None:
        pass


class SerialDistanceSensor:
    """
    Background-thread reader for an ESP32 that sends "Distance: NNN mm" lines.

    Args:
        port:     serial port, e.g. "COM3" or "/dev/ttyUSB0"
        baud:     baud rate (default 115200 to match ESP32 sketch)
        timeout:  serial read timeout in seconds
    """

    def __init__(self, port: str, baud: int = 115_200, timeout: float = 1.0) -> None:
        import serial  # type: ignore[import-untyped]
        self._ser   = serial.Serial(port, baud, timeout=timeout)
        self._lock  = threading.Lock()
        self._latest: Optional[int] = None
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            try:
                raw = self._ser.readline().decode(errors="ignore").strip().lower()
                if "distance:" not in raw:
                    continue
                val = int(raw.replace("distance:", "").replace("mm", "").strip())
                if val <= 0:
                    continue
                with self._lock:
                    self._latest = val
            except (ValueError, OSError):
                pass

    def read(self) -> Optional[int]:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._running = False
        try:
            self._ser.close()
        except OSError:
            pass


# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------

@dataclass
class DistanceConfig:
    """Tuning parameters for the distance servo."""

    # Stability validation (mirrors robot_sam2_app config)
    buffer_size:    int   = 10    # rolling window length
    stable_n:       int   = 3     # consecutive readings that must be stable
    stable_window:  int   = 15    # max spread (mm) across last N readings
    max_jump:       int   = 30    # max change (mm) between any two consecutive readings
    grip_dist_mm:   int   = 75    # distance (mm) at which to attempt grip

    # Approach P-controller
    Kp_elbow:       float = 0.002   # elbow delta (rad) per mm of distance error
    elbow_limit:    tuple[float, float] = (-1.5, 1.5)   # rad

    # After how many consecutive stable-and-close readings to signal grip
    grip_confirm_n: int = 3


# ------------------------------------------------------------------
# State returned each cycle
# ------------------------------------------------------------------

@dataclass
class DistanceState:
    """Output of DistanceServo.step()."""
    dist_mm:        Optional[int]   # latest sensor reading (None = no data)
    is_stable:      bool            # last N readings are stable
    is_close:       bool            # average of last N ≤ grip_dist_mm
    should_grip:    bool            # stable + close + confirmed N times
    elbow_delta:    float           # correction applied this step (rad)
    confirm_count:  int             # consecutive cycles where grip criteria met

    def __str__(self) -> str:
        d = f"{self.dist_mm} mm" if self.dist_mm is not None else "None"
        return (
            f"DistState  dist={d}  stable={self.is_stable}  "
            f"close={self.is_close}  grip={self.should_grip}  "
            f"confirm={self.confirm_count}"
        )


# ------------------------------------------------------------------
# Controller
# ------------------------------------------------------------------

class DistanceServo:
    """
    Final-approach controller driven by a distance sensor.

    Call step() once per control cycle.  The controller:
      1. Reads the sensor.
      2. Appends to a rolling buffer.
      3. Checks stability (spread + jump + range).
      4. Drives elbow toward object while dist > grip_dist.
      5. Returns should_grip=True after grip_confirm_n consecutive stable cycles.
    """

    def __init__(
        self,
        sensor: DistanceSensor,
        config: DistanceConfig = DistanceConfig(),
    ) -> None:
        self.sensor  = sensor
        self.cfg     = config
        self._buf: deque[int] = deque(maxlen=config.buffer_size)
        self._confirm = 0

    def reset(self) -> None:
        self._buf.clear()
        self._confirm = 0

    def step(self, adapter: LeRobotAdapter) -> DistanceState:
        """Run one control cycle."""
        dist = self.sensor.read()

        if dist is not None:
            self._buf.append(dist)

        is_stable, is_close = self._validate()

        # Approach: drive elbow while we have data but aren't close yet
        elbow_delta = 0.0
        if dist is not None and not is_close:
            err         = dist - self.cfg.grip_dist_mm   # positive = too far
            elbow_delta = self.cfg.Kp_elbow * err
            curr_elbow  = adapter.observe().get("elbow_flex", 0.0)
            new_elbow   = _clamp(curr_elbow + elbow_delta, self.cfg.elbow_limit)
            adapter.move_joints({"elbow_flex": new_elbow}, blocking=False)
            elbow_delta  = new_elbow - curr_elbow   # actual delta after clamping

        # Confirmation counter
        if is_stable and is_close:
            self._confirm += 1
        else:
            self._confirm = 0

        should_grip = self._confirm >= self.cfg.grip_confirm_n

        return DistanceState(
            dist_mm      = dist,
            is_stable    = is_stable,
            is_close     = is_close,
            should_grip  = should_grip,
            elbow_delta  = elbow_delta,
            confirm_count= self._confirm,
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _validate(self) -> tuple[bool, bool]:
        """Return (is_stable, is_close)."""
        buf = list(self._buf)
        n   = self.cfg.stable_n

        if len(buf) < n:
            return False, False

        last_n = buf[-n:]

        # Stability: spread of last N readings
        if max(last_n) - min(last_n) > self.cfg.stable_window:
            return False, False

        # No sudden jump anywhere in the full buffer
        for a, b in zip(buf, buf[1:]):
            if abs(b - a) > self.cfg.max_jump:
                return False, False

        # Range check
        avg = sum(last_n) / n
        return True, avg <= self.cfg.grip_dist_mm


# ------------------------------------------------------------------
# Utility
# ------------------------------------------------------------------

def _clamp(value: float, limits: tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))
