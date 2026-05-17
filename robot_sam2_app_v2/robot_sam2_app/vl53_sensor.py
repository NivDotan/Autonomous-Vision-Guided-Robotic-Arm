from __future__ import annotations

import threading
import time
from collections import deque


class VL53Sensor:
    """Reads distance (mm) from a VL53L1X via ESP32 serial in a background thread.

    ESP32 firmware sends lines like: "Distance: 243 mm"
    """

    def __init__(self, port: str, baud: int = 115200):
        self._port   = port
        self._baud   = baud
        self._dist_mm: int | None = None
        self._buffer: deque[int] = deque(maxlen=10)
        self._lock   = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop   = threading.Event()
        self.connected = False

    @property
    def distance_mm(self) -> int | None:
        with self._lock:
            return self._dist_mm

    def is_reading_valid(self, max_jump: int) -> bool:
        """True if the last step was smooth (no sudden jump) — used to gate each elbow move."""
        with self._lock:
            buf = list(self._buffer)
        if len(buf) < 2:
            return False
        return abs(buf[-1] - buf[-2]) <= max_jump

    def is_stable_and_close(self, threshold_mm: int, stable_window: int, max_jump: int) -> bool:
        """Return True only when the last 3 readings are stable, not jumpy, and within threshold."""
        with self._lock:
            buf = list(self._buffer)
        if len(buf) < 3:
            return False
        last3 = buf[-3:]
        # Check 1: last 3 readings are close to each other
        if max(last3) - min(last3) > stable_window:
            return False
        # Check 2: no sudden jump anywhere in the buffer (rejects occlusions/noise)
        for a, b in zip(buf, buf[1:]):
            if abs(b - a) > max_jump:
                return False
        # Check 3: average of last 3 is within grip distance
        return sum(last3) / 3 <= threshold_mm

    def connect(self) -> bool:
        try:
            import serial
            self._serial = serial.Serial(self._port, self._baud, timeout=1)
            self._stop.clear()
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self.connected = True
            print(f"VL53 sensor connected on {self._port} @ {self._baud}")
            return True
        except Exception as exc:
            print(f"VL53 sensor unavailable ({self._port}): {exc}")
            self.connected = False
            return False

    def disconnect(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        try:
            self._serial.close()
        except Exception:
            pass
        self.connected = False

    def _read_loop(self) -> None:
        last_print = 0.0
        while not self._stop.is_set():
            try:
                line = self._serial.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                val = int(line.lower().replace("distance:", "").replace("mm", "").strip())
                if val <= 0:
                    continue  # ignore invalid startup readings
                with self._lock:
                    self._dist_mm = val
                    self._buffer.append(val)
                now = time.monotonic()
                if now - last_print >= 1.0:
                    print(f"[VL53] {val} mm")
                    last_print = now
            except ValueError:
                pass
            except Exception as exc:
                print(f"[VL53 error] {exc}")
                time.sleep(0.1)
