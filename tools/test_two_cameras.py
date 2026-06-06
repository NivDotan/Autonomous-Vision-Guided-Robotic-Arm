import cv2
import numpy as np
import time
import threading
import subprocess
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "robot_sam2_app_v2"))
from robot_sam2_app.vl53_sensor import VL53Sensor

VL53_BAUD = 115200


class CameraReader:
    """Background thread that continuously grabs frames, keeping only the latest."""
    def __init__(self, device: str, label: str):
        self.label = label
        self.device = device
        self._frame = None
        self._ok = False
        self._lock = threading.Lock()
        self._cap = cv2.VideoCapture(device)
        if self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            print(f"{label} opened at {device}: {int(self._cap.get(3))}x{int(self._cap.get(4))}")
        else:
            print(f"{label} ({device}) NOT available")
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while True:
            ret, frame = self._cap.read()
            with self._lock:
                self._ok = ret
                if ret:
                    self._frame = frame

    def read(self):
        with self._lock:
            return self._ok, (self._frame.copy() if self._frame is not None else None)

    def release(self):
        self._cap.release()


def find_vl53_port() -> str:
    import glob, serial
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    for port in candidates:
        try:
            s = serial.Serial(port, VL53_BAUD, timeout=0.5)
            time.sleep(0.3)
            for _ in range(10):
                if "distance" in s.readline().decode("utf-8", errors="ignore").lower():
                    s.close()
                    print(f"VL53 found on {port}")
                    return port
            s.close()
        except Exception:
            pass
    print("VL53 not found, tried:", candidates)
    return candidates[0] if candidates else "/dev/ttyUSB0"


def find_camera_device(name_fragment: str) -> str | None:
    try:
        out = subprocess.check_output(["v4l2-ctl", "--list-devices"], stderr=subprocess.STDOUT).decode()
    except Exception:
        return None
    current_name = ""
    for line in out.splitlines():
        line = line.strip()
        if not line.startswith("/dev/"):
            current_name = line
        elif name_fragment.lower() in current_name.lower():
            return line
    return None


def blank(label: str) -> np.ndarray:
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, label, (60, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 200), 2)
    return img


# Auto-detect cameras by name
dev0 = find_camera_device("LifeCam") or "/dev/video1"
dev2 = find_camera_device("Arducam") or "/dev/video4"
print(f"Detected: LifeCam={dev0}  Arducam={dev2}")

sensor = VL53Sensor(find_vl53_port(), VL53_BAUD)
sensor.connect()

cam0 = CameraReader(dev0, "LifeCam")
cam2 = CameraReader(dev2, "Arducam")

print("Press Q to quit")

last_dist_mm: int | None = None
last_sensor_read = 0.0

while True:
    now = time.monotonic()
    if now - last_sensor_read >= 2.0:
        last_dist_mm = sensor.distance_mm
        last_sensor_read = now
        print(f"[VL53] {last_dist_mm} mm" if last_dist_mm else "[VL53] no reading")

    ret0, frame0 = cam0.read()
    ret2, frame2 = cam2.read()

    f0 = frame0 if ret0 and frame0 is not None else blank("LifeCam OFFLINE")
    f2 = frame2 if ret2 and frame2 is not None else blank("Arducam OFFLINE")

    cv2.putText(f0, f"LifeCam ({dev0})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(f2, f"Arducam ({dev2})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    dist_text = f"VL53: {last_dist_mm} mm" if last_dist_mm is not None else "VL53: ---"
    cv2.putText(f0, dist_text, (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
    cv2.putText(f2, dist_text, (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

    if f0.shape[0] != f2.shape[0]:
        f2 = cv2.resize(f2, (f2.shape[1], f0.shape[0]))

    cv2.imshow("Camera Test -- Q to quit", cv2.hconcat([f0, f2]))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam0.release()
cam2.release()
sensor.disconnect()
cv2.destroyAllWindows()
