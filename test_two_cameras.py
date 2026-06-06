import cv2
import numpy as np

def open_camera(index):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # Warmup: drain a few frames
        for _ in range(5):
            cap.read()
        print(f"Camera {index} opened: {int(cap.get(3))}x{int(cap.get(4))}")
    else:
        print(f"Camera {index} NOT available")
    return cap

cap0 = open_camera(0)  # Microsoft LifeCam VX-5000
cap2 = open_camera(2)  # Arducam IMX323

print("Press Q to quit")

def blank(label):
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, label, (80, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 200), 2)
    return img

while True:
    ret0, frame0 = cap0.read()
    ret2, frame2 = cap2.read()

    f0 = frame0 if ret0 else blank("Camera 0 OFFLINE")
    f2 = frame2 if ret2 else blank("Camera 2 OFFLINE")

    cv2.putText(f0, "Cam 0: LifeCam", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(f2, "Cam 2: Arducam", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    h0 = f0.shape[0]
    h2 = f2.shape[0]
    if h0 != h2:
        f2 = cv2.resize(f2, (f2.shape[1], h0))

    combined = cv2.hconcat([f0, f2])
    cv2.imshow("Camera Test -- Q to quit", combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap0.release()
cap2.release()
cv2.destroyAllWindows()
