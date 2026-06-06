import cv2


def main():
    cap0 = cv2.VideoCapture(0)
    cap1 = cv2.VideoCapture(1)

    if not cap0.isOpened():
        print("Camera 0 could not be opened.")
    if not cap1.isOpened():
        print("Camera 1 could not be opened.")

    while True:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()

        if ret0:
            cv2.imshow("Camera 0", frame0)
        if ret1:
            cv2.imshow("Camera 1", frame1)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
