import cv2
from pathlib import Path
from datetime import datetime

photo_dir = Path.home() / "Pictures" / "camera_lab"
photo_dir.mkdir(parents=True, exist_ok=True)
window = "USB Camera - SPACE: photo, Q: quit"
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

try:
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera. Check the USB connection.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    print("Click the camera window. SPACE: photo; Q or Esc: quit.")
    print("Photo folder:", photo_dir)
    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Cannot read camera frame.")
        cv2.imshow(window, frame)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            break
        if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key == 32:
            path = photo_dir / (datetime.now().strftime("photo_%Y%m%d_%H%M%S_%f") + ".jpg")
            if not cv2.imwrite(str(path), frame):
                raise RuntimeError("Could not save photo.")
            print("Saved:", path)
finally:
    cap.release()
    cv2.destroyAllWindows()
