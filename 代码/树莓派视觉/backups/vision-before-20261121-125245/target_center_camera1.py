"""Camera 1: near-frontal red/green/blue target-centre detection.

Run in Raspberry Pi Thonny. No lens/perspective calibration is applied.
Coordinates refer to the captured image, not the resized preview.
"""
from pathlib import Path
from datetime import datetime
import time
import cv2
import numpy as np

DEVICE = "/dev/video0"
WIDTH, HEIGHT, FPS = 800, 600, 10
from target_center_detector import COLORS, color_masks, detect_target


def text(image, message, point, color=(255, 255, 255), scale=0.5):
    cv2.putText(image, message, point, cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, message, point, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, 1, cv2.LINE_AA)


def cross(image, center, color, size=10):
    p = tuple(np.rint(center).astype(int))
    cv2.drawMarker(image, p, (0, 0, 0), cv2.MARKER_CROSS, size*2, 4)
    cv2.drawMarker(image, p, color, cv2.MARKER_CROSS, size*2, 2)


def main():
    window = "Camera 1 targets | M: masks SPACE: save Q: quit"
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    show_masks = False
    smoothed = {}
    photo_dir = Path.home()/"Pictures"/"target_lab"
    try:
        if not cap.isOpened():
            raise RuntimeError("Cannot open camera 1. Stop the other camera program first.")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(window, 960, 720)
        print("Near-frontal estimate, not a calibrated optical measurement.")
        print("dx positive: right; dy positive: down. M: masks; SPACE: save; Q: quit.")
        last_time = time.monotonic()
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Camera read failed. Check USB connection.")
            h, w = frame.shape[:2]
            origin = np.array([(w-1)/2, (h-1)/2])
            masks = color_masks(frame)
            display = frame.copy()
            cross(display, origin, (255, 255, 255), 16)
            text(display, "IMAGE CENTER", (int(origin[0])+12, int(origin[1])-15), scale=0.4)
            for row, (name, mask) in enumerate(masks.items()):
                target = detect_target(mask)
                color = COLORS[name]
                if target is None:
                    smoothed.pop(name, None)
                    text(display, name+": NOT FOUND", (8, 22+row*23), color)
                    continue
                measured = target["center"]
                previous = smoothed.get(name)
                # Filter jitter only when the new detection is nearby. Never coast on lost data.
                if previous is not None and np.linalg.norm(previous-measured) < target["diameter"]*0.2:
                    alpha = 0.35 if np.linalg.norm(previous-measured) < 2 else 0.75
                    center = alpha*measured+(1-alpha)*previous
                else:
                    center = measured
                smoothed[name] = center
                cross(display, center, color)
                delta = center-origin
                text(display, f"{name}: ({center[0]:.1f},{center[1]:.1f})  dx={delta[0]:+.1f} dy={delta[1]:+.1f}",
                     (8, 22+row*23), color, 0.44)
                px, py = np.rint(center).astype(int)
                text(display, f"{name} CENTER",
                     (max(0, min(px+12, w-160)), max(90, min(py-12, h-30))), color, 0.42)
            now = time.monotonic()
            text(display, f"{w}x{h} | {1/max(now-last_time,0.001):.1f} FPS | NEAR-FRONTAL ESTIMATE",
                 (8, h-12), scale=0.40)
            last_time = now
            if show_masks:
                debug = np.zeros_like(frame)
                for name, mask in masks.items():
                    debug[mask != 0] = COLORS[name]
                display = np.hstack((display, debug))
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord("m"), ord("M")):
                show_masks = not show_masks
            if key == 32:
                photo_dir.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                for suffix, image in [("raw", frame), ("result", display)]:
                    path = photo_dir/f"{stamp}_{suffix}.jpg"
                    if not cv2.imwrite(str(path), image):
                        raise RuntimeError("Failed to save "+str(path))
                print("Saved raw and result:", photo_dir/stamp)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
