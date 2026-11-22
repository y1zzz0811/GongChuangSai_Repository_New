"""Camera 1: near-frontal red/green/blue concentric-ring centre detection.

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
# HSV thresholds use OpenCV's H=0..179, S/V=0..255.
HSV_RANGES = {
    "BLUE": [((95, 65, 25), (135, 255, 255))],
    "GREEN": [((32, 65, 30), (88, 255, 255))],
    "RED": [((0, 75, 35), (12, 255, 255)),
            ((168, 75, 35), (179, 255, 255))],
}
COLORS = {"BLUE": (255, 170, 40), "GREEN": (40, 230, 40), "RED": (40, 70, 255)}
MIN_DIAMETER = 28
MIN_RINGS = 2


def color_masks(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    result = {}
    for name, ranges in HSV_RANGES.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for low, high in ranges:
            mask |= cv2.inRange(hsv, np.array(low), np.array(high))
        # Small closing bridges tiny breaks, without filling ring interiors.
        result[name] = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                      np.ones((3, 3), np.uint8))
    return result


def ellipse_candidate(contour, image_shape):
    if len(contour) < 20 or cv2.contourArea(contour) < 100:
        return None
    x, y, w, h = cv2.boundingRect(contour)
    ih, iw = image_shape
    if x <= 1 or y <= 1 or x+w >= iw-1 or y+h >= ih-1:
        return None  # Do not claim a centre from a clipped target.
    ellipse = cv2.fitEllipse(contour)
    (cx, cy), (a, b), angle = ellipse
    if min(a, b) < MIN_DIAMETER or min(a, b)/max(a, b) < 0.55:
        return None
    points = contour[:, 0, :].astype(np.float64) - (cx, cy)
    theta = np.deg2rad(angle)
    u = (points[:, 0]*np.cos(theta) + points[:, 1]*np.sin(theta))/(a/2)
    v = (-points[:, 0]*np.sin(theta) + points[:, 1]*np.cos(theta))/(b/2)
    residual = np.abs(np.hypot(u, v)-1)
    error = float(np.quantile(residual, 0.8))
    bins = ((np.arctan2(v, u)+np.pi)*36/(2*np.pi)).astype(int) % 36
    coverage = len(np.unique(bins))/36
    if error > 0.09 or coverage < 0.80:
        return None
    return dict(center=np.array([cx, cy]), diameter=np.sqrt(a*b),
                ellipse=ellipse, error=error, coverage=coverage,
                ratio=min(a, b)/max(a, b))


def detect_target(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    candidates = []
    for contour in contours:
        candidate = ellipse_candidate(contour, mask.shape)
        if candidate is not None:
            candidates.append(candidate)
    best = None
    for seed in candidates:
        group = [c for c in candidates
                 if np.linalg.norm(c["center"]-seed["center"]) <= max(3, seed["diameter"]*0.035)
                 and abs(c["ratio"]-seed["ratio"]) < 0.16
                 and 0.40 < c["diameter"]/seed["diameter"] < 2.5]
        # Inner/outer boundaries of one printed stroke are not independent rings.
        rings = []
        for c in sorted(group, key=lambda c: c["diameter"], reverse=True):
            if all(abs(c["diameter"]-r["diameter"]) > max(4, r["diameter"]*0.045)
                   for r in rings):
                rings.append(c)
        if len(rings) < MIN_RINGS:
            continue
        centres = np.array([c["center"] for c in rings])
        center = np.median(centres, axis=0)
        spread = float(np.max(np.linalg.norm(centres-center, axis=1)))
        error = float(np.mean([c["error"] for c in rings]))
        rank = len(rings)*10 - error*20 - spread
        if best is None or rank > best["rank"]:
            best = dict(center=center, rings=rings, rank=rank, spread=spread,
                        diameter=max(c["diameter"] for c in rings))
    return best


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
                    center = 0.4*measured+0.6*previous
                else:
                    center = measured
                smoothed[name] = center
                for ring in target["rings"]:
                    cv2.ellipse(display, ring["ellipse"], color, 1, cv2.LINE_AA)
                cross(display, center, color)
                delta = center-origin
                text(display, f"{name}: ({center[0]:.1f},{center[1]:.1f})  dx={delta[0]:+.1f} dy={delta[1]:+.1f}",
                     (8, 22+row*23), color, 0.44)
                px, py = np.rint(center).astype(int)
                text(display, f"{name} rings={len(target['rings'])}",
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
