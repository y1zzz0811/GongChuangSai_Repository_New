"""Independent colored-circle pickup preview. Original raw_pickup_vision is unchanged."""
import argparse
from pathlib import Path

import cv2
import numpy as np

from ring_metric import RingMetric
from stacking_vision import StackVision, PIXELS_PER_MM, DIRECTIONS, add_preview_arguments, run, validate_preview_arguments


class CirclePickupVision(StackVision):
    def __init__(self, calibration, grid_mm, height_mm, diameter_mm, search_mm=40,
                 base_diameter_mm=50, base_height_mm=80, roi=None):
        self.calibration = calibration
        width, height = calibration.image_size
        self.roi = tuple(roi or (0, round(height * 130 / 600), width, height - round(height * 130 / 600)))
        x, y, w, h = self.roi
        if not (0 <= x < width and 0 <= y < height and w > 0 and h > 0
                and x + w <= width and y + h <= height):
            raise ValueError('ROI must lie inside the camera image')
        self.height = height_mm
        self.radius = diameter_mm * PIXELS_PER_MM / 2
        camera_position = calibration.camera_position * grid_mm
        self.depth = camera_position[2] - height_mm
        if self.depth <= 0:
            raise ValueError('Target plane must be below the camera')
        self.origin = camera_position[:2] + calibration.zero_on_plane * self.depth
        self.rotation = cv2.Rodrigues(calibration.rotation)[0]
        self.translation = -calibration.rotation @ camera_position
        # Derive a rectangular rectification map from the requested image ROI.
        # Sample its boundary to account for lens distortion, adding rim margin.
        t = np.linspace(0, 1, 65)
        boundary = np.concatenate((np.column_stack((x + t * (w - 1), np.full_like(t, y))),
                                   np.column_stack((x + t * (w - 1), np.full_like(t, y + h - 1))),
                                   np.column_stack((np.full_like(t, x), y + t * (h - 1))),
                                   np.column_stack((np.full_like(t, x + w - 1), y + t * (h - 1)))))
        rays = calibration._plane_points(boundary)
        if rays is None:
            raise ValueError('ROI cannot be projected onto the calibrated plane')
        world = camera_position[:2] + rays * self.depth
        relative = -(world - self.origin)[:, ::-1] * PIXELS_PER_MM
        margin = self.radius * 2 + 8
        low = np.floor(relative.min(axis=0) - margin)
        high = np.ceil(relative.max(axis=0) + margin)
        self.map_center = -low
        map_width, map_height = (high - low + 1).astype(int)
        if map_width * map_height > 12000000:
            raise ValueError('ROI projection is too large; check calibration and heights')
        yy, xx = np.indices((map_height, map_width), dtype=np.float32)
        pixel_map = self.project(np.stack((xx, yy), axis=-1)).astype(np.float32)
        self.visible = ((pixel_map[..., 0] >= 2) & (pixel_map[..., 0] < width - 2)
                        & (pixel_map[..., 1] >= 2) & (pixel_map[..., 1] < height - 2)).astype(np.uint8)
        self.rectification_maps = cv2.convertMaps(pixel_map, None, cv2.CV_16SC2)
        self.detected_circles = []

    def project(self, points):
        points = np.asarray(points)
        xy = self.origin + (self.map_center - points.reshape(-1, 2))[:, ::-1] / PIXELS_PER_MM
        world = np.column_stack((xy, np.full(len(xy), self.height)))
        image = cv2.projectPoints(world, self.rotation, self.translation,
                                  self.calibration.camera_matrix, self.calibration.dist_coeffs)[0]
        return image.reshape(points.shape)

    def detect(self, frame):
        self.detected_circles = []
        if tuple(frame.shape[:2][::-1]) != self.calibration.image_size:
            return "SIZE MISMATCH", None
        # Fit in the physical top plane: a tilted camera sees circles as ellipses.
        rectified = cv2.remap(frame, *self.rectification_maps, cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(cv2.GaussianBlur(rectified, (5, 5), 0), cv2.COLOR_BGR2HSV)
        masks = {
            "RED": cv2.inRange(hsv, (0, 80, 45), (12, 255, 255)) | cv2.inRange(hsv, (168, 80, 45), (179, 255, 255)),
            "YELLOW": cv2.inRange(hsv, (15, 65, 45), (40, 255, 255)),
            "GREEN": cv2.inRange(hsv, (41, 65, 35), (90, 255, 255)),
            "BLUE": cv2.inRange(hsv, (91, 70, 35), (135, 255, 255)),
        }
        kernel = np.ones((3, 3), np.uint8)
        candidates = []
        for color, mask in masks.items():
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask &= self.visible * 255
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < np.pi * (self.radius * 0.60) ** 2 or len(contour) < 20:
                    continue
                perimeter = cv2.arcLength(contour, True)
                if perimeter <= 0 or 4 * np.pi * area / perimeter ** 2 < 0.80:
                    continue
                (cx, cy), (width, height), _ = cv2.fitEllipse(contour)
                radius = (width + height) / 4
                if min(width, height) / max(width, height) < 0.72:
                    continue
                if not self.radius * 0.60 <= radius <= self.radius * 2.0:
                    continue
                center = np.array([cx, cy], dtype=float)
                image_center = self.project(center.reshape(1, 2))[0]
                roi_x, roi_y, roi_w, roi_h = self.roi
                if not (roi_x <= image_center[0] < roi_x + roi_w
                        and roi_y <= image_center[1] < roi_y + roi_h):
                    continue
                # Reject silhouettes touching the image/map boundary.
                border = center + (radius + 2) * DIRECTIONS
                indices = np.rint(border).astype(int)
                if (np.any(indices < 0) or np.any(indices[:, 0] >= self.visible.shape[1]) or np.any(indices[:, 1] >= self.visible.shape[0])
                        or not np.all(self.visible[indices[:, 1], indices[:, 0]])):
                    continue
                radial = np.linalg.norm(contour[:, 0, :] - center, axis=1)
                if np.quantile(np.abs(radial - radius), 0.90) / radius > 0.12:
                    continue
                quality = float(4 * np.pi * area / perimeter ** 2)
                candidates.append((quality, area, color, center, radius))
        if not candidates:
            return "NO CIRCLE", None
        # Keep every detected circle. The legacy single-result interface
        # reports the nearest center to zero; previews show all detections.
        candidates.sort(key=lambda item: np.linalg.norm(item[3] - self.map_center))
        for quality, _, color, center, radius in candidates:
            image_center = self.project(center.reshape(1, 2))[0]
            rim = self.project(center + radius * DIRECTIONS)
            offset = (self.map_center - center)[::-1] / PIXELS_PER_MM
            self.detected_circles.append((color, (image_center, rim, offset, quality)))
        color, primary = self.detected_circles[0]
        status = f"{color} CIRCLE" if len(candidates) == 1 else f"{len(candidates)} CIRCLES"
        return status, primary

    def annotate(self, frame, status, result):
        image = frame.copy()
        x, y, w, h = self.roi
        cv2.rectangle(image, (x, y), (x + w - 1, y + h - 1), (255, 185, 0), 2)
        zero = tuple(np.rint(self.calibration.zero_pixel).astype(int))
        cv2.drawMarker(image, zero, (70, 215, 255), cv2.MARKER_CROSS, 19, 2)
        for thickness, ink in [(3, (0, 0, 0)), (1, (240, 245, 245))]:
            cv2.putText(image, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.57, ink, thickness, cv2.LINE_AA)
        for index, (color, detection) in enumerate(self.detected_circles, 1):
            center, rim, offset, _quality = detection
            point = tuple(np.rint(center).astype(int))
            cv2.polylines(image, [np.rint(rim).astype(np.int32)], True,
                          (0, 255, 0), 2, cv2.LINE_AA)
            cv2.drawMarker(image, point, (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
            cv2.circle(image, point, 4, (0, 255, 0), -1, cv2.LINE_AA)
            label = f"{index} {color} CENTER ({point[0]}, {point[1]})"
            position = (max(5, min(point[0] - 65, image.shape[1] - 290)), max(20, point[1] - 14))
            cv2.putText(image, label, position, cv2.FONT_HERSHEY_SIMPLEX,
                        0.43, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image, label, position, cv2.FONT_HERSHEY_SIMPLEX,
                        0.43, (0, 255, 0), 1, cv2.LINE_AA)
            detail = f"{index} {color}: Fwd {offset[0]:+.1f} Left {offset[1]:+.1f} mm"
            for thickness, ink in [(3, (0, 0, 0)), (1, (240, 245, 245))]:
                cv2.putText(image, detail, (10, 25 + 23 * index),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, ink, thickness, cv2.LINE_AA)
        return image

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_preview_arguments(parser)
    default_calibration = Path(__file__).resolve().with_name("ring_metric_Z184_image_center.json")
    parser.set_defaults(search_mm=40, calibration=default_calibration)
    next(action for action in parser._actions if action.dest == "calibration").required = False
    parser.add_argument("--tray-height-mm", type=float, default=80)
    parser.add_argument("--part-height-mm", type=float, default=60)
    parser.add_argument('--roi', nargs=4, type=int, metavar=('X', 'Y', 'WIDTH', 'HEIGHT'),
                        help='Recognition rectangle in original image pixels')
    args = parser.parse_args()
    validate_preview_arguments(parser, args, max_search_mm=40)
    if not (0 < args.tray_height_mm <= 150 and 0 < args.part_height_mm <= 120):
        parser.error("Invalid tray/part height")
    try:
        vision = CirclePickupVision(RingMetric.load(args.calibration), args.grid_mm,
                                   args.tray_height_mm + args.part_height_mm,
                                   args.diameter_mm, args.search_mm,
                                   args.base_diameter_mm, args.tray_height_mm, roi=args.roi)
        run(args, vision, title="Circle pickup - green: detected center, yellow: reference; Q: quit",
            prefix="pickup_circle")
    except (ValueError, OSError, cv2.error) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
