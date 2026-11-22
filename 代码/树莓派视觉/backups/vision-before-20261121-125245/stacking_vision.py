"""Standalone measurement of a 30 mm material top, with no motor output."""

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from ring_metric import RingMetric


PIXELS_PER_MM = 3
MIN_EDGE_SCORE = 1.2
DEFAULT_TOP_SEARCH_MM = 30
MAX_SEARCH_MM = 100
PREVIEW = "Stacking vision - Space: save raw, Q: quit"
ANGLES = np.arange(72) * (2 * np.pi / 72)
DIRECTIONS = np.column_stack((np.cos(ANGLES), np.sin(ANGLES))).astype(np.float32)


class StackVision:
    def __init__(self, calibration, grid_mm, height_mm, diameter_mm, zero_pixel, search_mm,
                 base_diameter_mm=50, base_height_mm=0, min_top_score=MIN_EDGE_SCORE):
        self.calibration = calibration
        self.height = height_mm
        self.radius = diameter_mm * PIXELS_PER_MM / 2
        self.search = search_mm * PIXELS_PER_MM
        self.min_top_score = min_top_score
        camera_position = calibration.camera_position * grid_mm
        self.depth = camera_position[2] - height_mm
        if self.depth <= 0 or calibration.zero_on_plane is None:
            raise ValueError("需要含地面对点零位的有效相机标定，且顶面必须低于相机")

        self.origin = camera_position[:2] + calibration.zero_on_plane * camera_position[2]
        self.has_top_zero = zero_pixel is not None
        if self.has_top_zero:
            zero = np.asarray(zero_pixel, dtype=float)
            if (zero.shape != (2,) or not np.isfinite(zero).all()
                    or np.any(zero < 0) or np.any(zero >= calibration.image_size)):
                raise ValueError("顶面零点超出图像")
            ray = calibration._plane_points(zero.reshape(1, 2))
            if ray is None:
                raise ValueError("顶面零点无法投影")
            self.origin = camera_position[:2] + ray[0] * self.depth

        self.rotation = cv2.Rodrigues(calibration.rotation)[0]
        self.translation = -calibration.rotation @ camera_position
        if not 0 <= base_height_mm < height_mm:
            raise ValueError("支撑面高度必须低于物料顶面")
        # Both outlines are sampled on the rectified top plane. The base is
        # farther from the camera, so its projected radius and center shrink
        # toward the camera's vertical projection by this depth ratio.
        self.base_scale = float(self.depth / (camera_position[2] - base_height_mm))
        self.base_radius = base_diameter_mm * self.base_scale * PIXELS_PER_MM / 2
        self.base_shift = ((self.origin - camera_position[:2])[::-1] * PIXELS_PER_MM
                           * (1 - self.base_scale)).astype(np.float32)
        self.mid = int(np.ceil(self.search + max(self.radius, self.base_radius) * 1.15
                               + np.max(np.abs(self.base_shift)) + 8))
        y, x = np.indices((2 * self.mid + 1,) * 2, dtype=np.float32)
        self.pixel_map = self.project(np.stack((x, y), axis=-1)).astype(np.float32)
        width, height = calibration.image_size
        self.visible = ((self.pixel_map[..., 0] >= 2) & (self.pixel_map[..., 0] < width - 2)
                        & (self.pixel_map[..., 1] >= 2) & (self.pixel_map[..., 1] < height - 2))
        self.visible = self.visible.astype(np.uint8)
        axis = np.linspace(-self.search, self.search, int(np.ceil(2 * self.search / 5)) + 1,
                           dtype=np.float32)
        x, y = np.meshgrid(axis, axis)
        self.centers = np.column_stack((x.ravel(), y.ravel()))
        base_centers = (self.mid + self.centers * self.base_scale + self.base_shift).astype(np.float32)
        base_radii = self.base_radius + np.array([-3, 0, 3], dtype=np.float32)
        bases = np.column_stack((np.repeat(base_centers, 3, axis=0),
                                 np.tile(base_radii, len(self.centers))))
        self.base_batches = np.array_split(bases, int(np.ceil(len(bases) / 1500)))
        spacing = float(axis[1] - axis[0])
        kernel_axis = np.arange(-int(np.ceil(11 / spacing)), int(np.ceil(11 / spacing)) + 1)
        self.base_neighborhood = (np.hypot(kernel_axis[:, None], kernel_axis) * spacing <= 11).astype(np.uint8)
        self.grid_shape = x.shape

    def project(self, points):
        shape = points.shape
        xy = self.origin + (self.mid - points.reshape(-1, 2)[:, ::-1]) / PIXELS_PER_MM
        world = np.column_stack((xy, np.full(len(xy), self.height)))
        image = cv2.projectPoints(world, self.rotation, self.translation,
                                  self.calibration.camera_matrix, self.calibration.dist_coeffs)[0]
        return image.reshape(shape)

    def score(self, lab, candidates, base=False):
        """Score contrast across the rim with consistent polarity around the circle."""
        locations = candidates[:, None, :2]
        radii = candidates[:, None, 2:3]
        inside = locations + (radii - 3) * DIRECTIONS
        outside = locations + (radii + 3) * DIRECTIONS
        a = cv2.remap(lab, inside, None, cv2.INTER_LINEAR)
        b = cv2.remap(lab, outside, None, cv2.INTER_LINEAR)
        visible = (cv2.remap(self.visible, inside, None, cv2.INTER_NEAREST) &
                   cv2.remap(self.visible, outside, None, cv2.INTER_NEAREST))
        contrast = (a - b) * visible[..., None]
        # The same search applies to dark and coloured parts. The channels
        # supply edges, and never assign a material colour or a target ID.
        signed = contrast * np.sign(np.median(contrast, axis=1))[:, None, :]
        support = signed >= 4.0
        coverage = support.mean(axis=1)
        sectors = support.reshape(len(candidates), 8, 9, 3).sum(axis=2)
        broad = (sectors >= 3).sum(axis=1) >= (7 if base else 5)
        strength = np.sqrt(np.clip(signed, 0, 90)).mean(axis=1) * np.sqrt(18)
        scores = np.where((coverage >= (0.65 if base else 0.48)) & broad, strength * coverage, 0)
        score = scores.max(axis=1)
        # A top is a filled disk. Reject circles whose interior cuts across
        # the printed floor rings or the gripper, even if the rim scores well.
        index = np.flatnonzero(score >= MIN_EDGE_SCORE)
        samples = locations[index] + (0.48 if base else 0.72) * radii[index] * DIRECTIONS[::3]
        if len(samples):
            interior = cv2.remap(lab, samples, None, cv2.INTER_LINEAR)
            median = np.median(interior, axis=1, keepdims=True)
            spread = np.abs(interior - median)
            variation = np.mean(np.minimum(spread / [35, 12, 12], 2), axis=(1, 2))
            score[index] *= np.clip(1 - variation, 0, 1) ** 3
        return score

    def detect(self, frame):
        if frame.shape[1::-1] != self.calibration.image_size:
            return "SIZE MISMATCH", None
        image = cv2.remap(frame, self.pixel_map, None, cv2.INTER_LINEAR)
        lab = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2LAB), (5, 5), 1.2)
        lab = lab.astype(np.float32)
        # Coarse grid, then refine the strongest, spatially distinct maxima.
        base_scores = np.concatenate([self.score(lab, batch, base=True)
                                      for batch in self.base_batches])
        base_scores = base_scores.reshape(-1, 3).max(axis=1)
        has_base = base_scores >= max(2.2, base_scores.max() * 0.75)
        base_centers = self.centers[has_base]
        if not len(base_centers):
            return "NO MATERIAL OUTLINE", None
        near_base = cv2.dilate(has_base.reshape(self.grid_shape).astype(np.uint8),
                              self.base_neighborhood).ravel().astype(bool)
        centers = self.centers[near_base]
        radii = self.radius + np.arange(-3, 4, dtype=np.float32)
        candidates = np.column_stack((np.repeat(centers + self.mid, len(radii), axis=0),
                                      np.tile(radii, len(centers))))
        scores = np.concatenate([self.score(lab, batch) for batch in np.array_split(
            candidates, max(1, int(np.ceil(len(candidates) / 1500))))])
        peaks = []
        for _ in range(5):
            index = int(scores.argmax())
            if scores[index] < MIN_EDGE_SCORE:
                break
            seed = candidates[index]
            delta = np.arange(-4, 4.1, 1, dtype=np.float32)
            dx, dy, dr = np.meshgrid(delta, delta, np.arange(-2, 3, dtype=np.float32))
            local = seed + np.column_stack((dx.ravel(), dy.ravel(), dr.ravel()))
            eligible = ((np.abs(local[:, :2] - self.mid) <= self.search + 0.001).all(axis=1)
                        & (np.abs(local[:, 2] - self.radius) <= 3)
                        & (np.linalg.norm(local[:, None, :2] - self.mid - base_centers,
                                          axis=2).min(axis=1) <= 11))
            local = local[eligible]
            scores[np.linalg.norm(candidates[:, :2] - seed[:2], axis=1) < self.radius * 0.60] = 0
            if not len(local):
                continue
            refined = self.score(lab, local)
            best = int(refined.argmax())
            if refined[best] >= MIN_EDGE_SCORE:
                peaks.append((float(refined[best]), local[best]))

        if not peaks:
            return "NO RELIABLE TOP EDGE", None
        peaks.sort(key=lambda peak: peak[0], reverse=True)
        quality, circle = peaks[0]
        if quality < self.min_top_score:
            return "WEAK TOP EDGE", None
        if np.any(np.abs(circle[:2] - self.mid) >= self.search - 3):
            return "NEAR SEARCH LIMIT - RECENTER", None
        for other_quality, other in peaks[1:]:
            if (other_quality >= 0.85 * quality
                    and np.linalg.norm(circle[:2] - other[:2]) > 2 * PIXELS_PER_MM):
                return "AMBIGUOUS TOP EDGE", None
        center = self.project(circle[:2].reshape(1, 2))[0]
        rim = self.project(circle[:2] + circle[2] * DIRECTIONS)
        offset = (self.mid - circle[1::-1]) / PIXELS_PER_MM
        return "TOP EDGE" if self.has_top_zero else "ESTIMATE - ZERO UNVERIFIED", (
            center, rim, offset, quality)

    def annotate(self, frame, status, result):
        image = frame.copy()
        if status != "SIZE MISMATCH":
            d = self.search
            corners = np.array([[-d, -d], [d, -d], [d, d], [-d, d]]) + self.mid
            cv2.polylines(image, [self.project(corners).round().astype(np.int32)], True,
                          (220, 185, 70), 1, cv2.LINE_AA)
            zero = self.project(np.full((1, 2), self.mid))[0].round().astype(int)
            cv2.drawMarker(image, tuple(zero), (70, 215, 255), cv2.MARKER_CROSS, 19, 2)
        lines = status.split(" | ", 1)
        if result is not None:
            center, rim, (forward, left), quality = result
            cv2.polylines(image, [rim.round().astype(np.int32)], True, (80, 255, 80), 2, cv2.LINE_AA)
            cv2.drawMarker(image, tuple(center.round().astype(int)), (80, 255, 80),
                           cv2.MARKER_CROSS, 16, 2)
            lines += [f"u={center[0]:.1f} v={center[1]:.1f}  edge={quality:.1f}",
                      f"Fwd {forward:+.1f} mm  Left {left:+.1f} mm"]
        for i, line in enumerate(lines):
            position = (10, 25 + i * 24)
            cv2.putText(image, line, position, cv2.FONT_HERSHEY_SIMPLEX, 0.57,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(image, line, position, cv2.FONT_HERSHEY_SIMPLEX, 0.57,
                        (240, 245, 245), 1, cv2.LINE_AA)
        return image


def read_image(path):
    frame = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError(f"无法读取图像 {path}")
    return frame


def write_image(path, frame):
    ok, data = cv2.imencode(path.suffix, frame)
    if not ok:
        raise ValueError(f"无法编码图像 {path}")
    path.write_bytes(data.tobytes())


def run(args, vision, observe=None, title=PREVIEW, prefix="stack"):
    if args.image:
        frame = read_image(args.image)
        status, result = vision.detect(frame)
        data = {"status": status, "top_zero_supplied": vision.has_top_zero}
        if result is not None:
            center, _, offset, quality = result
            data.update(center_px=center.tolist(), offset_mm=offset.tolist(), edge_score=quality)
        print(json.dumps(data))
        if args.output:
            write_image(args.output, vision.annotate(frame, status, result))
        return

    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    try:
        if not capture.isOpened():
            raise ValueError("无法打开相机 2，请先关闭占用它的双摄程序")
        width, height = vision.calibration.image_size
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, 10)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cv2.namedWindow(title)

        def print_pixel(event, u, v, _flags, _userdata):
            if event == cv2.EVENT_LBUTTONDOWN:
                print(f"pixel: {u} {v}", flush=True)

        cv2.setMouseCallback(title, print_pixel)
        while True:
            ok, frame = capture.read()
            if not ok:
                raise ValueError("无法读取相机 2 画面")
            captured_at = time.monotonic()
            status, result = vision.detect(frame)
            if observe is not None:
                status = observe(status, result, captured_at)
            cv2.imshow(title, vision.annotate(frame, status, result))
            key = cv2.waitKey(1) & 0xff
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                path = Path(f"{prefix}_raw_{time.time_ns()}.png")
                write_image(path, frame)
                print(path.resolve(), flush=True)
    finally:
        capture.release()
        cv2.destroyAllWindows()


def add_preview_arguments(parser):
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--grid-mm", type=float, default=10.0, help="地面标定文件每坐标单位的毫米数")
    parser.add_argument("--diameter-mm", type=float, default=30.0)
    parser.add_argument("--base-diameter-mm", type=float, default=50.0)
    parser.add_argument("--zero", nargs=2, type=float, metavar=("U", "V"), help="顶面实测零点像素")
    parser.add_argument("--search-mm", type=float, default=DEFAULT_TOP_SEARCH_MM,
                        help="对点零位两侧各搜索多少毫米（默认 %(default)s）")
    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument("--image", type=Path, help="回放一张相机原图")
    parser.add_argument("--output", type=Path, help="保存回放标注图")


def validate_preview_arguments(parser, args, max_search_mm=MAX_SEARCH_MM):
    if not (0 < args.grid_mm <= 100
            and 10 <= args.diameter_mm < args.base_diameter_mm <= 100 and 5 <= args.search_mm <= max_search_mm
            and (args.zero is None or np.isfinite(args.zero).all())):
        parser.error("高度、环径、搜索范围或零点无效")
    if args.output and not args.image:
        parser.error("--output 需与 --image 一起使用")
    if args.output and args.output.resolve() == args.image.resolve():
        parser.error("标注图路径须与原图不同")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_preview_arguments(parser)
    parser.add_argument("--height-mm", type=float, default=60.0)
    args = parser.parse_args()
    validate_preview_arguments(parser, args)
    if not 0 < args.height_mm <= 200:
        parser.error("顶面高度无效")
    try:
        vision = StackVision(RingMetric.load(args.calibration), args.grid_mm, args.height_mm,
                             args.diameter_mm, args.zero, args.search_mm, args.base_diameter_mm)
        run(args, vision)
    except (ValueError, OSError, cv2.error) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
