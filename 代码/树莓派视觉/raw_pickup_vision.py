"""Camera 2 preview of material on the raw-material turntable."""

import argparse
from collections import deque

import cv2
import numpy as np

from ring_metric import RingMetric
from stacking_vision import StackVision, add_preview_arguments, run, validate_preview_arguments


RAW_MIN_TOP_SCORE = 5.0


class MotionObservation:
    """Report a stop only after continuous observations of motion and rest."""

    def __init__(self, pickup_offset=None, window_mm=8):
        self.pickup_offset = pickup_offset
        self.window_mm = window_mm
        self.samples = deque()
        self.saw_motion = False
        self.last_motion = None

    def update(self, offset, captured_at):
        if (offset is None or (self.samples and (
                captured_at <= self.samples[-1][0]
                or captured_at - self.samples[-1][0] > 0.5
                or np.linalg.norm(offset - self.samples[-1][1]) > 20))):
            self.samples.clear()
            self.saw_motion = False
            self.last_motion = None
        if offset is None:
            return "TARGET LOST"
        self.samples.append((captured_at, offset))
        while len(self.samples) > 1 and self.samples[1][0] <= captured_at - 1.0:
            self.samples.popleft()
        recent = np.array([point for _, point in self.samples])
        spread = float(np.max(np.ptp(recent, axis=0)))
        if len(recent) >= 3:
            first_step = recent[-2] - recent[-3]
            second_step = recent[-1] - recent[-2]
            if (np.linalg.norm(first_step) >= 1.5 and np.linalg.norm(second_step) >= 1.5
                    and first_step @ second_step > 0 and np.linalg.norm(first_step + second_step) >= 4):
                self.saw_motion = True
                self.last_motion = captured_at
        if not self.saw_motion:
            return "WAIT FOR MOTION"
        if (len(self.samples) >= 4 and captured_at - self.samples[0][0] >= 1.0
                and spread <= 1.0):
            return "STOP OBSERVED"
        return "TRACKING"

    def preview(self, status, result, captured_at):
        motion = self.update(None if result is None else result[2], captured_at)
        if result is None:
            return status
        if self.pickup_offset is None:
            return f"{status} | {motion} | WINDOW NOT SET"
        # The window is circular on the physical top plane and is taught
        # separately from the gripper zero.
        distance = np.linalg.norm(result[2] - self.pickup_offset)
        window = "IN WINDOW" if distance <= self.window_mm else "OUTSIDE WINDOW"
        return f"{status} | {motion} | {window}"


class RawStopSession:
    """Carry only confirmed stop evidence across corrections, never old frames."""

    # Allow odometry/rail correction error when checking the same physical
    # stop location; the taught pickup window still applies independently.
    MAX_STATION_SHIFT_MM = 10.0

    def __init__(self):
        self.confirmed = None

    def begin(self, target, displacement, continuation, now, raw):
        previous, self.confirmed = self.confirmed, None
        tracker = MotionObservation()
        if (not continuation or previous is None or raw is None
                or getattr(raw, 'static_test', False)):
            return tracker
        old_target, motion_at, station = previous
        if (target != old_target or not 0 <= now - motion_at
                or raw.dwell - (now - motion_at) < raw.action + 0.35):
            return tracker
        return _RawResumeObservation(motion_at, station, displacement,
                                     self.MAX_STATION_SHIFT_MM)

    def remember(self, target, displacement, offset, tracker):
        if tracker.last_motion is not None:
            station = (tracker.station.copy() if getattr(tracker, 'inherited_motion_at', None) is not None
                       else np.asarray(offset, dtype=float) + np.asarray(displacement, dtype=float))
            self.confirmed = (target, tracker.last_motion, station)


class _RawResumeObservation(MotionObservation):
    def __init__(self, motion_at, station, displacement, max_shift):
        super().__init__()
        self.saw_motion = True
        self.last_motion = motion_at
        self.station = np.asarray(station, dtype=float)
        self.displacement = np.asarray(displacement, dtype=float)
        self.max_shift = max_shift
        self.inherited_motion_at = motion_at

    def update(self, offset, captured_at):
        if self.inherited_motion_at is not None:
            if (offset is None or not np.isfinite(offset).all()
                    or np.linalg.norm(offset + self.displacement - self.station) > self.max_shift):
                # Loss or a different physical position invalidates inherited
                # evidence. Require a newly observed movement and stop.
                super().update(None, captured_at)
                self.inherited_motion_at = None
                if offset is None or not np.isfinite(offset).all():
                    return 'TARGET LOST'
        state = super().update(offset, captured_at)
        if self.last_motion != self.inherited_motion_at:
            self.inherited_motion_at = None
        return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_preview_arguments(parser)
    parser.set_defaults(search_mm=40)
    parser.add_argument("--tray-height-mm", type=float, default=80)
    parser.add_argument("--part-height-mm", type=float, default=60)
    parser.add_argument("--pickup", nargs=2, type=float, metavar=("U", "V"),
                        help="场内侧抓取窗口中心的实测像素")
    parser.add_argument("--window-mm", type=float, default=8, help="抓取窗口的半径")
    args = parser.parse_args()
    validate_preview_arguments(parser, args, max_search_mm=40)
    if not (0 < args.tray_height_mm <= 150 and 0 < args.part_height_mm <= 120
            and 0 < args.window_mm < args.search_mm):
        parser.error("盘面高度、物料高度或窗口半径无效")
    try:
        calibration = RingMetric.load(args.calibration)
        height = args.tray_height_mm + args.part_height_mm
        vision = StackVision(calibration, args.grid_mm, height, args.diameter_mm,
                             args.search_mm, args.base_diameter_mm, args.tray_height_mm,
                             min_top_score=RAW_MIN_TOP_SCORE)
        pickup_offset = None
        if args.pickup is not None:
            pixel = np.asarray(args.pickup, dtype=float)
            if (not np.isfinite(pixel).all() or np.any(pixel < 0)
                    or np.any(pixel >= calibration.image_size)):
                raise ValueError("抓取窗口中心超出图像")
            ray = calibration._plane_points(pixel.reshape(1, 2))
            if ray is None:
                raise ValueError("抓取窗口中心无法投影到顶面")
            camera_position = calibration.camera_position * args.grid_mm
            pickup_offset = camera_position[:2] + ray[0] * vision.depth - vision.origin
            if np.any(np.abs(pickup_offset) + args.window_mm + 1 >= args.search_mm):
                raise ValueError("抓取窗口超出搜索范围，请调整观察姿态或搜索范围")
        motion = MotionObservation(pickup_offset, args.window_mm)
        run(args, vision, observe=motion.preview, title="Raw pickup vision - Space: save raw, Q: quit",
            prefix="pickup")
    except (ValueError, OSError, cv2.error) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
