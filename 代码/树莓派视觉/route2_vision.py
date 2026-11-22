"""Route 2 ring observations over the existing QR serial link."""

import math
import re
import time
import json

import cv2
import numpy as np

from raw_pickup_vision import MotionObservation, RAW_MIN_TOP_SCORE
from stacking_vision import StackVision


TOP_TIMEOUT_S = 8.0
RING_FRAME_AGE_S = 1.0  # Single-frame limit for numbered ground-ring UART.
TOP_FRAME_AGE_S = 1.25
TOP_GROUP_AGE_MS = 3000
RAW_FORWARD_TOLERANCE_MM = 2.0
RAW_LEFT_TOLERANCE_MM = 5.0


class RawTarget:
    """Calibrated raw-station observation. No defaults for color or timing."""

    def __init__(self, metric, settings):
        # Legacy settings["zero"] is ignored; the pickup window remains taught.
        self.vision = StackVision(metric, 10, 140, 30, 30, 50, 80,
                                  min_top_score=RAW_MIN_TOP_SCORE)
        pickup = np.asarray(settings["pickup"], dtype=float)
        if pickup.shape != (2,) or not np.isfinite(pickup).all() or np.any(pickup < 0) or np.any(pickup >= metric.image_size):
            raise ValueError("原料抓取窗口中心无效")
        ray = metric._plane_points(pickup.reshape(1, 2))
        if ray is None:
            raise ValueError("原料抓取窗口无法投影")
        self.pickup = 10 * metric.camera_position[:2] + ray[0] * self.vision.depth - self.vision.origin
        self.window = float(settings["window_mm"])
        self.dwell = float(settings["minimum_stop_ms"]) / 1000
        self.action = float(settings["pickup_clear_ms"]) / 1000
        self.colors = {k: np.asarray(v, dtype=float) for k, v in settings["lab_ranges"].items()}
        if (not 0 < self.window < 29 or np.any(np.abs(self.pickup) + self.window >= 29)
                or not 0 < self.action < self.dwell <= 120 or self.dwell <= self.action + 1.35
                or self.colors.keys() != set("123456")
                or any(v.shape != (2, 3) or not np.isfinite(v).all() or np.any(v < 0)
                       or np.any(v > 255) or np.any(v[0] >= v[1]) for v in self.colors.values())):
            raise ValueError("原料窗口、停留预算或六色 Lab 范围无效")

    @classmethod
    def load(cls, metric, path):
        return cls(metric, json.loads(path.read_text(encoding="utf-8-sig")))

    def observe(self, frame, stamp, target, tracker):
        _, result = self.vision.detect(frame)
        if result is not None:
            center, rim, offset, _ = result
            u, v = center.round().astype(int)
            radius = int(np.linalg.norm(rim - center, axis=1).min() / 3)
            if (radius >= 2 and radius <= u < frame.shape[1] - radius
                    and radius <= v < frame.shape[0] - radius):
                patch = frame[v-radius:v+radius+1, u-radius:u+radius+1]
                lab = np.median(cv2.cvtColor(patch, cv2.COLOR_BGR2LAB), axis=(0, 1))
                matches = [key for key, bounds in self.colors.items()
                           if np.all(bounds[0] <= lab) and np.all(lab <= bounds[1])]
                if matches == [target]:
                    stopped = tracker.update(offset, stamp) == "STOP OBSERVED"
                    if (stopped and np.linalg.norm(offset - self.pickup) <= self.window
                            and abs(offset[0]) <= RAW_FORWARD_TOLERANCE_MM
                            and abs(offset[1]) <= RAW_LEFT_TOLERANCE_MM):
                        return offset
                    return None
        tracker.update(None, stamp)
        return None


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (7 if crc & 0x80 else 0)) & 0xff
    return crc


class RingLink:
    def __init__(self, port, camera, top=None, raw=None, debug=False):
        self.port = port
        self.camera = camera
        self.top = top
        self.raw = raw
        self.debug = debug
        self.line = bytearray()
        self.discard = False
        self.pending = None
        self.last_frame = 0
        self.samples = []
        self.tracker = MotionObservation()

    def poll(self, now):
        for byte in self.port.read(min(self.port.in_waiting, 128)):
            if byte == 10:
                if not self.discard:
                    match = re.fullmatch(rb"VQ,([1-9][0-9]{0,9}),([1-6])(?:,(TOP|RAW))?\r?", self.line)
                    if match and int(match[1]) <= 0xffffffff and (match[3] == b"RAW" or match[2] in (b"1", b"2", b"3")):
                        self.pending = (int(match[1]), match[2].decode(), now, match[3])
                        # Skip the next buffered frame as well as existing frames.
                        self.last_frame = self.camera.capture.snapshot()[1] + 1
                        self.samples.clear()
                        self.tracker = MotionObservation()
                        if self.debug:
                            print(f"VQ RX {self.line.decode('ascii').strip()}", flush=True)
                self.line.clear()
                self.discard = False
            elif not self.discard:
                if len(self.line) < 32:
                    self.line.append(byte)
                else:
                    self.line.clear()
                    self.discard = True
        if self.pending is None:
            return
        request, ring, requested_at, kind = self.pending
        timeout = TOP_TIMEOUT_S if kind == b"TOP" else 89 if kind == b"RAW" else 4
        frame_age_limit = TOP_FRAME_AGE_S if kind == b"TOP" else (0.65 if kind == b"RAW" else RING_FRAME_AGE_S)
        group_age_limit = TOP_GROUP_AGE_MS if kind == b"TOP" else 1500
        if now - requested_at >= timeout:
            if self.debug:
                print(f"VQ ID={request}: expired without reply", flush=True)
            self.pending = None
            return
        if kind is None:
            result, error = self.camera.snapshot()
        else:
            frame, sequence, _, stamp = self.camera.capture.timed_snapshot()
            result = None if frame is None else dict(raw=frame, sequence=sequence, captured_at=stamp)
            error = None
        if error or result is None or result["sequence"] <= self.last_frame:
            return
        self.last_frame = result["sequence"]
        stamp = result["captured_at"]
        if kind is not None:
            detector = self.raw if kind == b"RAW" else self.top
            if detector is None:
                if self.debug:
                    print(f"VQ ID={request}: target mode not configured", flush=True)
                self.pending = None
                return
            if stamp <= requested_at or not 0 <= now - stamp <= frame_age_limit:
                self.samples.clear()
                self.tracker.update(None, stamp)
                return
            started = time.monotonic()
            if kind == b"RAW":
                offset = detector.observe(result["raw"], stamp, ring, self.tracker)
            else:
                status, observation = detector.detect(result["raw"])
                offset = None if observation is None else observation[2]
            now += time.monotonic() - started
            if kind == b"TOP" and self.debug:
                print(f"TOP ID={request} frame={self.last_frame} age={(now-stamp)*1000:.0f} ms: "
                      f"{status} {offset if offset is not None else ''}", flush=True)
            if kind == b"TOP" and now - requested_at >= timeout:
                if self.debug:
                    print(f"VQ ID={request}: expired during detection", flush=True)
                self.pending = None
                return
        else:
            offset = result["rings"].get(ring, {}).get("offset_mm")
        if (stamp <= requested_at or not 0 <= now - stamp <= frame_age_limit or offset is None
                or not all(math.isfinite(v) and abs(v) <= 500 for v in offset)):
            if self.debug and offset is not None:
                print(f"VQ ID={request}: invalid measurement or frame age {(now-stamp)*1000:.0f} ms", flush=True)
            self.samples.clear()
            return
        self.samples.append((offset, stamp))
        if len(self.samples) < 3:
            return
        axes = tuple(zip(*(sample for sample, _ in self.samples)))
        if any(max(axis) - min(axis) > 2 for axis in axes):
            if self.debug:
                print(f"VQ ID={request}: three-frame spread exceeds 2 mm", flush=True)
            self.samples.pop(0)
            return
        forward, left = (round(sorted(axis)[1] * 1000) for axis in axes)
        age = math.ceil((now - self.samples[0][1]) * 1000)
        if age > group_age_limit:
            if self.debug:
                print(f"VQ ID={request}: oldest frame age {age} ms exceeds {group_age_limit} ms", flush=True)
            self.samples.pop(0)
            return
        frame_id = ((self.last_frame - 1) % 0xffffffff) + 1
        body = f"VR,{request},{ring},{frame_id},{age},{forward},{left}".encode("ascii")
        if kind is not None:
            body += b"," + kind
        if kind == b"RAW":
            # Budget from the last observed moving frame, with link and
            # MCU reaction margin. Age reflects the oldest median sample.
            remaining = self.raw.dwell - (now - self.tracker.last_motion)
            if now - stamp > 0.35 or remaining < self.raw.action + 0.35:
                self.samples.pop(0)
                return
        packet = body + f",{crc8(body):02X}\n".encode("ascii")
        if self.port.write(packet) != len(packet):
            raise OSError("视觉串口帧未写完整")
        if self.debug:
            print(f"VR TX {packet.decode('ascii').strip()}", flush=True)
        self.pending = None
