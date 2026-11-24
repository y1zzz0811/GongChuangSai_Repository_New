"""Route 2 ring observations over the existing QR serial link."""

import math
import re
import time
import json

import cv2
import numpy as np

from raw_pickup_vision import MotionObservation, RawStopSession, RAW_MIN_TOP_SCORE
from stacking_vision import StackVision


TOP_TIMEOUT_S = 8.0
RING_TIMEOUT_S = 11.0
RING_FRAME_AGE_S = 1.0  # Single-frame limit for numbered ground-ring UART.
TOP_FRAME_AGE_S = 1.25
TOP_GROUP_AGE_MS = 3000


class _StaticManualObservation(MotionObservation):
    """Manual stationary material: verify fresh stability without inventing motion."""

    def update(self, offset, captured_at):
        super().update(offset, captured_at)
        self.saw_motion = False
        self.last_motion = None
        if offset is None:
            return 'TARGET LOST'
        recent = np.asarray([point for _, point in self.samples])
        if (len(recent) >= 4 and captured_at-self.samples[0][0] >= 1.0
                and np.max(np.ptp(recent, axis=0)) <= 1.0):
            return 'STOP OBSERVED'
        return 'VERIFYING STATIC'


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
        self.unlimited_stop = settings.get("unlimited_stop", False)
        if type(self.unlimited_stop) is not bool:
            raise ValueError("unlimited_stop must be true or false")
        self.static_manual = settings.get("static_manual", False)
        if type(self.static_manual) is not bool or (self.static_manual and not self.unlimited_stop):
            raise ValueError("static_manual requires explicit unlimited_stop=true")
        if self.unlimited_stop:
            # Explicit stationary/manual-feed testing: no invented dwell/action time.
            self.dwell = math.inf
            self.action = 0.0
        else:
            try:
                self.dwell = float(settings["minimum_stop_ms"]) / 1000
                self.action = float(settings["pickup_clear_ms"]) / 1000
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("Finite RAW timing requires measured minimum_stop_ms and pickup_clear_ms") from exc
        self.colors = {k: np.asarray(v, dtype=float) for k, v in settings["lab_ranges"].items()}
        if (not 0 < self.window < 29 or np.any(np.abs(self.pickup) + self.window >= 29)
                or (not self.unlimited_stop and (not 0 < self.action < self.dwell <= 120
                                                or self.dwell <= self.action + 1.35))
                or self.colors.keys() != set("123456")
                or any(v.shape != (2, 3) or not np.isfinite(v).all() or np.any(v < 0)
                       or np.any(v > 255) or np.any(v[0] >= v[1]) for v in self.colors.values())):
            raise ValueError("原料窗口、停留预算或六色 Lab 范围无效")

    @classmethod
    def load(cls, metric, path):
        return cls(metric, json.loads(path.read_text(encoding="utf-8-sig")))

    def observe(self, frame, stamp, target, tracker, moved_mm=(0.0, 0.0)):
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
                    moved_mm = np.asarray(moved_mm, dtype=float)
                    if (stopped and np.isfinite(moved_mm).all()
                            and np.linalg.norm(offset + moved_mm - self.pickup) <= self.window):
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
        self.raw_stop = RawStopSession()
        self.last_status = None
        self.last_status_at = -math.inf

    def _report_status(self, message, now):
        setter = getattr(self.camera, 'set_vision_status', None)
        if setter is not None:
            setter(message)
        if message and self.debug and message != self.last_status and now - self.last_status_at >= 1.0:
            print(message, flush=True)
            self.last_status, self.last_status_at = message, now

    def _set_request_mode(self, mode, requested_ring=None):
        setter = getattr(self.camera, "set_vision_request_mode", None)
        if setter is not None:
            if requested_ring is None:
                setter(mode)
            else:
                setter(mode, requested_ring)

    def _clear_pending(self):
        self.pending = None
        self._set_request_mode(None)
        self._report_status('', time.monotonic())

    def poll(self, now):
        for byte in self.port.read(min(self.port.in_waiting, 128)):
            if byte == 10:
                if not self.discard:
                    match = re.fullmatch(
                        rb"VQ,([1-9][0-9]{0,9}),([1-6])(?:,(TOP|RAW)(?:,([+-]?[0-9]{1,9}(?:\.[0-9]{1,6})?),([+-]?[0-9]{1,9}(?:\.[0-9]{1,6})?))?)?\r?",
                        self.line)
                    valid = (match is not None and int(match[1]) <= 0xffffffff
                             and (match[3] == b"RAW" or match[2] in (b"1", b"2", b"3")))
                    valid = valid and (match[4] is None or match[3] == b"RAW")
                    if valid:
                        moved_mm = (float(match[4] or 0), float(match[5] or 0))
                        kind = match[3]
                        self.pending = (int(match[1]), match[2].decode(), now, kind, moved_mm)
                        if kind is None:
                            self._set_request_mode("RING", int(match[2]))
                        else:
                            self._set_request_mode(kind.decode())
                        # Skip the next buffered frame as well as existing frames.
                        self.last_frame = self.camera.capture.snapshot()[1] + 1
                        self.samples.clear()
                        self.tracker = self.raw_stop.begin(
                            match[2].decode(), moved_mm,
                            kind == b"RAW" and match[4] is not None, now, self.raw)
                        if (kind == b"RAW" and getattr(self.raw, 'static_manual', False)
                                and not getattr(self.raw, 'static_test', False)):
                            self.tracker = _StaticManualObservation()
                        self.last_status = None
                        self._report_status('WAIT: fresh camera frame', now)
                        if self.debug:
                            print(f"VQ RX {self.line.decode('ascii').strip()}", flush=True)
                self.line.clear()
                self.discard = False
            elif not self.discard:
                if len(self.line) < 79:
                    self.line.append(byte)
                else:
                    self.line.clear()
                    self.discard = True
        if self.pending is None:
            return
        request, ring, requested_at, kind, moved_mm = self.pending
        timeout = TOP_TIMEOUT_S if kind == b"TOP" else 89 if kind == b"RAW" else RING_TIMEOUT_S
        frame_age_limit = TOP_FRAME_AGE_S if kind == b"TOP" else (0.65 if kind == b"RAW" else RING_FRAME_AGE_S)
        group_age_limit = TOP_GROUP_AGE_MS if kind == b"TOP" else 1500
        if now - requested_at >= timeout:
            if self.debug:
                print(f"VQ ID={request}: expired without reply", flush=True)
            self._clear_pending()
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
                self._clear_pending()
                return
            if stamp <= requested_at or not 0 <= now - stamp <= frame_age_limit:
                self.samples.clear()
                self.tracker.update(None, stamp)
                self._report_status('WAIT: stale/pre-request frame', now)
                return
            started = time.monotonic()
            if kind == b"RAW":
                offset = detector.observe(result["raw"], stamp, ring, self.tracker,
                                          moved_mm=moved_mm)
            else:
                status, observation = detector.detect(result["raw"])
                offset = None if observation is None else observation[2]
            now += time.monotonic() - started
            if kind == b"RAW" and offset is None:
                self._report_status(getattr(detector, 'status', 'RAW WAIT: no valid target'), now)
            if kind == b"TOP" and self.debug:
                print(f"TOP ID={request} frame={self.last_frame} age={(now-stamp)*1000:.0f} ms: "
                      f"{status} {offset if offset is not None else ''}", flush=True)
            if now - requested_at >= timeout:
                if self.debug:
                    print(f"VQ ID={request}: expired during detection", flush=True)
                self._clear_pending()
                return
        else:
            offset = result["rings"].get(ring, {}).get("offset_mm")
        if (stamp <= requested_at or not 0 <= now - stamp <= frame_age_limit or offset is None
                or not all(math.isfinite(v) and abs(v) <= 500 for v in offset)):
            if self.debug and offset is not None:
                print(f"VQ ID={request}: invalid measurement or frame age {(now-stamp)*1000:.0f} ms", flush=True)
            self.samples.clear()
            if offset is not None:
                self._report_status('WAIT: invalid coordinate or stale frame', now)
            return
        self.samples.append((offset, stamp))
        if len(self.samples) < 3:
            self._report_status(f'WAIT: stable sample {len(self.samples)}/3', now)
            return
        axes = tuple(zip(*(sample for sample, _ in self.samples)))
        if any(max(axis) - min(axis) > 2 for axis in axes):
            if self.debug:
                print(f"VQ ID={request}: three-frame spread exceeds 2 mm", flush=True)
            self.samples.pop(0)
            self._report_status('WAIT: coordinate spread exceeds 2mm', now)
            return
        forward, left = (round(sorted(axis)[1] * 1000) for axis in axes)
        age = math.ceil((now - self.samples[0][1]) * 1000)
        if age > group_age_limit:
            if self.debug:
                print(f"VQ ID={request}: oldest frame age {age} ms exceeds {group_age_limit} ms", flush=True)
            self.samples.pop(0)
            self._report_status('WAIT: observation group expired', now)
            return
        frame_id = ((self.last_frame - 1) % 0xffffffff) + 1
        body = f"VR,{request},{ring},{frame_id},{age},{forward},{left}".encode("ascii")
        if kind is not None:
            body += b"," + kind
        if kind == b"RAW":
            # Budget from the last observed moving frame, with link and
            # MCU reaction margin. Age reflects the oldest median sample.
            remaining = (self.raw.dwell - (now - self.tracker.last_motion)
                         if self.tracker.last_motion is not None else -1)
            if now - stamp > 0.35 or (not getattr(self.raw, 'static_test', False)
                                      and not getattr(self.raw, 'unlimited_stop', False)
                                      and remaining < self.raw.action + 0.35):
                self.samples.pop(0)
                self._report_status('RAW WAIT: latest frame >350ms or insufficient stop time', now)
                return
        packet = body + f",{crc8(body):02X}\n".encode("ascii")
        if self.port.write(packet) != len(packet):
            raise OSError("视觉串口帧未写完整")
        if kind == b"RAW":
            self.raw_stop.remember(ring, moved_mm, (forward / 1000, left / 1000), self.tracker)
        if self.debug:
            print(f"VR TX {packet.decode('ascii').strip()}", flush=True)
        self._clear_pending()
