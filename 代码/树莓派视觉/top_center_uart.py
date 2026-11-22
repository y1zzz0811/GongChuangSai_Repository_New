"""Protocol 2.0 TOP/RAW link for the independent top-edge preview.
TOP uses a ground-plane top at 60mm; RAW uses the tray top at 140mm.
The existing dual-camera service retains QR and untyped numbered-ring VQ.
"""
import math
import re
import time
import numpy as np
import cv2
from route2_vision import crc8, RawTarget
from raw_pickup_vision import MotionObservation


class RawGate:
    def __init__(self, target):
        self.target = target
        self.dwell = target.dwell
        self.action = target.action
        self.displacement_mm = np.zeros(2)

    def set_displacement(self, displacement):
        self.displacement_mm = np.asarray(displacement, dtype=float).copy()

    @classmethod
    def load(cls, metric, path):
        import json
        from pathlib import Path
        settings = json.loads(Path(path).read_text(encoding='utf-8-sig'))
        required = {'pickup', 'window_mm', 'minimum_stop_ms', 'pickup_clear_ms', 'lab_ranges'}
        if not isinstance(settings, dict) or not required <= settings.keys():
            missing = sorted(required - settings.keys()) if isinstance(settings, dict) else sorted(required)
            raise ValueError('RAW config missing: ' + ', '.join(missing) + '; camera geometry JSON is not RAW config')
        try:
            return cls(RawTarget(metric, settings))
        except (KeyError, TypeError) as exc:
            raise ValueError('Invalid RAW configuration: ' + str(exc)) from exc

    def measure(self, records, frame, stamp, target_id, tracker):
        if frame is None:
            tracker.update(None, stamp)
            return None, 'RAW WAIT: no current camera frame'
        eligible = []
        for item in records:
            if item.get('height_uncertain', True) or item.get('source') != 'TOP':
                continue
            if not np.isfinite(item['offset']).all() or np.any(np.abs(item['offset']) > 30):
                continue
            center, rim = item['center'], item['rim']
            u, v = np.rint(center).astype(int)
            radius = int(np.linalg.norm(rim - center, axis=1).min() / 3)
            if not (radius >= 2 and radius <= u < frame.shape[1] - radius
                    and radius <= v < frame.shape[0] - radius):
                continue
            patch = frame[v-radius:v+radius+1, u-radius:u+radius+1]
            lab = np.median(cv2.cvtColor(patch, cv2.COLOR_BGR2LAB), axis=(0, 1))
            matches = [key for key, bounds in self.target.colors.items()
                       if np.all(bounds[0] <= lab) and np.all(lab <= bounds[1])]
            if matches == [target_id]:
                eligible.append(item)
        if len(eligible) != 1:
            tracker.update(None, stamp)
            return None, f'RAW WAIT: {len(eligible)} unique requested-color matches'
        offset = np.asarray(eligible[0]['offset'], dtype=float)
        motion = tracker.update(offset, stamp)
        if motion != 'STOP OBSERVED':
            return None, 'RAW ' + motion
        if np.linalg.norm(offset + self.displacement_mm - self.target.pickup) > self.target.window:
            return None, 'RAW WAIT: outside taught pickup window'
        if abs(offset[0]) > 2 or abs(offset[1]) > 5:
            return None, 'RAW WAIT: outside Fwd +/-2mm / Left +/-5mm'
        return eligible[0], 'RAW stopped and aligned'


class StaticRawGate:
    """Explicit bench mode: skips rotation/timing/taught-window validation.

    Not the production moving-turntable RAW gate. Returned coordinates remain
    measured, current, requested-color and stable. Static replies deliberately
    include nonzero alignment errors; the controller decides subsequent actions.
    """
    static_test = True
    colors = {'RED': '1', 'YELLOW': '2', 'BLUE': '3', 'GREEN': '4', 'BLACK': '5', 'LIGHTBLUE': '6'}

    def measure(self, records, frame, stamp, target_id, tracker):
        matches = [item for item in records if item.get('source') == 'TOP'
                   and not item.get('height_uncertain', True)
                   and self.colors.get(item.get('color')) == target_id
                   and np.isfinite(item['offset']).all()
                   and np.all(np.abs(item['offset']) <= 500)]
        if len(matches) != 1:
            return None, f'RAW STATIC WAIT: {len(matches)} requested-color matches in ROI (protocol +/-500mm)'

        return matches[0], 'RAW STATIC: measured center (rotation/timing/window/alignment gates disabled)'

class TopCenterLink:
    def __init__(self, port, color_ids=None, clock=time.monotonic, search_mm=50,
                 raw=None, mode_callback=None, log_path='/tmp/top_center_uart.log'):
        self.port, self.clock = port, clock
        self.search_mm = float(search_mm)
        self.raw, self.mode_callback = raw, mode_callback
        self.log_path = log_path
        self.buffer = bytearray()
        self.discard = False
        self.pending = None
        self.samples = []
        self.sequence = 0
        self.state = 'WAIT VQ TOP/RAW'
        self.last_report = None
        self.displacement_mm = np.zeros(2)
        self.tracker = MotionObservation()

    def report(self, message, force=False):
        self.state = message
        if not force and message == self.last_report:
            return
        self.last_report = message
        line = time.strftime('%Y-%m-%d %H:%M:%S') + ' ' + message
        print(line, flush=True)
        if self.log_path:
            try:
                with open(self.log_path, 'a', encoding='utf-8') as log:
                    log.write(line + '\n')
            except OSError:
                pass

    def observe(self, records, captured_at, frame=None):
        # Keep a monotonic internal sequence; wrap only the wire FRAME field.
        self.sequence += 1
        now = self.clock()
        for byte in self.port.read(min(self.port.in_waiting, 128)):
            if byte == 10:
                if not self.discard:
                    line = bytes(self.buffer)
                    self.report('UART RX ' + repr(line), force=True)
                    match = re.fullmatch(rb'VQ,([1-9][0-9]{0,9}),([1-6]),(TOP|RAW)\r?', line)
                    valid = (match is not None and int(match[1]) <= 0xffffffff
                             and (match[3] == b'RAW' or int(match[2]) <= 3))
                    extended = re.fullmatch(
                        rb'VQ,([1-9][0-9]{0,9}),([1-6]),RAW,([+-]?[0-9]{1,9}(?:\.[0-9]{1,6})?),([+-]?[0-9]{1,9}(?:\.[0-9]{1,6})?)\r?', line)
                    extended_valid = extended is not None and int(extended[1]) <= 0xffffffff
                    if extended_valid or valid:
                        request_match = extended if extended_valid else match
                        kind = 'RAW' if extended_valid else match[3].decode()
                        displacement = (np.array([float(extended[3]), float(extended[4])])
                                        if extended_valid else np.zeros(2))
                        self.displacement_mm = displacement
                        if self.raw is not None and hasattr(self.raw, 'set_displacement'):
                            self.raw.set_displacement(displacement)
                        self.pending = (int(request_match[1]), request_match[2].decode(), kind, now, self.sequence)
                        self.samples.clear()
                        self.tracker = MotionObservation()
                        if self.mode_callback:
                            self.mode_callback(kind)
                        self.report(f'VQ RX accepted {kind} ID={int(request_match[1])} '
                                    f'target={request_match[2].decode()} DF/DL_mm={displacement.tolist()}')
                    else:
                        self.report('IGNORED: invalid TOP/RAW request; untyped rings use the unified service')
                self.buffer.clear()
                self.discard = False
            elif not self.discard:
                if len(self.buffer) < 79:
                    self.buffer.append(byte)
                else:
                    self.buffer.clear()
                    self.discard = True
        if self.pending is None:
            return
        request, target, kind, started, first_sequence = self.pending
        is_raw = kind == 'RAW'
        timeout, frame_limit, group_limit = (89, 0.65, 1500) if is_raw else (8, 1.25, 3000)
        if now - started >= timeout:
            self.report(f'{kind} TIMEOUT: no valid response within {timeout}s')
            self.pending = None
            self.samples.clear()
            return
        if is_raw and self.raw is None:
            self.report('RAW ACCEPTED - CONFIG MISSING: --raw-config needs pickup/window/timing/Lab ranges')
            return
        if self.sequence <= first_sequence + 1 or captured_at <= started:
            self.report('WAIT: skipping pre-request/cached frame')
            return
        if not 0 <= now - captured_at <= frame_limit:
            self.samples.clear()
            if is_raw:
                self.tracker.update(None, captured_at)
            self.report(f'{kind} WAIT: stale frame {(now-captured_at)*1000:.0f}ms')
            return
        if is_raw:
            item, reason = self.raw.measure(records, frame, captured_at, target, self.tracker)
            if item is None:
                self.samples.clear()
                self.report(reason)
                return
        else:
            matches = [item for item in records if item.get('source') == 'TOP'
                       and not item.get('height_uncertain', True)
                       and all(math.isfinite(float(v)) and abs(float(v)) <= min(500, self.search_mm)
                               for v in item['offset'])]
            if len(matches) != 1:
                self.samples.clear()
                self.report(f'TOP WAIT: {len(matches)} eligible targets, limit +/-{self.search_mm:g}mm')
                return
            item = matches[0]
        offset = tuple(float(v) for v in item['offset'])
        if len(offset) != 2 or not all(math.isfinite(v) and abs(v) <= 500 for v in offset):
            self.samples.clear()
            return
        center = tuple(float(v) for v in item['center'])
        if self.samples and math.dist(center, self.samples[-1][2]) > 12:
            self.samples.clear()
        self.samples.append((offset, captured_at, center))
        self.samples = self.samples[-3:]
        if len(self.samples) < 3:
            self.report(f'{kind} WAIT: stable sample {len(self.samples)}/3')
            return
        axes = list(zip(*(sample[0] for sample in self.samples)))
        if any(max(axis) - min(axis) > 2 for axis in axes):
            self.report(kind + ' WAIT: coordinate spread exceeds 2mm')
            return
        now = self.clock()
        age = math.ceil((now - self.samples[0][1]) * 1000)
        if age > group_limit or now - captured_at > frame_limit or now - started >= timeout:
            self.samples.pop(0)
            self.report(kind + ' WAIT: observation expired during processing')
            return
        if is_raw:
            if now - captured_at > 0.35:
                self.samples.pop(0)
                self.report('RAW WAIT: latest frame >350ms')
                return
            if not getattr(self.raw, 'static_test', False):
                remaining = (self.raw.dwell - (now - self.tracker.last_motion)
                             if self.tracker.last_motion is not None else -1)
                if remaining < self.raw.action + 0.35:
                    self.samples.pop(0)
                    self.report('RAW WAIT: insufficient remaining stop time')
                    return
        forward, left = (round(sorted(axis)[1] * 1000) for axis in axes)
        frame_id = (self.sequence - 1) % 0xffffffff + 1
        body = f'VR,{request},{target},{frame_id},{age},{forward},{left},{kind}'.encode('ascii')
        packet = body + f',{crc8(body):02X}\n'.encode('ascii')
        if len(packet.rstrip(b'\n')) > 79:
            raise ValueError('Coordinate packet exceeds controller line capacity')
        if self.port.write(packet) != len(packet):
            raise OSError('Incomplete coordinate serial write')
        self.report('VR TX ' + packet.decode('ascii').strip(), force=True)
        self.pending = None
        self.samples.clear()
