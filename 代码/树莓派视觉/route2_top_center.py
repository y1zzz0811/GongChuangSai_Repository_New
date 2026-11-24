"""Route-2 unified QR/ring/TOP/RAW service using the new top-edge detector.

Original programs are unchanged. One dual-camera worker owns the UART.
RAW uses --raw-config; static_manual=true shares the independent static gate.
"""
import argparse
from pathlib import Path
import sys
import cv2
import numpy as np
import dual_camera_recognition as dual
from route2_vision import RawTarget
from raw_pickup_axis_vision import FixedSurfaceTopVision
from top_center_uart import StaticRawGate


class TopAdapter:
    def __init__(self, metric, grid_mm=10, height_mm=60, diameter_mm=30,
                 search_mm=None, base_diameter_mm=50, base_height_mm=0,
                 min_top_score=5, surface='ground'):
        # Keep the old argument compatible; no distance gate below the
        # wire protocol's +/-500mm limit is applied to a detected TOP.
        self.vision = FixedSurfaceTopVision(metric, grid_mm, 80, 60, diameter_mm,
                                           base_diameter_mm, surface=surface)

    def detect(self, frame):
        status, _ = self.vision.detect(frame)
        eligible = [item for item in self.vision.axis_records
                    if np.isfinite(item['offset']).all() and np.all(np.abs(item['offset']) <= 500)]
        if len(eligible) != 1:
            return f'TOP candidates={len(eligible)} (protocol +/-500mm)', None
        item = eligible[0]
        return 'TOP EDGE', (item['center'], item['rim'], item['offset'], item['score'])


class EdgeRawTarget(RawTarget):
    def __init__(self, metric, settings):
        self.static_manual = settings.get('static_manual', False)
        if type(self.static_manual) is not bool:
            raise ValueError('static_manual must be true or false')
        self.static_test = self.static_manual
        if self.static_test:
            if settings.get('unlimited_stop') is not True:
                raise ValueError('static_manual requires explicit unlimited_stop=true')
            # Match the independent --raw-static-test path. Legacy Lab/window
            # fields are not alignment gates for a measured coordinate reply.
            self.unlimited_stop, self.dwell, self.action = True, float('inf'), 0.0
            self.static_gate = StaticRawGate()
        else:
            super().__init__(metric, settings)
        self.vision = TopAdapter(metric, search_mm=500 if self.static_test else 30, surface='tray')
        self.status = 'RAW WAIT: fresh requested-color target'

    def observe(self, frame, stamp, target, tracker, moved_mm=(0.0, 0.0)):
        self.vision.vision.detect(frame)
        if self.static_test:
            item, self.status = self.static_gate.measure(
                self.vision.vision.axis_records, frame, stamp, target, tracker)
            return None if item is None else item['offset']
        matched = []
        for item in self.vision.vision.axis_records:
            if np.any(np.abs(item['offset']) > 30):
                continue
            center, rim = item['center'], item['rim']
            u, v = np.rint(center).astype(int)
            radius = int(np.linalg.norm(rim - center, axis=1).min() / 3)
            if not (radius >= 2 and radius <= u < frame.shape[1] - radius
                    and radius <= v < frame.shape[0] - radius):
                continue
            patch = frame[v-radius:v+radius+1, u-radius:u+radius+1]
            lab = np.median(cv2.cvtColor(patch, cv2.COLOR_BGR2LAB), axis=(0, 1))
            colors = [key for key, bounds in self.colors.items()
                      if np.all(bounds[0] <= lab) and np.all(lab <= bounds[1])]
            if colors == [target]:
                matched.append(item['offset'])
        if len(matched) != 1:
            tracker.update(None, stamp)
            return None
        offset = matched[0]
        stopped = tracker.update(offset, stamp) == 'STOP OBSERVED'
        moved_mm = np.asarray(moved_mm, dtype=float)
        if (stopped and np.isfinite(moved_mm).all()
                and np.linalg.norm(offset + moved_mm - self.pickup) <= self.window):
            return offset
        return None


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--surface', choices=['ground', 'tray'], default='ground',
                        help='TOP target plane; RAW always uses the tray plane')
    settings, remaining = parser.parse_known_args(argv)
    # Dependency substitution is limited to this launcher's process. The
    # existing RingLink handles QR/ground/TOP/RAW frames and their deadlines.
    dual.StackVision = lambda *args, **kwargs: TopAdapter(*args, **kwargs, surface=settings.surface)
    dual.RawTarget = EdgeRawTarget
    if '--help' not in remaining and '-h' not in remaining:
        print(f'Unified route2 UART: TOP surface={settings.surface}; RAW surface=tray; '
              'normal VQ=ring; TOP=ring ID; RAW=color ID', flush=True)
        if '--raw-config' not in remaining:
            print('RAW NOT CONFIGURED: provide --raw-config; static_manual=true uses the single-item static gate', flush=True)
        if '--no-uart' not in remaining:
            if '--route2-vision' not in remaining and '--pickup-test' not in remaining:
                remaining.append('--route2-vision')
            if '--ring-metric' not in remaining:
                remaining.extend(['--ring-metric', str(Path(__file__).resolve().with_name('ring_metric_Z184_image_center.json'))])
            if '--vision-debug' not in remaining:
                remaining.append('--vision-debug')
    dual.main(remaining)


if __name__ == '__main__':
    main(sys.argv[1:])
