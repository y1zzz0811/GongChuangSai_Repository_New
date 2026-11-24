"""Top circular edges only, with ground/tray hypotheses; no motor output."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from ring_metric import RingMetric
from raw_pickup_circle_vision import CirclePickupVision
from stacking_vision import DIRECTIONS, PIXELS_PER_MM, add_preview_arguments, run, validate_preview_arguments


# Empirical ground forward correction: actual 10/20/30mm vs vision 15/25/40mm.
# Zero-preserving least squares: sum(actual*vision)/sum(vision**2).
GROUND_FWD_SCALE = 1850.0 / 2450.0
# User-approved provisional tray correction (2026-11-24); remeasure on the
# 140mm RAW plane. Kept separate from ground so they can be calibrated apart.
RAW_FWD_SCALE = 1850.0 / 2450.0

COLOR_IDS = {'RED': '1', 'YELLOW': '2', 'BLUE': '3', 'GREEN': '4', 'BLACK': '5', 'LIGHTBLUE': '6'}


def classify_surface_color(samples):
    """Require a dominant material color over the entire inner disk."""
    samples = np.asarray(samples)
    h, s, v = samples.T
    black = (v <= 65) & (s <= 90)
    if black.mean() >= 0.90:
        return 'BLACK'
    colored = (s >= 55) & (v >= 60)
    masks = {
        'RED': colored & ((h < 13) | (h >= 168)),
        'YELLOW': colored & (h >= 15) & (h <= 40),
        'GREEN': colored & (h >= 41) & (h < 86),
        'LIGHTBLUE': colored & (((h >= 86) & (h <= 103))
                               | ((h > 103) & (h <= 125) & (s <= 145))),
        'BLUE': colored & (h > 103) & (h <= 135) & (s > 145),
    }
    color, support = max(((color, mask.mean()) for color, mask in masks.items()), key=lambda pair: pair[1])
    return color if support >= 0.85 else None


def validate_top_surface(plane, lab, hsv, center, radius, complete_disk=False):
    # A Hough proposal must have a broadly supported, consistently signed rim.
    # Partial arcs on clamps, paper and the background are not filled disks.
    contrast = plane.rim_contrast(lab, center[None, None, :].astype(np.float32),
                                  np.array([[[radius]]], dtype=np.float32), DIRECTIONS)[0]
    signed = contrast * np.sign(np.median(contrast, axis=0))
    support = signed >= 4
    coverage = support.mean(axis=0)
    sectors = (support.reshape(8, 9, 3).sum(axis=1) >= 3).sum(axis=0)
    minimum_coverage, minimum_sectors = (0.48, 5) if complete_disk else (0.65, 6)
    if not np.any((coverage >= minimum_coverage) & (sectors >= minimum_sectors)):
        return None
    # Sample a filled interior disk, rather than one ring of 72 pixels.
    x0 = max(0, int(np.floor(center[0] - radius * 0.72)))
    x1 = min(hsv.shape[1], int(np.ceil(center[0] + radius * 0.72)) + 1)
    y0 = max(0, int(np.floor(center[1] - radius * 0.72)))
    y1 = min(hsv.shape[0], int(np.ceil(center[1] + radius * 0.72)) + 1)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    mask = (xx - center[0]) ** 2 + (yy - center[1]) ** 2 <= (radius * 0.72) ** 2
    if mask.sum() < 100 or np.mean(plane.visible[y0:y1, x0:x1][mask]) < 0.98:
        return None
    colors = hsv[y0:y1, x0:x1][mask]
    color = classify_surface_color(colors)
    if color is None:
        return None
    values = lab[y0:y1, x0:x1][mask]
    median = np.median(values, axis=0)
    luma_spread = np.quantile(np.abs(values[:, 0] - median[0]), 0.90)
    chroma_spread = np.quantile(np.linalg.norm(values[:, 1:] - median[1:], axis=1), 0.90)
    if luma_spread > 24 or chroma_spread > 12:
        return None
    return color

def refine_top_disk(lab, center, radius):
    """Fit a complete filled disk, rejecting a crescent-shaped cylinder side."""
    x0 = max(0, int(center[0] - radius * 1.8))
    x1 = min(lab.shape[1], int(center[0] + radius * 1.8) + 1)
    y0 = max(0, int(center[1] - radius * 1.8))
    y1 = min(lab.shape[0], int(center[1] + radius * 1.8) + 1)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    inner = (xx-center[0])**2 + (yy-center[1])**2 < (radius * 0.60)**2
    patch = lab[y0:y1, x0:x1]
    if inner.sum() < 100:
        return None
    values = patch[inner]
    median = np.median(values, axis=0)
    spread = np.quantile(np.abs(values-median), 0.90, axis=0)
    tolerance_l = np.clip(spread[0]*2, 12, 24)
    tolerance_c = np.clip(np.linalg.norm(spread[1:])*2, 6, 12)
    mask = ((np.abs(patch[..., 0]-median[0]) <= tolerance_l)
            & (np.linalg.norm(patch[..., 1:]-median[1:], axis=2) <= tolerance_c)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    sx, sy = np.rint(center-[x0,y0]).astype(int)
    if not (0 <= sx < mask.shape[1] and 0 <= sy < mask.shape[0]):
        return None
    label = labels[sy, sx]
    if label == 0:
        return None
    bx, by, bw, bh, pixels = stats[label]
    if bx <= 0 or by <= 0 or bx+bw >= mask.shape[1] or by+bh >= mask.shape[0]:
        return None
    component = (labels == label).astype(np.uint8)
    contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if len(contour) < 20:
        return None
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    if (area < 100 or perimeter <= 0 or hull_area <= 0
            or 4*np.pi*area/perimeter**2 < 0.78 or area/hull_area < 0.93):
        return None
    (cx,cy),(width,height),_ = cv2.fitEllipse(contour)
    fitted_center = np.array([cx+x0, cy+y0])
    fitted_radius = (width+height)/4
    if (min(width,height)/max(width,height) < 0.80
            or not 0.82 <= pixels/(np.pi*width*height/4) <= 1.12
            or np.linalg.norm(fitted_center-center) > radius*0.25):
        return None
    radial = np.linalg.norm(contour[:,0,:]-[cx,cy], axis=1)
    if np.quantile(np.abs(radial-fitted_radius),0.90)/fitted_radius > 0.12:
        return None
    return fitted_center, fitted_radius

class AxisVision(CirclePickupVision):
    def __init__(self, calibration, grid_mm, tray_height, part_height, diameter, base_diameter, roi=None):
        super().__init__(calibration, grid_mm, tray_height + part_height, diameter,
                         base_diameter_mm=base_diameter, base_height_mm=tray_height, roi=roi)

    def plane_edges(self, frame, plane, source):
        rectified = cv2.remap(frame, *plane.rectification_maps, cv2.INTER_LINEAR)
        lab = cv2.GaussianBlur(cv2.cvtColor(rectified, cv2.COLOR_BGR2LAB), (5, 5), 1.0)
        hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
        seeds = []
        # Independent geometric proposals from brightness and chromatic edges.
        for channel in cv2.split(lab):
            circles = cv2.HoughCircles(channel, cv2.HOUGH_GRADIENT, dp=1.5,
                                      minDist=plane.radius * 0.65, param1=60, param2=22,
                                      minRadius=round(plane.radius * 0.80),
                                      maxRadius=round(plane.radius * (1.30 if plane.height == 140 else 1.20)))
            if circles is not None:
                seeds.extend(circles[0])
        if not seeds:
            return []
        lab_float = lab.astype(np.float32)
        candidates = []
        for seed in seeds:
            delta = np.arange(-3, 4, 1.5, dtype=np.float32)
            dx, dy, dr = np.meshgrid(delta, delta, np.arange(-2, 3, dtype=np.float32))
            local = np.asarray(seed) + np.column_stack((dx.ravel(), dy.ravel(), dr.ravel()))
            local = local[(local[:, 2] >= plane.radius * 0.80) & (local[:, 2] <= plane.radius * (1.30 if plane.height == 140 else 1.20))]
            if not len(local):
                continue
            scores = plane.score(lab_float, local, base=(source == 'BASE'))
            best = int(scores.argmax())
            if scores[best] < 1.2:
                continue
            cx, cy, radius = local[best]
            center = np.array([cx, cy])
            disk = refine_top_disk(lab_float, center, radius)
            if disk is None:
                continue
            center, radius = disk
            if not plane.radius*0.80 <= radius <= plane.radius*(1.30 if plane.height == 140 else 1.20):
                continue
            # Color segmentation contracts slightly inside an antialiased rim.
            # Keep the contour-fitted CENTER fixed; search only nearby edge radii.
            edge_radii = radius + np.arange(-1, 6, dtype=np.float32)
            edge_candidates = np.column_stack((np.repeat(center.reshape(1,2), len(edge_radii), axis=0), edge_radii)).astype(np.float32)
            edge_scores = plane.score(lab_float, edge_candidates)
            edge_index = int(edge_scores.argmax())
            if edge_scores[edge_index] < 1.2:
                continue
            radius = float(edge_radii[edge_index])
            scores[best] = edge_scores[edge_index]
            # Color only confirms a material surface and labels it. It does
            # not supply the centroid or the fitted radius.
            color = validate_top_surface(plane, lab_float, hsv, center, radius, complete_disk=True)
            if color is None:
                continue
            world = plane.origin + (plane.map_center - center)[::-1] / PIXELS_PER_MM
            offset = world - self.origin
            top_center = self.map_center - offset[::-1] * PIXELS_PER_MM
            axis_pixel = self.project(top_center.reshape(1, 2))[0]
            x, y, w, h = self.roi
            if not (x <= axis_pixel[0] < x + w and y <= axis_pixel[1] < y + h):
                continue
            rim = plane.project(center + radius * DIRECTIONS)
            scores[best] *= np.exp(-((radius / plane.radius - 1) / 0.10) ** 2)
            item = {'color': color, 'source': source, 'offset': offset,
                    'center': axis_pixel, 'rim': rim, 'score': float(scores[best])}
            if all(np.linalg.norm(offset - old['offset']) > 8 for old in candidates):
                candidates.append(item)
            else:
                index = min(range(len(candidates)), key=lambda i: np.linalg.norm(offset - candidates[i]['offset']))
                if item['score'] > candidates[index]['score']:
                    candidates[index] = item
        return candidates

    def detect(self, frame):
        self.detected_circles = []
        self.axis_records = []
        if tuple(frame.shape[:2][::-1]) != self.calibration.image_size:
            return 'SIZE MISMATCH', None
        tops = self.plane_edges(frame, self, 'TOP')
        self.axis_records = tops
        self.axis_records.sort(key=lambda item: np.linalg.norm(item['offset']))
        for item in self.axis_records:
            label = item['color'] + ' ' + item['source']
            self.detected_circles.append((label, (item['center'], item['rim'], item['offset'], item['score'])))
        if not self.detected_circles:
            return 'NO RELIABLE AXIS EDGE', None
        return f'{len(self.detected_circles)} AXES (MARKER AT TOP HEIGHT)', self.detected_circles[0][1]


class MultiSurfaceAxisVision(AxisVision):
    def __init__(self, calibration, grid_mm, tray_height, part_height, diameter, base_diameter, roi=None):
        super().__init__(calibration, grid_mm, tray_height, part_height, diameter, base_diameter, roi)
        self.ground = AxisVision(calibration, grid_mm, 0, part_height, diameter, base_diameter, roi)

    def detect(self, frame):
        self.detected_circles = []
        self.axis_records = []
        if tuple(frame.shape[:2][::-1]) != self.calibration.image_size:
            return 'SIZE MISMATCH', None
        AxisVision.detect(self, frame)
        tray = list(self.axis_records)
        self.ground.detect(frame)
        hypotheses = []
        for surface, items in [('TRAY', tray), ('GROUND', self.ground.axis_records)]:
            for item in items:
                item = dict(item)
                item['surface'] = surface
                # Offsets use a common gripper reference at top height 140 mm,
                # regardless of the surface supporting the material.
                if surface == 'GROUND':
                    item['offset'] = item['offset'] + self.ground.origin - self.origin
                item['rank'] = item['score']
                hypotheses.append(item)
        hypotheses.sort(key=lambda item: item['rank'], reverse=True)
        chosen = []
        for item in hypotheses:
            duplicate = next((old for old in chosen if old['color'] == item['color']
                              and np.linalg.norm(old['center'] - item['center']) < 0.65 * (
                                  np.mean(np.linalg.norm(old['rim'] - old['center'], axis=1))
                                  + np.mean(np.linalg.norm(item['rim'] - item['center'], axis=1)))), None)
            if duplicate is None:
                item['height_uncertain'] = True
                chosen.append(item)
            elif item['surface'] != duplicate['surface'] and item['rank'] >= 0.80 * duplicate['rank']:
                duplicate['height_uncertain'] = True
        self.axis_records = sorted(chosen, key=lambda item: item['center'][0])
        self.detected_circles = []
        for item in self.axis_records:
            height_label = 'HEIGHT?' if item['height_uncertain'] else item['surface']
            label = item['color'] + ' ' + item['source'] + ' ' + height_label
            self.detected_circles.append((label, (item['center'], item['rim'], item['offset'], item['score'])))
        if not chosen:
            return 'NO RELIABLE AXIS EDGE', None
        return f'{len(chosen)} TOP CIRCLES', self.detected_circles[0][1]
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
            if self.axis_records[index - 1].get('height_uncertain'):
                detail = f'{index}: HEIGHT UNCERTAIN - PIXEL CENTER ONLY'
            else:
                detail = f'{index}: Fwd {offset[0]:+.1f} Left {offset[1]:+.1f} mm'
            for thickness, ink in [(3, (0, 0, 0)), (1, (240, 245, 245))]:
                cv2.putText(image, detail, (10, 25 + 23 * index),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, ink, thickness, cv2.LINE_AA)
        return image

class FixedSurfaceTopVision(MultiSurfaceAxisVision):
    def __init__(self, calibration, grid_mm, tray_height, part_height, diameter, base_diameter, roi=None, surface='ground'):
        support_height = 0 if surface == 'ground' else tray_height
        AxisVision.__init__(self, calibration, grid_mm, support_height, part_height, diameter, base_diameter, roi)
        self.fixed_surface = surface.upper()

    def detect(self, frame):
        status, result = AxisVision.detect(self, frame)
        for item in self.axis_records:
            item['surface'] = self.fixed_surface
            item['height_uncertain'] = False
            item['offset_geometric_mm'] = np.asarray(item['offset'], dtype=float).copy()
            if self.fixed_surface == 'GROUND':
                # Modify the shared array so records, overlay and returned result agree.
                item['offset'][0] *= GROUND_FWD_SCALE
            elif self.fixed_surface == 'TRAY':
                item['offset'][0] *= RAW_FWD_SCALE
        return status, result
class ProtocolTopVision(MultiSurfaceAxisVision):
    """Use the request type to select its physical plane, never the color ID."""
    def __init__(self, calibration, grid_mm, tray_height, part_height, diameter, base_diameter, roi=None, surface='tray'):
        self.detectors = {
            'TOP': FixedSurfaceTopVision(calibration, grid_mm, 80, 60, diameter, base_diameter, roi, 'ground'),
            'RAW': FixedSurfaceTopVision(calibration, grid_mm, 80, 60, diameter, base_diameter, roi, 'tray'),
        }
        self.calibration = calibration
        self.roi = self.detectors['TOP'].roi
        self.axis_records = []
        self.detected_circles = []
        self.last_frame = None
        self.mode = 'TOP' if surface == 'ground' else 'RAW'

    def select_mode(self, mode):
        self.mode = mode

    def detect(self, frame):
        self.last_frame = frame
        detector = self.detectors[self.mode]
        status, result = detector.detect(frame)
        self.axis_records = detector.axis_records
        self.detected_circles = detector.detected_circles
        return self.mode + ' ' + status, result
def main():
    cv2.setNumThreads(2)
    parser = argparse.ArgumentParser(description=__doc__)
    add_preview_arguments(parser)
    parser.set_defaults(calibration=Path(__file__).resolve().with_name('ring_metric_Z184_image_center.json'), search_mm=40)
    next(a for a in parser._actions if a.dest == 'calibration').required = False
    parser.add_argument('--tray-height-mm', type=float, default=80)
    parser.add_argument('--part-height-mm', type=float, default=60)
    parser.add_argument('--roi', nargs=4, type=int)
    parser.add_argument('--surface', choices=['auto', 'ground', 'tray'], default='auto',
                        help='Select actual support plane for metric coordinates')
    parser.add_argument('--top-search-mm', type=float, default=None,
                        help='Legacy option, ignored; TOP replies use the +/-500mm protocol range')
    parser.add_argument('--raw-static-test', action='store_true',
                        help='Explicit static RAW bench mode: bypass motion/timing/taught window; no RAW config required')
    parser.add_argument('--raw-config', type=Path, help='Measured RAW pickup window, timing and six Lab ranges')
    parser.add_argument('--uart', action='store_true', help='Enable existing VQ/VR coordinate protocol')
    parser.add_argument('--color-ids', nargs='+', metavar='COLOR=ID',
                        default=[f'{color}={target}' for color, target in COLOR_IDS.items()],
                        help='Controller colors: 1 red, 2 yellow, 3 blue, 4 green, 5 black, 6 light blue')
    args = parser.parse_args()
    color_ids = {}
    for specification in args.color_ids or []:
        try:
            color, target = specification.upper().split('=')
        except ValueError:
            parser.error('Color mapping must be COLOR=ID')
        if color not in COLOR_IDS or target not in '123456' or len(target) != 1:
            parser.error('Invalid color or target ID')
        if color in color_ids or target in color_ids.values():
            parser.error('Duplicate color or target ID')
        color_ids[color] = target
    if args.raw_static_test:
        args.uart = True
        if args.raw_config:
            parser.error('Use either formal --raw-config or --raw-static-test, not both')
    if args.uart and args.image:
        parser.error('Image replay never transmits')
    if args.uart and (args.tray_height_mm != 80 or args.part_height_mm != 60):
        parser.error('Protocol 2.0 uses tray 80mm / part 60mm; custom heights are preview-only')
    validate_preview_arguments(parser, args, max_search_mm=40)
    if not (0 < args.tray_height_mm <= 150 and 0 < args.part_height_mm <= 120):
        parser.error('Invalid material/tray height')
    try:
        calibration = RingMetric.load(args.calibration)
        if args.uart:
            vision = ProtocolTopVision(calibration, args.grid_mm, 80, 60, args.diameter_mm,
                        args.base_diameter_mm, args.roi, args.surface)
        elif args.surface == 'auto':
            vision = MultiSurfaceAxisVision(calibration, args.grid_mm, args.tray_height_mm,
                        args.part_height_mm, args.diameter_mm, args.base_diameter_mm, args.roi)
        else:
            vision = FixedSurfaceTopVision(calibration, args.grid_mm, args.tray_height_mm,
                        args.part_height_mm, args.diameter_mm, args.base_diameter_mm, args.roi, args.surface)
        if args.image:
            from stacking_vision import read_image, write_image
            frame = read_image(args.image)
            status, _ = vision.detect(frame)
            print(json.dumps({'status': status, 'axes': [dict(color=i['color'], source=i['source'],
                  center_px=i['center'].tolist(), surface=i['surface'], height_uncertain=i['height_uncertain'],
                  offset_mm=None if i['height_uncertain'] else i['offset'].tolist(), score=i['score']) for i in vision.axis_records]}))
            if args.output:
                write_image(args.output, vision.annotate(frame, status, None))
        else:
            if not args.uart:
                run(args, vision, title='Top circles only; Space: save raw, Q: quit', prefix='pickup_axis')
            else:
                import serial
                from qr_camera1 import UART_DEVICE, UART_BAUD
                from top_center_uart import TopCenterLink, RawGate, StaticRawGate
                raw_config_path = args.raw_config or Path(__file__).resolve().with_name('raw_config.json')
                raw_gate = (StaticRawGate() if args.raw_static_test else
                            RawGate.load(calibration, raw_config_path) if raw_config_path.exists() else None)
                if args.raw_config and not raw_config_path.exists():
                    raise ValueError(f'RAW config not found: {raw_config_path}')
                with serial.Serial(UART_DEVICE, UART_BAUD, bytesize=serial.EIGHTBITS,
                                   parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                   timeout=0, write_timeout=0.1, xonxoff=False,
                                   rtscts=False, dsrdtr=False, exclusive=True) as port:
                    link = TopCenterLink(port, search_mm=args.top_search_mm, raw=raw_gate,
                                         mode_callback=vision.select_mode)
                    link.report(f'UART OPEN {UART_DEVICE} {UART_BAUD} 8N1; protocol2.0: TOP=ground60 RAW=tray140')
                    if args.raw_static_test:
                        link.report('RAW STATIC TEST: rotation/timing/taught-window checks OFF; measured stable coordinates only')
                    if raw_gate is None:
                        link.report('RAW requests accepted; valid --raw-config required before RAW can reply')
                    def observe(status, result, captured_at):
                        try:
                            link.observe(vision.axis_records, captured_at, frame=vision.last_frame)
                        except (serial.SerialException, OSError) as exc:
                            raise OSError(f'Coordinate UART failed: {exc}') from exc
                        prefix = 'RAW STATIC TEST | ' if args.raw_static_test else ''
                        return prefix + status + ' | UART: ' + link.state
                    run(args, vision, observe=observe,
                        title='TOP ground / RAW tray - protocol 2.0; Q: quit', prefix='pickup_axis')
    except (ValueError, OSError, cv2.error) as exc:
        parser.exit(1, f'{exc}\n')


if __name__ == '__main__':
    main()
