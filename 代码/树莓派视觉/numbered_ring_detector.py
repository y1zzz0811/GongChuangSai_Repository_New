"""Detect printed 1/2/3 ring targets in a near-overhead camera image."""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


_ANGLES = np.arange(96, dtype=np.float32) * (2*np.pi/96)
_COS, _SIN = np.cos(_ANGLES)[:, None], np.sin(_ANGLES)[:, None]
_GLYPH_SIZE = 48
_PATCH_SIZE = 96
_TEMPLATE_FILE = Path(__file__).with_name("numbered_ring_templates.png")
_BACKGROUND_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (23, 23))
_STROKE_KERNEL = np.ones((3, 3), np.uint8)
_MIDPOINT = (_PATCH_SIZE-1)/2
_Y, _X = np.ogrid[:_PATCH_SIZE, :_PATCH_SIZE]
_DIGIT_DISK = (_X-_MIDPOINT)**2+(_Y-_MIDPOINT)**2 <= (_PATCH_SIZE*0.43)**2


@dataclass(frozen=True)
class Ring:
    center: tuple
    radius: float  # Approximate inner boundary in image pixels, for tracking only.
    number: Optional[int]
    score: float
    rim_points: Optional[np.ndarray] = None


def _radial_contrast(image, center, radii, half_width):
    """Measure bright-to-dark transitions without extending edges off-screen."""
    x, y = center
    inside, outside = radii-half_width, radii+half_width
    ix, iy = x + _COS*inside, y + _SIN*inside
    ox, oy = x + _COS*outside, y + _SIN*outside
    h, w = image.shape
    visible = ((ix >= 1) & (ix < w-2) & (iy >= 1) & (iy < h-2) &
               (ox >= 1) & (ox < w-2) & (oy >= 1) & (oy < h-2))
    dark = cv2.remap(image, ox, oy, cv2.INTER_LINEAR)
    light = cv2.remap(image, ix, iy, cv2.INTER_LINEAR)
    return np.where(visible, light.astype(np.float32)-dark, 0)


def _ellipse_radii(a, b, angle):
    directions = _ANGLES-np.deg2rad(angle)
    return (1/np.sqrt((2*np.cos(directions)/a)**2+
                      (2*np.sin(directions)/b)**2)).astype(np.float32)


def _inner_boundary(image, seed):
    x, y, radius = map(float, seed)
    radii = np.arange(max(9, radius*0.43), radius*1.35, 1, dtype=np.float32)
    if not len(radii):
        return None
    contrast = _radial_contrast(image, (x, y), radii, 2)
    profile = np.clip(contrast, 0, 35).mean(axis=0)
    good = (profile >= max(12, profile.max()*0.80)) & ((contrast > 9).mean(axis=0) >= 0.55)
    if not np.any(good):
        return None
    # Select the strongest inward edge near the first supported dark ring.
    start = int(np.flatnonzero(good)[0])
    index = start + int(np.argmax(profile[start:start+4]))
    radius = float(radii[index])
    a = b = 2*radius
    angle = 0.0
    for band in (max(3.5, radius*0.19), max(2.0, radius*0.10)):
        expected = _ellipse_radii(a, b, angle)
        offsets = np.arange(-band, band+0.25, 0.5, dtype=np.float32)
        radii = expected[:, None]+offsets
        contrast = _radial_contrast(image, (x, y), radii, 2)
        best = contrast.argmax(axis=1)
        strength = contrast[np.arange(len(_ANGLES)), best]
        supported = strength >= 10
        if supported.mean() < 0.55:
            return None
        angles = _ANGLES[supported]
        if np.diff(np.r_[angles, angles[0]+2*np.pi]).max() > 2.5:
            return None
        r = radii[np.arange(len(_ANGLES)), best][supported]
        points = np.column_stack((x+np.cos(angles)*r, y+np.sin(angles)*r))
        (nx, ny), (a, b), angle = cv2.fitEllipse(points.astype(np.float32))
        if (not np.isfinite((nx, ny, a, b, angle)).all() or min(a, b) < radius or
                min(a, b)/max(a, b) < 0.72 or max(a, b) > 2.65*radius or
                np.hypot(nx-x, ny-y) > radius*0.35):
            return None
        x, y = nx, ny
    if not (0 <= x < image.shape[1] and 0 <= y < image.shape[0]):
        return None
    return ((x, y), (a, b), angle), points


def _has_outer_ring(image, ellipse):
    center, (a, b), angle = ellipse
    # Verify the light gap beyond the inner rim and the next dark band.
    # These broad concentric transitions reject cables and curved shadows.
    base = _ellipse_radii(a, b, angle)[:, None]
    for start, end, direction, minimum in ((1.10, 1.50, -1, 0.56), (1.47, 1.76, 1, 0.43)):
        radii = base*np.linspace(start, end, 15, dtype=np.float32)
        contrast = direction*_radial_contrast(image, center, radii, 1.6)
        support = (contrast >= 8).mean(axis=0)
        strength = np.clip(contrast, 0, 35).mean(axis=0)
        if not np.any((support >= minimum) & (strength >= 7)):
            return False
    return True


def _glyph(gray, ellipse):
    (x, y), (a, b), angle = ellipse
    # Sample an upright patch with the fitted ellipse made circular. The
    # symmetric transform preserves the digit's image rotation for matching.
    c, s = np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))
    rotation = np.array([[c, -s], [s, c]])
    matrix = rotation @ np.diag([a, b]) @ rotation.T / _PATCH_SIZE
    transform = np.column_stack((matrix, np.array([x, y])-matrix @ [_MIDPOINT, _MIDPOINT]))
    patch = cv2.warpAffine(gray, transform, (_PATCH_SIZE, _PATCH_SIZE),
                           flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                           borderMode=cv2.BORDER_REPLICATE)
    background = cv2.morphologyEx(patch, cv2.MORPH_CLOSE, _BACKGROUND_KERNEL)
    contrast = background.astype(np.float32)-patch
    threshold = max(5.0, float(np.percentile(contrast[_DIGIT_DISK], 98))*0.30)
    mask = ((contrast >= threshold) & _DIGIT_DISK).astype(np.uint8)*255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _STROKE_KERNEL)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return None
    areas = stats[1:, cv2.CC_STAT_AREA]
    keep = np.r_[False, areas >= max(18, 0.13*areas.max())]
    mask = keep[labels].astype(np.uint8)*255
    left, top, width, height = cv2.boundingRect(mask)
    area = cv2.countNonZero(mask)
    if not (32 <= max(width, height) <= 82 and 95 <= area <= 2400):
        return None
    if np.hypot(left+width/2-_MIDPOINT, top+height/2-_MIDPOINT) > 16:
        return None
    # Pad a square centered on the digit before scaling. Preserve aspect ratio.
    glyph = mask[top:top+height, left:left+width]
    span = max(width, height)
    padded = np.zeros((span, span), np.uint8)
    i, j = (span-height)//2, (span-width)//2
    padded[i:i+height, j:j+width] = glyph
    result = np.zeros((_GLYPH_SIZE, _GLYPH_SIZE), np.uint8)
    result[5:-5, 5:-5] = cv2.resize(padded, (_GLYPH_SIZE-10, _GLYPH_SIZE-10),
                                   interpolation=cv2.INTER_AREA)
    return result


class NumberedRingDetector:
    def __init__(self):
        # The three tiles, in order, are reference glyphs extracted from the
        # actual printed 1/2/3 targets. Rotation variants are generated once.
        atlas = cv2.imdecode(np.fromfile(_TEMPLATE_FILE, np.uint8), cv2.IMREAD_GRAYSCALE)
        if atlas is None or atlas.shape != (_GLYPH_SIZE, 3*_GLYPH_SIZE):
            raise ValueError("数字圆环模板文件无效")
        rotations = [cv2.getRotationMatrix2D((23.5, 23.5), angle, 1)
                     for angle in range(0, 360, 10)]
        banks = []
        for tile in np.hsplit(atlas, 3):
            rotated = [cv2.warpAffine(tile, rotation, (_GLYPH_SIZE, _GLYPH_SIZE))
                       for rotation in rotations]
            vectors = np.array(rotated, dtype=np.float32).reshape(len(rotations), -1)
            banks.append(vectors/np.maximum(np.linalg.norm(vectors, axis=1, keepdims=True), 1))
        self.templates = np.concatenate(banks)

    def _read(self, glyph):
        if glyph is None:
            return None, 0.0
        vector = glyph.reshape(-1).astype(np.float32)
        scores = (self.templates @ (vector/max(np.linalg.norm(vector), 1))).reshape(3, -1).max(axis=1)
        best = int(np.argmax(scores))
        score = min(1.0, float(scores[best]))
        other = float(np.delete(scores, best).max())
        return (best+1 if score >= 0.69 and score-other >= 0.075 else None), score

    @staticmethod
    def _working_gray(frame):
        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        scale = min(1.0, 480/max(height, width))
        return (cv2.resize(gray, (round(width*scale), round(height*scale)),
                           interpolation=cv2.INTER_AREA) if scale < 1 else gray)

    @staticmethod
    def _contrast(work):
        background = cv2.GaussianBlur(work, (0, 0), 11)
        contrast = np.clip(128+2*(work.astype(np.float32)-background), 0, 255).astype(np.uint8)
        return cv2.GaussianBlur(contrast, (5, 5), 1)

    def _fit(self, gray, contrast, seed, scale, origin=(0, 0)):
        boundary = _inner_boundary(contrast, seed)
        if boundary is None:
            return None
        ellipse, points = boundary
        if not _has_outer_ring(contrast, ellipse):
            return None
        center, (a, b), _ = ellipse
        number, score = self._read(_glyph(gray, ellipse))
        center = (np.asarray(center)+origin+0.5)*scale-0.5
        radius = 0.5*float(np.sqrt(a*b*scale[0]*scale[1]))
        rim = (points+origin+0.5)*scale-0.5
        return Ring(tuple(center), radius, number, score, rim)

    def _fit_near(self, work, scale, hint, target):
        center, radius = hint
        x, y = (np.asarray(center)+0.5)/scale-0.5
        r = radius/np.sqrt(scale[0]*scale[1])
        # Keep the outer validation rings and enough background for the blur.
        margin = int(np.ceil(2.1*r+30))
        x0, y0 = max(0, int(x)-margin), max(0, int(y)-margin)
        x1, y1 = min(work.shape[1], int(x)+margin+1), min(work.shape[0], int(y)+margin+1)
        crop = work[y0:y1, x0:x1]
        if min(crop.shape) < 70:
            return None
        ring = self._fit(crop, self._contrast(crop), (x-x0, y-y0, r), scale, (x0, y0))
        if (ring is not None and ring.number == target and abs(ring.radius/radius-1) <= 0.15
                and np.linalg.norm(np.asarray(ring.center)-center) < max(5, radius*0.20)):
            return ring
        return None

    def detect(self, frame, *, target=None, hint=None, usable=None):
        height, width = frame.shape[:2]
        work = self._working_gray(frame)
        h, w = work.shape
        if min(h, w) < 70:
            return []
        scale = np.array([width/w, height/h])
        if target is not None and hint is not None:
            ring = self._fit_near(work, scale, hint, target)
            if ring is not None and (usable is None or usable(ring)):
                return [ring]
        contrast = self._contrast(work)
        candidates = []
        seen_seeds = set()
        # A narrower radius search can recover the inner rim when the first
        # pass placed its proposal on a clipped or blurred outer ring.
        for limit in (68, 52):
            seeds = cv2.HoughCircles(contrast, cv2.HOUGH_GRADIENT, 1, 28, param1=65, param2=23,
                                     minRadius=13, maxRadius=min(limit, min(h, w)//4))
            if seeds is None:
                continue
            for seed in seeds[0, :16]:
                key = tuple(seed)
                if key in seen_seeds:
                    continue
                seen_seeds.add(key)
                ring = self._fit(work, contrast, seed, scale)
                if ring is not None:
                    candidates.append(ring)
            rings = self._distinct(candidates)
            if target is None:
                if {ring.number for ring in candidates} >= {1, 2, 3}:
                    return rings
            else:
                matches = [ring for ring in rings if ring.number == target]
                if len(matches) == 1 and (usable is None or usable(matches[0])):
                    return rings
        return self._distinct(candidates)

    @staticmethod
    def _distinct(candidates):
        rings = []
        for ring in sorted(candidates, key=lambda item: (item.number is not None, item.score), reverse=True):
            x, y = ring.center
            if not any(np.hypot(x-other.center[0], y-other.center[1]) < min(ring.radius, other.radius)*0.85
                       for other in rings):
                rings.append(ring)
        return rings
