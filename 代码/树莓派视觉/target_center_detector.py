"""Estimate a coloured target's centre from complete or interrupted elliptical arcs.

No ring count is required. A candidate must surround a mostly uncoloured centre
and have colour support around a broad visible arc. All coordinates are image
coordinates; perspective/lens calibration is outside this detector.
"""
import cv2
import numpy as np


HSV_RANGES = {
    "BLUE": [((94, 18, 18), (145, 255, 255))],
    "GREEN": [((32, 40, 22), (92, 255, 255))],
    "RED": [((0, 42, 22), (16, 255, 255)),
            ((164, 42, 22), (179, 255, 255))],
}
COLORS = {"BLUE": (255, 170, 40), "GREEN": (40, 230, 40), "RED": (40, 70, 255)}
MIN_DIAMETER = 26
_ANGLES = np.linspace(0, 2*np.pi, 72, endpoint=False)
_RADII = np.linspace(0.10, 1.12, 65)[:, None]
_POLAR_U = (_RADII*np.cos(_ANGLES)[None, :]).astype(np.float32)
_POLAR_V = (_RADII*np.sin(_ANGLES)[None, :]).astype(np.float32)
_INNER = (_RADII[:, 0] >= 0.36) & (_RADII[:, 0] < 0.50)


def color_masks(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue, green, red = cv2.split(frame)
    chroma = cv2.subtract(cv2.max(cv2.max(blue, green), red), cv2.min(cv2.min(blue, green), red))
    result = {}
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    for name, ranges in HSV_RANGES.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for low, high in ranges:
            mask |= cv2.inRange(hsv, np.array(low), np.array(high))
        # Retain faded/dark green ink without accepting nearly neutral paper.
        mask[chroma < {"GREEN": 7, "BLUE": 4, "RED": 9}[name]] = 0
        result[name] = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return result


def _coordinates(points, ellipse):
    (cx, cy), (a, b), angle = ellipse
    theta = np.deg2rad(angle)
    points = points - (cx, cy)
    u = (points[:, 0]*np.cos(theta) + points[:, 1]*np.sin(theta))/(a/2)
    v = (-points[:, 0]*np.sin(theta) + points[:, 1]*np.cos(theta))/(b/2)
    return u, v


def _geometry_ok(ellipse, shape):
    (cx, cy), (a, b), _ = ellipse
    h, w = shape
    values = np.asarray((cx, cy, a, b))
    return (np.isfinite(values).all() and 0 <= cx < w and 0 <= cy < h
            and min(a, b) >= MIN_DIAMETER and max(a, b) < 2.2*max(w, h)
            and min(a, b)/max(a, b) >= 0.45)


def _fit(points, shape):
    points = np.asarray(points, np.float32).reshape(-1, 2)
    h, w = shape
    # Image-edge segments are crop boundaries, not printed elliptical arcs.
    points = points[(points[:, 0] > 2) & (points[:, 0] < w-3)
                    & (points[:, 1] > 2) & (points[:, 1] < h-3)]
    if len(points) < 8:
        return None
    try:
        ellipse = cv2.fitEllipse(points)
        if not _geometry_ok(ellipse, shape):
            return None
        u, v = _coordinates(points, ellipse)
        residual = np.abs(np.hypot(u, v)-1)
        tolerance = max(0.065, 5.0/min(ellipse[1]))
        inliers = residual < tolerance
        if inliers.mean() < 0.66:
            return None
        bins = ((np.arctan2(v[inliers], u[inliers])+np.pi)*36/(2*np.pi)).astype(int) % 36
        minimum_bins = min(14, max(7, int(inliers.sum()*0.6)))
        if len(np.unique(bins)) < minimum_bins:
            return None
        # Sparse hull vertices can span an entire small target. Check angular
        # span as well as bin count instead of rejecting such hulls outright.
        angles = np.sort(np.mod(np.arctan2(v[inliers], u[inliers]), 2*np.pi))
        if np.max(np.diff(np.r_[angles, angles[0]+2*np.pi])) > 0.96*np.pi:
            return None
        return ellipse, float(np.quantile(residual, 0.7))
    except cv2.error:
        return None


def _support(mask, ellipse):
    """Check an annular pattern instead of assuming every stroke is closed."""
    (cx, cy), (a, b), angle = ellipse
    theta = np.deg2rad(angle)
    u = _POLAR_U*(a/2)
    v = _POLAR_V*(b/2)
    x = (cx + u*np.cos(theta) - v*np.sin(theta)).astype(np.float32)
    y = (cy + u*np.sin(theta) + v*np.cos(theta)).astype(np.float32)
    h, w = mask.shape
    visible = (x >= 1) & (x < w-2) & (y >= 1) & (y < h-2)
    sampled = cv2.remap(mask, x, y, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT) != 0
    coverage = visible.mean(axis=1)
    support = (sampled & visible).sum(axis=1)/np.maximum(visible.sum(axis=1), 1)
    # Inspect the hollow region around the very centre. A small centre dot or
    # cross must not make an otherwise convincing ring target look like a blob.
    if (sampled[_INNER] & visible[_INNER]).sum()/max(int(visible[_INNER].sum()), 1) > 0.22:
        return None
    eligible = (_RADII[:, 0] >= 0.70) & (_RADII[:, 0] <= 1.08) & (coverage >= 0.55)
    if not np.any(eligible):
        return None
    indices = np.flatnonzero(eligible)
    best = indices[np.argmax(support[indices]*np.sqrt(coverage[indices]))]
    # More than half of the full perimeter must be visible, with a convincing
    # fraction of that perimeter covered by ink. Small isolated arcs cannot pass.
    if support[best] < 0.68 or support[best]*coverage[best] < 0.48:
        return None
    return float(support[best]*np.sqrt(coverage[best]))


def _hulls(mask):
    size = max(7, min(17, round(min(mask.shape)/55) | 1))
    joined = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size)))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(joined)
    indices = sorted(range(1, count), key=lambda i: stats[i, cv2.CC_STAT_AREA], reverse=True)[:12]
    for index in indices:
        x, y, w, h, area = stats[index]
        if min(w, h) < MIN_DIAMETER or area < 180:
            continue
        yy, xx = np.nonzero((labels[y:y+h, x:x+w] == index) & (mask[y:y+h, x:x+w] != 0))
        if len(xx) >= 60:
            points = np.column_stack((xx+x, yy+y)).astype(np.float32)
            yield cv2.convexHull(points).reshape(-1, 2)


def _candidates(boundaries, shape, validation_mask):
    for points in boundaries:
        fitted = _fit(points, shape)
        if fitted is None:
            # A clipped thick stroke contains both its inner and outer edges.
            fitted = _fit(cv2.convexHull(np.asarray(points, np.float32)), shape)
        if fitted is None:
            continue
        ellipse, error = fitted
        support = _support(validation_mask, ellipse)
        if support is None:
            continue
        quality = support - min(error, 0.15)*1.5
        yield dict(center=np.array(ellipse[0]), ellipse=ellipse,
                   diameter=float(np.sqrt(np.prod(ellipse[1]))),
                   quality=quality, error=error)


def detect_target(mask):
    if cv2.countNonZero(mask) < 60:
        return None
    # A one-pixel tolerance makes validation stable for thin printed strokes.
    validation_mask = cv2.dilate(mask, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    boundaries = [c.reshape(-1, 2) for c in contours
                  if len(c) >= 24 and cv2.contourArea(c) >= 35
                  and min(cv2.boundingRect(c)[2:]) >= MIN_DIAMETER/2]
    candidates = list(_candidates(boundaries, mask.shape, validation_mask))
    # Group disconnected arcs only when no strong direct fit was found. This
    # avoids full-frame component labelling on already well-detected targets.
    if not candidates or max(c["quality"] for c in candidates) < 0.90:
        candidates.extend(_candidates(_hulls(mask), mask.shape, validation_mask))
    if not candidates:
        return None
    best = max(candidates, key=lambda item: item["quality"])
    if best["quality"] < 0.78:
        return None
    neighbours = [c for c in candidates
                  if c["quality"] >= best["quality"]-0.12
                  and np.linalg.norm(c["center"]-best["center"]) <= max(3, best["diameter"]*0.04)]
    # Fuse consistent boundary estimates. No count of printed rings is needed.
    centres = np.array([c["center"] for c in neighbours])
    median = np.median(centres, axis=0)
    distances = np.linalg.norm(centres-median, axis=1)
    weights = np.array([max(c["quality"], 0.1)**2 for c in neighbours])/np.maximum(1, distances)
    center = np.average(centres, axis=0, weights=weights)
    return dict(center=center, diameter=max(c["diameter"] for c in neighbours),
                confidence=float(np.clip(best["quality"], 0, 1)))
