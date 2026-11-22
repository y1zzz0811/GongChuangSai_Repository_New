"""Compare saved screenshots/raw frames without opening cameras or UART."""
from pathlib import Path
import json
import time

import cv2
import numpy as np

import target_center_detector as detector
from target_validation import legacy_target_center as legacy

folder = Path(__file__).parent / "target_validation"
cv2.setNumThreads(2)
for path in sorted(folder.glob("*.png")):
    if path.stem.endswith("_optimized"):
        continue
    frame = cv2.imread(str(path))
    report = {"image": path.name}
    output = frame.copy()
    for title, module in (("old", legacy), ("new", detector)):
        started = time.monotonic()
        targets = {}
        for name, mask in module.color_masks(frame).items():
            target = module.detect_target(mask)
            if target is not None:
                center = target["center"]
                targets[name] = [round(float(v), 2) for v in center]
                if title == "new":
                    point = tuple(np.rint(center).astype(int))
                    cv2.drawMarker(output, point, (255, 255, 255), cv2.MARKER_CROSS, 24, 4)
                    cv2.drawMarker(output, point, detector.COLORS[name], cv2.MARKER_CROSS, 24, 2)
                    cv2.putText(output, name + " CENTER", (point[0]+12, point[1]-12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
                    cv2.putText(output, name + " CENTER", (point[0]+12, point[1]-12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, detector.COLORS[name], 1, cv2.LINE_AA)
        report[title] = targets
        report[title+"_ms"] = round((time.monotonic()-started)*1000, 1)
    cv2.imwrite(str(path.with_stem(path.stem+"_optimized")), output)
    print(json.dumps(report), flush=True)
