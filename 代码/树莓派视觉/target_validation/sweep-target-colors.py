import cv2
import numpy as np
import target_center_detector as detector

cv2.setNumThreads(2)
for name in ('sample_red_green.png', 'sample_green_blue.png'):
    frame = cv2.imread('/home/string/Documents/target_validation/' + name)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    for saturation in (28, 35, 40, 45, 50, 55, 60, 65):
        mask = cv2.inRange(hsv, (32, saturation, 22), (92, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        target = detector.detect_target(mask)
        print(name, saturation, None if target is None else target['center'].round(2), flush=True)
