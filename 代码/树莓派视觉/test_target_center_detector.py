"""Target geometry, loss/false-positive checks, and supplied-image regressions."""
from pathlib import Path
import unittest

import cv2
import numpy as np

from target_center_detector import color_masks, detect_target


def draw_target(frame, center, color, dashed=False, axes=(92, 80), angle=12):
    for scale in (0.60, 0.80, 1.0):
        radii = tuple(round(value*scale) for value in axes)
        if dashed:
            for start in range(0, 360, 24):
                cv2.ellipse(frame, center, radii, angle, start, start+18, color, 3, cv2.LINE_AA)
        else:
            cv2.ellipse(frame, center, radii, angle, 0, 360, color, 3, cv2.LINE_AA)


class TargetCenterTests(unittest.TestCase):
    def setUp(self):
        cv2.setNumThreads(2)

    def test_three_colours_and_faded_green(self):
        frame = np.full((600, 800, 3), 180, np.uint8)
        faded_green = tuple(int(v) for v in cv2.cvtColor(
            np.array([[[60, 48, 110]]], np.uint8), cv2.COLOR_HSV2BGR)[0, 0])
        centres = {"RED": (135, 290), "GREEN": (400, 320), "BLUE": (665, 290)}
        for name, color in (("RED", (25, 30, 160)), ("GREEN", faded_green), ("BLUE", (100, 30, 25))):
            draw_target(frame, centres[name], color)
        for name, mask in color_masks(frame).items():
            with self.subTest(colour=name):
                target = detect_target(mask)
                self.assertIsNotNone(target)
                np.testing.assert_allclose(target["center"], centres[name], atol=1.5)

    def test_dashed_arcs_without_closed_rings(self):
        frame = np.full((600, 800, 3), 180, np.uint8)
        draw_target(frame, (400, 300), (20, 40, 180), dashed=True)
        target = detect_target(color_masks(frame)["RED"])
        self.assertIsNotNone(target)
        np.testing.assert_allclose(target["center"], (400, 300), atol=2)

    def test_target_partly_outside_bottom_edge(self):
        frame = np.full((600, 800, 3), 180, np.uint8)
        draw_target(frame, (400, 565), (130, 30, 20), axes=(100, 90))
        target = detect_target(color_masks(frame)["BLUE"])
        self.assertIsNotNone(target)
        np.testing.assert_allclose(target["center"], (400, 565), atol=2)

    def test_single_ring_is_enough(self):
        frame = np.full((600, 800, 3), 180, np.uint8)
        cv2.ellipse(frame, (400, 300), (80, 60), 25, 0, 360, (30, 160, 30), 3)
        target = detect_target(color_masks(frame)["GREEN"])
        self.assertIsNotNone(target)
        np.testing.assert_allclose(target["center"], (400, 300), atol=1)

    def test_solid_blobs_rectangles_and_short_arcs_are_rejected(self):
        for kind in ("disk", "rectangle", "short_arc"):
            frame = np.full((600, 800, 3), 180, np.uint8)
            if kind == "disk":
                cv2.circle(frame, (400, 300), 80, (20, 160, 20), -1)
            elif kind == "rectangle":
                cv2.rectangle(frame, (300, 220), (500, 380), (20, 160, 20), 4)
            else:
                cv2.ellipse(frame, (400, 300), (90, 80), 10, 0, 75, (20, 160, 20), 4)
            with self.subTest(shape=kind):
                self.assertIsNone(detect_target(color_masks(frame)["GREEN"]))

    def test_user_screenshots(self):
        # Approximate reference centres read from the printed target geometry,
        # not from the old program's crosses. These are regression tolerances,
        # not camera calibration or a measurement of physical accuracy.
        expected = {
            "sample_red.png": {"RED": (698, 412)},
            "sample_red_green.png": {"RED": (285, 379), "GREEN": (643, 412)},
            "sample_green_blue.png": {"GREEN": (154, 604), "BLUE": (532, 631)},
            "sample_blue_missed.png": {"BLUE": (649, 293)},
            "sample_blue_intermittent.png": {"BLUE": (663, 277)},
        }
        folder = Path(__file__).parent/"target_validation"
        for filename, centres in expected.items():
            path = folder/filename
            if not path.exists():
                self.skipTest("Reference screenshot fixtures are not present")
            masks = color_masks(cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR))
            for name, center in centres.items():
                with self.subTest(image=filename, colour=name):
                    target = detect_target(masks[name])
                    self.assertIsNotNone(target)
                    np.testing.assert_allclose(target["center"], center, atol=8)

    def test_faded_small_blue_target(self):
        frame = np.full((600, 800, 3), 155, np.uint8)
        blue = tuple(int(v) for v in cv2.cvtColor(
            np.array([[[112, 27, 90]]], np.uint8), cv2.COLOR_HSV2BGR)[0, 0])
        draw_target(frame, (580, 270), blue, axes=(48, 44))
        frame = cv2.GaussianBlur(frame, (3, 3), 0.6)
        target = detect_target(color_masks(frame)["BLUE"])
        self.assertIsNotNone(target)
        np.testing.assert_allclose(target["center"], (580, 270), atol=2)

    def test_neutral_noise_is_not_a_blue_target(self):
        rng = np.random.default_rng(234)
        frame = np.clip(145 + rng.normal(0, 2, (600, 800, 3)), 0, 255).astype(np.uint8)
        self.assertIsNone(detect_target(color_masks(frame)["BLUE"]))

    def test_blue_reference_with_exposure_and_focus_variations(self):
        path = Path(__file__).parent/"target_validation"/"sample_blue_missed.png"
        if not path.exists():
            self.skipTest("Reference screenshot fixture is not present")
        original = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
        for gain in (0.75, 0.9, 1.1, 1.25):
            for sigma in (0, 0.6, 1.0):
                frame = np.clip(original.astype(np.float32)*gain, 0, 255).astype(np.uint8)
                if sigma:
                    frame = cv2.GaussianBlur(frame, (5, 5), sigma)
                with self.subTest(gain=gain, blur=sigma):
                    target = detect_target(color_masks(frame)["BLUE"])
                    self.assertIsNotNone(target)
                    np.testing.assert_allclose(target["center"], (649, 293), atol=5)


if __name__ == "__main__":
    unittest.main()
