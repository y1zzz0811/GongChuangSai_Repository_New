"""Integration regression checks; no camera, display, or UART hardware is opened."""
import unittest

import cv2
import numpy as np

from dual_camera_recognition import QRTransmitter, circle_result, qr_result


class FakePort:
    def __init__(self):
        self.packets = []

    def write(self, packet):
        self.packets.append(packet)
        return len(packet)

    def flush(self):
        pass


class RecognitionTests(unittest.TestCase):
    def test_qr_decode_and_source_is_unchanged(self):
        payload = "253+231+532+132"
        qr = cv2.QRCodeEncoder_create().encode(payload)
        qr = cv2.copyMakeBorder(qr, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=255)
        qr = cv2.resize(qr, (300, 300), interpolation=cv2.INTER_NEAREST)
        frame = np.full((480, 640, 3), 255, np.uint8)
        frame[90:390, 170:470] = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
        original = frame.copy()
        display, results = qr_result(frame, cv2.QRCodeDetector())
        self.assertEqual([value for value, _ in results], [payload])
        self.assertEqual(display.shape, frame.shape)
        np.testing.assert_array_equal(frame, original)

    def test_circle_coordinates_center_cross_and_loss(self):
        frame = np.zeros((600, 800, 3), np.uint8)
        center = (470, 330)
        for radius in (30, 45, 60):
            cv2.circle(frame, center, radius, (255, 0, 0), 6, cv2.LINE_AA)
        original = frame.copy()
        smoothed = {}
        display, targets = circle_result(frame, smoothed)
        self.assertIn("BLUE", targets)
        np.testing.assert_allclose(targets["BLUE"]["center"], center, atol=2)
        np.testing.assert_allclose(targets["BLUE"]["offset"], (70.5, 30.5), atol=2)
        np.testing.assert_array_equal(display[300, 400], (0, 255, 0))
        np.testing.assert_array_equal(frame, original)
        blank = np.zeros_like(frame)
        display, targets = circle_result(blank, smoothed)
        self.assertEqual(targets, {})
        self.assertEqual(smoothed, {})
        np.testing.assert_array_equal(display[300, 400], (0, 255, 0))

    def test_uart_stability_deduplication_and_rearm(self):
        port = FakePort()
        sender = QRTransmitter(port)
        code = "253+231+532+132"
        for index in range(8):
            sender.update((code,), index*0.1)
        self.assertEqual(port.packets, [b"QR,253+231+532+132\n"])
        sender.update((), 1.0)
        sender.update((), 2.1)
        for now in (2.2, 2.3, 2.4):
            sender.update((code,), now)
        self.assertEqual(port.packets, [b"QR,253+231+532+132\n"]*2)

    def test_multiple_and_invalid_codes_reset_stability(self):
        port = FakePort()
        sender = QRTransmitter(port)
        for now in (0.0, 0.1):
            sender.update(("253",), now)
        sender.update(("253", "231"), 0.2)
        sender.update(("253",), 0.3)
        sender.update(("253",), 0.4)
        self.assertEqual(port.packets, [])
        for index in range(4):
            sender.update(("invalid",), 1.0+index*0.1)
        self.assertEqual(port.packets, [])

    def test_partial_uart_write_is_not_marked_sent(self):
        port = FakePort()
        port.write = lambda packet: len(packet)-1
        sender = QRTransmitter(port)
        sender.update(("253",), 0.0)
        sender.update(("253",), 0.1)
        with self.assertRaises(OSError):
            sender.update(("253",), 0.2)
        self.assertIsNone(sender.last_sent)


if __name__ == "__main__":
    unittest.main()
