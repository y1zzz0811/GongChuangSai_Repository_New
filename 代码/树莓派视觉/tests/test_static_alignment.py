"""Offline contract checks; never opens cameras, serial ports or motors."""
from pathlib import Path
import json
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from ring_metric import RingMetric
from route2_top_center import EdgeRawTarget, TopAdapter
from route2_vision import RingLink, crc8
from top_center_uart import StaticRawGate, TopCenterLink
from dual_camera_recognition import RecognitionCamera


def record(color='YELLOW', offset=(70, -45), **changes):
    item = dict(color=color, source='TOP', height_uncertain=False,
                offset=np.array(offset, dtype=float), center=np.array([399.5, 299.5]),
                rim=np.array([[390., 299.5], [410., 299.5]]), score=10)
    item.update(changes)
    return item


class Port:
    def __init__(self, request=b'VQ,1,2,RAW\n'):
        self.rx, self.tx = bytearray(request), []

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def write(self, data):
        self.tx.append(data)
        return len(data)


class Camera:
    def __init__(self):
        self.capture = self
        self.frame = np.zeros((600, 800, 3), dtype=np.uint8)
        self.sequence, self.stamp, self.status, self.mode = 0, 100., '', None

    def snapshot(self):
        return self.frame, self.sequence, None

    def timed_snapshot(self):
        return self.frame, self.sequence, None, self.stamp

    def set_vision_request_mode(self, mode, requested_ring=None):
        self.mode = mode

    def set_vision_status(self, message):
        self.status = message


class StaticAlignment(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import ring_metric
        cls.metric = RingMetric.load(Path(ring_metric.__file__).with_name(
            'ring_metric_Z184_image_center.json'))

    def target(self):
        # Window and Lab data deliberately absent: neither gates static replies.
        target = EdgeRawTarget(self.metric, dict(static_manual=True, unlimited_stop=True))
        self.records = [record()]
        def detect(frame):
            target.vision.vision.axis_records = self.records
            return 'mock measured records', None
        target.vision.vision.detect = detect
        return target

    def test_same_six_color_selection_and_protocol_limits(self):
        gate, target = StaticRawGate(), self.target()
        for color, color_id in gate.colors.items():
            self.records = [record(color, (500, -500))]
            actual = target.observe(None, 100., color_id, None)
            selected, _ = gate.measure(self.records, None, 100., color_id, None)
            np.testing.assert_equal(actual, selected['offset'])
        for items in ([], [record('RED')], [record(), record()],
                      [record(offset=(501, 0))], [record(offset=(np.nan, 0))],
                      [record(offset=(1,))], [record(height_uncertain=True)],
                      [record(source='BASE')]):
            self.records = items
            self.assertIsNone(target.observe(None, 100., '2', None))

    def test_static_requires_explicit_stationary_mode(self):
        for settings in (dict(static_manual=True, unlimited_stop=False),
                         dict(static_manual='true', unlimited_stop=True)):
            with self.assertRaises(ValueError):
                EdgeRawTarget(self.metric, settings)

    def session(self, request=b'VQ,1,2,RAW\n', target=None):
        clock, camera, port = [100.], Camera(), Port(request)
        target = target or self.target()
        link = RingLink(port, camera, raw=target)
        return clock, camera, port, link

    def test_static_setup_exports_without_unused_lab_or_window(self):
        from raw_config_setup import make_config
        settings = make_config(None, {}, 4, '', '', '', True, True)
        self.assertEqual(settings, dict(static_manual=True, unlimited_stop=True))
        self.assertTrue(EdgeRawTarget(self.metric, settings).static_test)
        with self.assertRaises(ValueError):
            make_config(None, {}, 4, '', '', '', False, True)

    def test_top_returns_beyond_50mm_in_both_entries(self):
        adapter = TopAdapter(self.metric, search_mm=50)
        items = [record(offset=(120, -80))]
        def detect(frame):
            adapter.vision.axis_records = items
            return 'TOP EDGE', None
        adapter.vision.detect = detect
        _, result = adapter.detect(None)
        np.testing.assert_equal(result[2], (120, -80))
        clock, port = [100.], Port(b'VQ,5,2,TOP\n')
        standalone = TopCenterLink(port, search_mm=50, clock=lambda: clock[0], log_path=None)
        for _ in range(5):
            clock[0] += .1
            standalone.observe(items, clock[0])
        self.assertEqual(len(port.tx), 1)
        self.assertIn(b',120000,-80000,TOP,', port.tx[0])
        items.append(record(offset=(121, -80)))
        self.assertIsNone(adapter.detect(None)[1])

    def test_raw_selects_requested_color_among_different_colors(self):
        target = self.target()
        self.records = [record('RED'), record('YELLOW'), record('BLUE')]
        np.testing.assert_equal(target.observe(None, 100., '2', None), (70, -45))
        self.records.append(record('YELLOW', (80, -40)))
        self.assertIsNone(target.observe(None, 100., '2', None))

    def test_raw_and_ground_use_their_separate_forward_scales(self):
        from raw_pickup_axis_vision import AxisVision, FixedSurfaceTopVision, RAW_FWD_SCALE, GROUND_FWD_SCALE
        def detect(vision, frame):
            item = record(offset=(20, -10))
            vision.axis_records = [item]
            return 'TOP EDGE', (item['center'], item['rim'], item['offset'], item['score'])
        for surface, scale in [('tray', RAW_FWD_SCALE), ('ground', GROUND_FWD_SCALE)]:
            vision = FixedSurfaceTopVision(self.metric, 10, 80, 60, 30, 50, surface=surface)
            with patch.object(AxisVision, 'detect', detect):
                _, result = vision.detect(None)
            np.testing.assert_allclose(result[2], (20 * scale, -10))
            np.testing.assert_equal(vision.axis_records[0]['offset_geometric_mm'], (20, -10))
        self.assertAlmostEqual(RAW_FWD_SCALE, 1850/2450)

    def drive(self, session, count=6, ages=None):
        clock, camera, _, link = session
        for step in range(count):
            camera.sequence += 1
            clock[0] += .1
            camera.stamp = clock[0] - (ages or [0] * count)[step]
            with patch('route2_vision.time.monotonic', side_effect=lambda: clock[0]):
                link.poll(clock[0])

    def test_nonzero_errors_match_standalone_wire_and_retake(self):
        session = self.session()
        clock, camera, port, _ = session
        standalone_port = Port()
        standalone = TopCenterLink(standalone_port, raw=StaticRawGate(),
                                   clock=lambda: clock[0], log_path=None)
        # Run both paths on equivalent fresh observations; no one-second gate.
        for _ in range(5):
            camera.sequence += 1
            clock[0] += .1
            camera.stamp = clock[0]
            with patch('route2_vision.time.monotonic', side_effect=lambda: clock[0]):
                session[3].poll(clock[0])
            standalone.observe(self.records, camera.stamp, camera.frame)
        self.assertEqual(len(port.tx), 1)
        self.assertEqual(port.tx, standalone_port.tx)
        body, checksum = port.tx[0].rstrip().rsplit(b',', 1)
        self.assertEqual(checksum, f'{crc8(body):02X}'.encode())
        self.assertIn(b',70000,-45000,RAW', body)
        # A correction >50 mm and a signed fractional displacement are accepted
        # just as in the independent parser, followed by fresh measurements.
        port.rx.extend(b'VQ,2,2,RAW,+70.5,-45\n')
        self.records = [record(offset=(3, -2))]
        self.drive(session)
        self.assertEqual(len(port.tx), 2)
        self.assertTrue(port.tx[1].startswith(b'VR,2,2,'))
        self.assertIn(b',3000,-2000,RAW,', port.tx[1])

    def test_no_reply_for_stale_or_unstable_or_missing_target(self):
        session = self.session()
        self.drive(session, ages=[1] * 6)
        self.assertFalse(session[2].tx)
        self.assertIn('stale', session[1].status)
        session = self.session()
        for index in range(7):
            self.records = [record(offset=(70 + (index % 2) * 10, -45))]
            self.drive(session, 1)
        self.assertFalse(session[2].tx)
        self.records = []
        self.drive(session, 1)
        self.assertIn('0 requested-color', session[1].status)

    def test_latest_frame_limit_is_still_350ms(self):
        session = self.session()
        self.drive(session, ages=[.4] * 6)
        self.assertFalse(session[2].tx)

    def test_no_cached_replies_and_target_color_change(self):
        session = self.session()
        self.drive(session)
        port = session[2]
        port.rx.extend(b'VQ,9,3,RAW\n')
        self.drive(session)
        self.assertEqual(len(port.tx), 1)
        self.records = [record('BLUE', (12, 5))]
        self.drive(session)
        self.assertEqual(len(port.tx), 2)
        self.assertTrue(port.tx[1].startswith(b'VR,9,3,'))

    def test_top_and_numbered_ring_replies_remain_compatible(self):
        class Top:
            def detect(self, frame):
                item = record(offset=(40, 10))
                return 'TOP EDGE', (item['center'], item['rim'], item['offset'], 10)
        for request in (b'VQ,7,2,TOP\n', b'VQ,7,2\n'):
            session = self.session(request)
            clock, camera, port, link = session
            link.top = Top()
            if b'TOP' not in request:
                camera.capture = SimpleNamespace(snapshot=camera.snapshot,
                                                 timed_snapshot=camera.timed_snapshot)
                def ring_snapshot():
                    return dict(sequence=camera.sequence, captured_at=camera.stamp,
                                rings={'2': {'offset_mm': (40, 10)}}), None
                camera.snapshot = ring_snapshot
            self.drive(session)
            self.assertEqual(len(port.tx), 1)
            self.assertIn(b',40000,10000', port.tx[0])
            body, check = port.tx[0].rstrip().rsplit(b',', 1)
            self.assertEqual(check, f'{crc8(body):02X}'.encode())
            self.assertEqual(body.endswith(b',TOP'), b'TOP' in request)

    def test_invalid_requests_never_produce_reply(self):
        for request in (b'VQ,0,2,RAW\n', b'VQ,1,7,RAW\n', b'VQ,1,2,raw\n',
                        b'VQ,1,2,TOP,1,2\n', b'VQ,1,2,RAW,nan,0\n',
                        b'VQ,4294967296,2,RAW\n'):
            session = self.session(request)
            self.drive(session)
            self.assertFalse(session[2].tx)

    def test_timeouts_still_close_pending_requests(self):
        session = self.session()
        self.drive(session, 1)
        session[0][0] += 90
        self.drive(session, 1)
        self.assertIsNone(session[3].pending)
        self.assertFalse(session[2].tx)

    def test_ui_status_access_without_camera_thread(self):
        camera = RecognitionCamera.__new__(RecognitionCamera)
        camera.lock = threading.Lock()
        camera.set_vision_status('RAW WAIT: 0 requested-color matches')
        self.assertEqual(camera.vision_status(), 'RAW WAIT: 0 requested-color matches')


if __name__ == '__main__':
    unittest.main()
