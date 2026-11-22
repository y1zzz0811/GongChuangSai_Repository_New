"""Camera 2 numbered ground-ring preview, optional VQ/VR UART."""
import argparse
import time
from pathlib import Path
import cv2
import serial
from qr_camera1 import UART_DEVICE, UART_BAUD
from route2_vision import RingLink
from dual_camera_recognition import RecognitionCamera, CAMERA_MODES
from ring_metric import RingMetric


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, default=Path(__file__).resolve().with_name('ring_metric_Z184_image_center.json'))
    parser.add_argument('--device', default='/dev/video2')
    parser.add_argument('--uart', action='store_true', help='Enable ground-ring VQ/VR UART')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float, default=0)
    args = parser.parse_args()
    if args.headless and args.duration <= 0:
        parser.error('--headless requires --duration')
    cv2.setNumThreads(2)
    metric = RingMetric.load(args.calibration)
    camera = RecognitionCamera(1, args.device, CAMERA_MODES[1], False, ring_metric=metric)
    title = 'Camera 2 rings - SPACE save, Q quit'
    started = time.monotonic()
    last_sequence = None
    received = 0
    port = None
    link = None
    try:
        if args.uart:
            port = serial.Serial(UART_DEVICE, UART_BAUD, timeout=0, write_timeout=0.1,
                                 xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
            link = RingLink(port, camera, debug=True)
            print(f'UART OPEN {UART_DEVICE} {UART_BAUD} 8N1; ground-ring frame limit 1000ms', flush=True)
        if not args.headless:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(title, 960, 720)
        while args.duration <= 0 or time.monotonic()-started < args.duration:
            if link is not None:
                link.poll(time.monotonic())
            result, error = camera.snapshot()
            if error:
                raise RuntimeError(error)
            if result is not None:
                if result['sequence'] != last_sequence:
                    received += 1
                    last_sequence = result['sequence']
                    if not args.headless:
                        cv2.imshow(title, result['display'])
                if args.headless:
                    time.sleep(0.02)
                    continue
            elif time.monotonic()-started > 10:
                raise RuntimeError('No camera frames within 10 seconds')
            if args.headless:
                time.sleep(0.02)
                continue
            key = cv2.waitKey(20) & 0xff
            if key in (ord('q'), 27):
                break
            if key == ord(' ') and result is not None:
                path = Path(__file__).resolve().with_name(f'camera2_ring_raw_{time.time_ns()}.png')
                if not cv2.imwrite(str(path), result['raw']):
                    raise OSError('Unable to save camera frame')
                print(path, flush=True)
        if args.headless:
            if received == 0:
                raise RuntimeError('No processed camera frames received')
            print(f'Camera 2 OK: {received} processed frames', flush=True)
    finally:
        if port is not None:
            port.close()
        camera.stop()
        camera.join()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
