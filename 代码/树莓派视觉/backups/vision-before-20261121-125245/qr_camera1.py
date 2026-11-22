"""USB camera QR reader plus UART sender for Raspberry Pi Thonny.

Camera 1 defaults to /dev/video0. Decoded text is printed in Thonny Shell.
The program only displays QR contents; links and commands are never executed.
"""
import time
from pathlib import Path
from datetime import datetime
import cv2
import numpy as np
import serial

DEVICE = "/dev/video0"
WIDTH, HEIGHT, FPS = 640, 480, 10
UART_DEVICE = "/dev/serial0"
UART_BAUD = 115200


def valid_target_code(value):
    """Accept groups such as 253+231+532+132; every group is three digits 1..6."""
    groups = value.split("+")
    return bool(groups) and all(
        len(group) == 3 and all(digit in "123456" for digit in group)
        for group in groups
    )


def decode_frame(detector, frame):
    results = []
    # The multi-code API also handles a single QR code.
    try:
        ok, texts, polygons, _ = detector.detectAndDecodeMulti(frame)
        if ok and polygons is not None:
            for value, polygon in zip(texts, polygons):
                if value:
                    results.append((value, np.asarray(polygon, np.float32)))
    except (cv2.error, UnicodeError):
        pass
    if not results:
        try:
            value, polygon, _ = detector.detectAndDecode(frame)
            if value and polygon is not None:
                results.append((value, np.asarray(polygon, np.float32).reshape(4, 2)))
        except (cv2.error, UnicodeError):
            pass
    return results


def label(image, text, position, color=(80, 255, 80)):
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX,
                0.55, color, 1, cv2.LINE_AA)


def main():
    if not Path(DEVICE).exists():
        print("没有检测到摄像头 1：", DEVICE)
        print("请插好 USB 摄像头，再运行本程序。")
        return
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    window = "QR camera 1 | Q: quit | SPACE: save"
    detector = cv2.QRCodeDetector()
    previous = None
    last_print = 0.0
    port = None
    try:
        if not cap.isOpened():
            raise RuntimeError("无法打开摄像头，请先停止其他摄像头程序。")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        try:
            port = serial.Serial(
                UART_DEVICE, UART_BAUD, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=0.1, write_timeout=1,
                xonxoff=False, rtscts=False, dsrdtr=False,
            )
        except (serial.SerialException, OSError) as error:
            raise RuntimeError(
                f"无法打开 {UART_DEVICE}：{error}。请在树莓派设置中开启 Serial Port，\n"
                "关闭 Serial Console，重启后再运行。"
            ) from error
        cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(window, 960, 720)
        print("二维码识别已启动：请让二维码完整进入画面，保持清晰并避免反光。")
        print(f"串口已打开：{UART_DEVICE}，{UART_BAUD} 8N1。")
        print("只发送格式如 253+231+532+132 的二维码；同一码稳定出现时只发一次。")
        print("识别内容显示在这里；点击画面后按 Q/Esc 退出，空格保存当前照片。")
        candidate = None
        candidate_frames = 0
        last_sent = None
        missing_since = None
        last_state_log = None
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("摄像头读取失败，请检查 USB 连接后重新运行。")
            results = decode_frame(detector, frame)
            display = frame.copy()
            for number, (value, polygon) in enumerate(results, 1):
                points = np.rint(polygon).astype(np.int32)
                cv2.polylines(display, [points], True, (0, 255, 0), 2, cv2.LINE_AA)
                center = tuple(np.rint(polygon.mean(axis=0)).astype(int))
                cv2.drawMarker(display, center, (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
                x = max(0, min(int(points[:, 0].min()), frame.shape[1]-100))
                y = max(50, int(points[:, 1].min())-10)
                label(display, f"QR {number}", (x, y))
                # OpenCV's built-in font cannot render Chinese. Full contents go to Shell.
                preview = value if value.isascii() else "Text decoded - see Thonny Shell"
                preview = preview.replace("\n", " ").replace("\r", " ")[:65]
                label(display, preview, (8, frame.shape[0]-15-(number-1)*22))
            label(display, f"QR codes decoded: {len(results)}", (8, 25))
            values = tuple(sorted({value for value, _ in results}))
            now = time.monotonic()
            valid_values = tuple(value for value in values if valid_target_code(value))

            # Require one identical valid code in three consecutive frames.
            # Multiple different codes are ambiguous and are deliberately not sent.
            if len(valid_values) == 1 and len(values) == 1:
                value = valid_values[0]
                missing_since = None
                if value == candidate:
                    candidate_frames += 1
                else:
                    candidate, candidate_frames = value, 1
                if candidate_frames >= 3 and value != last_sent:
                    packet = ("QR," + value + "\n").encode("ascii")
                    try:
                        port.write(packet)
                        port.flush()
                    except (serial.SerialException, OSError) as error:
                        raise RuntimeError(f"串口发送失败：{error}") from error
                    last_sent = value
                    print("串口已发送：", repr(packet))
                    last_state_log = ("sent", value)
            else:
                candidate, candidate_frames = None, 0
                if not values:
                    if missing_since is None:
                        missing_since = now
                    elif now-missing_since >= 1.0:
                        last_sent = None
                else:
                    missing_since = None
                state = "multiple or invalid QR" if values else "no QR"
                state_key = (state, values)
                if state_key != last_state_log:
                    if values:
                        print("识别到内容，但格式不符或有多个码，不发送：", values)
                    last_state_log = state_key
            if values and (values != previous or now-last_print >= 5):
                print("\n识别时间：", datetime.now().strftime("%H:%M:%S"))
                for number, (value, polygon) in enumerate(results, 1):
                    x, y = polygon.mean(axis=0)
                    print(f"二维码 {number}，中心 ({x:.1f}, {y:.1f})：")
                    print(value)
                last_print = now
            previous = values
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == 32:
                folder = Path.home()/"Pictures"/"qr_lab"
                folder.mkdir(parents=True, exist_ok=True)
                path = folder/(datetime.now().strftime("qr_%Y%m%d_%H%M%S_%f")+".jpg")
                if cv2.imwrite(str(path), display):
                    print("照片已保存：", path)
                else:
                    print("保存失败：", path)
    finally:
        if port is not None and port.is_open:
            port.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
