"""双摄像头识别：摄像头 1 读二维码，摄像头 2 检测彩色同心圆。

在 Thonny 打开本文件并运行。请与原来的三个脚本放在同一个目录。
空格保存两路原图和识别结果；Q/Esc 退出。--no-uart 可关闭二维码串口发送。
"""
import argparse
from datetime import datetime
from pathlib import Path
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import cv2
import numpy as np
import serial

from dual_camera_preview import Camera, command, fit_size, usb_devices
from qr_camera1 import (
    UART_BAUD, UART_DEVICE, decode_frame, label, valid_target_code,
)
from target_center_camera1 import COLORS, color_masks, cross, detect_target, text


# 顺序与原来双摄预览一致：第一台 /dev/video0，第二台 /dev/video2。
# 若物理接线相反，交换两个路径。也可以填写 /dev/v4l/by-path/ 下的稳定路径。
CAMERA_DEVICES = ("/dev/video0", "/dev/video2")
# 保留各识别脚本已经使用的分辨率和 YUYV 色彩格式；完整画面只缩放、不裁剪。
CAMERA_MODES = (("YUYV", 640, 480, 10), ("YUYV", 800, 600, 10))
ENABLE_UART = True


class QRTransmitter:
    """保留原 QR 协议和三帧稳定、防重复发送逻辑。"""

    def __init__(self, port):
        self.port = port
        self.candidate = None
        self.candidate_frames = 0
        self.last_sent = None
        self.missing_since = None

    def update(self, values, now):
        values = tuple(sorted(set(values)))
        if len(values) == 1 and valid_target_code(values[0]):
            value = values[0]
            self.missing_since = None
            if value == self.candidate:
                self.candidate_frames += 1
            else:
                self.candidate, self.candidate_frames = value, 1
            if self.candidate_frames >= 3 and value != self.last_sent:
                packet = ("QR," + value + "\n").encode("ascii")
                if self.port.write(packet) != len(packet):
                    raise OSError("串口没有完整发送二维码数据")
                self.port.flush()
                self.last_sent = value
                print("串口已发送：", repr(packet), flush=True)
        else:
            self.candidate, self.candidate_frames = None, 0
            if not values:
                if self.missing_since is None:
                    self.missing_since = now
                elif now - self.missing_since >= 1.0:
                    self.last_sent = None
            else:
                self.missing_since = None


def qr_result(frame, detector):
    results = decode_frame(detector, frame)
    display = frame.copy()
    for number, (value, polygon) in enumerate(results, 1):
        points = np.rint(polygon).astype(np.int32)
        cv2.polylines(display, [points], True, (0, 255, 0), 2, cv2.LINE_AA)
        center = tuple(np.rint(polygon.mean(axis=0)).astype(int))
        cv2.drawMarker(display, center, (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
        x = max(0, min(int(points[:, 0].min()), frame.shape[1] - 100))
        y = max(50, int(points[:, 1].min()) - 10)
        label(display, f"QR {number}", (x, y))
        preview = value if value.isascii() else "Text decoded - see result below"
        preview = preview.replace("\n", " ").replace("\r", " ")[:65]
        label(display, preview, (8, max(40, frame.shape[0] - 15 - (number-1)*22)))
    label(display, f"Camera 1 | QR codes: {len(results)}", (8, 25))
    return display, results


def circle_result(frame, smoothed):
    height, width = frame.shape[:2]
    origin = np.array([(width-1)/2, (height-1)/2])
    display = frame.copy()
    found = {}
    for row, (name, mask) in enumerate(color_masks(frame).items()):
        target = detect_target(mask)
        color = COLORS[name]
        if target is None:
            smoothed.pop(name, None)
            text(display, name + ": NOT FOUND", (8, 22+row*23), color)
            continue
        measured = target["center"]
        previous = smoothed.get(name)
        if previous is not None and np.linalg.norm(previous-measured) < target["diameter"]*0.2:
            center = 0.4*measured + 0.6*previous
        else:
            center = measured
        smoothed[name] = center
        for ring in target["rings"]:
            cv2.ellipse(display, ring["ellipse"], color, 1, cv2.LINE_AA)
        cross(display, center, color)
        delta = center - origin
        found[name] = {"center": center.copy(), "offset": delta.copy()}
        text(display, f"{name}: ({center[0]:.1f},{center[1]:.1f}) dx={delta[0]:+.1f} dy={delta[1]:+.1f}",
             (8, 22+row*23), color, 0.44)
        px, py = np.rint(center).astype(int)
        text(display, f"{name} rings={len(target['rings'])}",
             (max(0, min(px+12, width-160)), max(90, min(py-12, height-30))), color, 0.42)
    # 最后绘制中心十字，即使没有识别到圆圈、圆心与中心重合，也保留中心标识。
    cross(display, origin, (0, 255, 0), 18)
    text(display, "IMAGE CENTER", (int(origin[0])+22, int(origin[1])-15), scale=0.4)
    text(display, "Camera 2 | dx: right + | dy: down +", (8, height-12), scale=0.42)
    return display, found


class RecognitionCamera:
    """每路独立采集和识别，只保留最新画面，Tk 主线程负责显示。"""

    def __init__(self, index, device, mode, enable_uart):
        self.index, self.device = index, device
        self.enable_uart = enable_uart and index == 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.capture = Camera(device, mode)
        self.result = None
        self.error = None
        self.processed = 0
        self.uart_status = "串口已关闭"
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        port = None
        try:
            if self.enable_uart:
                try:
                    port = serial.Serial(UART_DEVICE, UART_BAUD, timeout=0.1, write_timeout=1,
                                         xonxoff=False, rtscts=False, dsrdtr=False)
                    self.uart_status = f"串口 {UART_DEVICE}，{UART_BAUD} 8N1"
                except (serial.SerialException, OSError) as error:
                    self.uart_status = f"串口不可用，仅识别：{error}"
                    print(self.uart_status, flush=True)
            transmitter = QRTransmitter(port) if port is not None else None
            detector = cv2.QRCodeDetector() if self.index == 0 else None
            smoothed = {}
            previous_values, last_print = (), 0.0
            previous_sequence = -1
            last_processed = time.monotonic()
            while not self.stop_event.is_set():
                started = time.monotonic()
                frame, sequence, capture_status = self.capture.snapshot()
                if frame is None or sequence == previous_sequence:
                    if capture_status.startswith("错误："):
                        raise RuntimeError(capture_status)
                    if started - last_processed > 5:
                        raise RuntimeError("超过 5 秒没有新画面，请检查摄像头连接")
                    self.stop_event.wait(0.01)
                    continue
                previous_sequence = sequence
                if self.index == 0:
                    display, codes = qr_result(frame, detector)
                    values = tuple(sorted({value for value, _ in codes}))
                    if values and (values != previous_values or started-last_print >= 5):
                        print("二维码识别：", values, flush=True)
                        last_print = started
                    previous_values = values
                    if transmitter is not None:
                        try:
                            transmitter.update(values, started)
                        except (serial.SerialException, OSError) as error:
                            self.uart_status = f"串口发送失败，仅识别：{error}"
                            print(self.uart_status, flush=True)
                            transmitter = None
                            port.close()
                            port = None
                    info = "；".join(values)[:300] if values else "尚未识别到二维码"
                    details = f"{self.uart_status}；{info}"
                else:
                    display, circles = circle_result(frame, smoothed)
                    h, w = frame.shape[:2]
                    details = f"图像中心 ({(w-1)/2:g}, {(h-1)/2:g})；"
                    details += "、".join(circles) + " 圆圈已识别" if circles else "尚未识别到彩色同心圆"
                elapsed = max(time.monotonic() - started, 0.001)
                self.processed += 1
                with self.lock:
                    self.result = dict(raw=frame, display=display, sequence=sequence,
                                       status=f"{capture_status}；识别用时 {elapsed*1000:.0f} ms",
                                       details=details, timestamp=time.monotonic())
                last_processed = time.monotonic()
                self.stop_event.wait(max(0, 0.1 - elapsed))
        except Exception as error:
            with self.lock:
                self.error = str(error)
                self.result = None
            print(f"摄像头 {self.index+1} 错误：{error}", flush=True)
        finally:
            self.capture.stop_event.set()
            if port is not None:
                port.close()

    def snapshot(self):
        with self.lock:
            return self.result, self.error

    def stop(self):
        self.stop_event.set()
        self.capture.stop_event.set()

    def join(self):
        self.thread.join(timeout=2)
        self.capture.thread.join(timeout=1)


class App:
    def __init__(self, root, cameras, duration=0):
        self.root, self.cameras = root, cameras
        self.closed = False
        self.after_id = None
        self.duration_id = None
        self.photos = [None, None]
        self.render_keys = [None, None]
        self.labels, self.statuses, self.details = [], [], []
        root.title("双摄像头识别：1 二维码 / 2 圆圈与中心十字")
        root.geometry("1280x720")
        root.minsize(800, 480)
        root.protocol("WM_DELETE_WINDOW", self.close)
        for key in ("<Escape>", "q", "Q"):
            root.bind(key, lambda event: self.close())
        root.bind("<space>", lambda event: self.save())
        header = ttk.Frame(root, padding=8)
        header.pack(fill="x")
        ttk.Label(header, text="完整画面，不裁剪；摄像头 2 绿色十字为图像中心；dx 向右为正，dy 向下为正。").pack(side="left")
        ttk.Button(header, text="退出", command=self.close).pack(side="right")
        ttk.Button(header, text="拍照（空格）", command=self.save).pack(side="right", padx=5)
        body = ttk.Frame(root)
        body.pack(fill="both", expand=True)
        body.rowconfigure(0, weight=1)
        for index, camera in enumerate(cameras):
            body.columnconfigure(index, weight=1, uniform="camera")
            panel = ttk.Frame(body, padding=5)
            panel.grid(row=0, column=index, sticky="nsew")
            role = "二维码识别" if index == 0 else "彩色圆圈识别 + 中心十字"
            ttk.Label(panel, text=f"摄像头 {index+1} — {role} — {camera.device}", anchor="center").pack(fill="x")
            status = tk.StringVar(value="正在打开摄像头…")
            ttk.Label(panel, textvariable=status, anchor="center", wraplength=580).pack(fill="x")
            image = tk.Label(panel, background="#151515", foreground="white", text="等待画面…")
            image.pack(fill="both", expand=True)
            details = tk.StringVar(value="")
            ttk.Label(panel, textvariable=details, wraplength=580, anchor="w").pack(fill="x")
            self.labels.append(image)
            self.statuses.append(status)
            self.details.append(details)
        self.notice = tk.StringVar(value="空格保存两路原图和识别结果；Q / Esc 退出。圆心坐标以采集原图为准，未做光学标定。")
        ttk.Label(root, textvariable=self.notice, padding=6).pack(fill="x")
        if duration > 0:
            self.duration_id = root.after(round(duration*1000), self.close)
        self.refresh()

    def refresh(self):
        if self.closed:
            return
        for index, camera in enumerate(self.cameras):
            result, error = camera.snapshot()
            if result is None or time.monotonic()-result["timestamp"] > 5:
                message = f"错误：{error}" if error else "等待新画面…"
                self.statuses[index].set(message)
                self.details[index].set("")
                self.photos[index] = None
                self.render_keys[index] = None
                self.labels[index].configure(image="", text=message)
                continue
            self.statuses[index].set(result["status"])
            self.details[index].set(result["details"])
            label_widget = self.labels[index]
            box_w, box_h = label_widget.winfo_width(), label_widget.winfo_height()
            if min(box_w, box_h) < 10:
                continue
            render_key = (result["sequence"], box_w, box_h)
            if self.render_keys[index] == render_key:
                continue
            frame = result["display"]
            h, w = frame.shape[:2]
            new_w, new_h = fit_size(w, h, box_w, box_h)
            display = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            ppm = f"P6\n{new_w} {new_h}\n255\n".encode() + rgb.tobytes()
            self.photos[index] = tk.PhotoImage(data=ppm, format="PPM")
            label_widget.configure(image=self.photos[index], text="")
            self.render_keys[index] = render_key
        self.after_id = self.root.after(70, self.refresh)

    def save(self):
        folder = Path.home()/"Pictures"/"dual_recognition"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        saved = 0
        try:
            folder.mkdir(parents=True, exist_ok=True)
            for index, camera in enumerate(self.cameras, 1):
                result, _ = camera.snapshot()
                if result is None or time.monotonic()-result["timestamp"] > 5:
                    continue
                for suffix, key in (("raw", "raw"), ("result", "display")):
                    path = folder/f"{stamp}_camera{index}_{suffix}.jpg"
                    if not cv2.imwrite(str(path), result[key]):
                        raise OSError(f"保存失败：{path}")
                    saved += 1
            self.notice.set(f"已保存 {saved} 张照片：{folder}" if saved else "还没有可保存的画面。")
            print(self.notice.get(), flush=True)
        except OSError as error:
            messagebox.showerror("保存照片", str(error), parent=self.root)

    def close(self):
        if self.closed:
            return
        self.closed = True
        for callback in (self.after_id, self.duration_id):
            if callback is not None:
                self.root.after_cancel(callback)
        for camera in self.cameras:
            camera.stop()
        self.root.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-uart", action="store_true", help="只识别，不打开串口、不发送数据")
    parser.add_argument("--duration", type=float, default=0, help="运行指定秒数后退出；默认持续运行")
    parser.add_argument("--headless", action="store_true", help="无窗口检查双路采集识别，必须配合 --duration")
    args = parser.parse_args(argv)
    if args.headless and args.duration <= 0:
        parser.error("--headless 需要 --duration 指定正数秒数")
    cv2.setNumThreads(2)
    cameras = []
    root = None
    app = None
    try:
        devices = CAMERA_DEVICES or usb_devices(command("v4l2-ctl", "--list-devices"))[:2]
        if len(devices) != 2 or len({Path(d).resolve() for d in devices}) != 2:
            raise RuntimeError("需要两个不同的 USB 摄像头，请检查 CAMERA_DEVICES")
        for index, (device, mode) in enumerate(zip(devices, CAMERA_MODES)):
            if not Path(device).exists():
                raise RuntimeError(f"没有检测到摄像头 {index+1}：{device}")
            cameras.append(RecognitionCamera(index, device, mode, ENABLE_UART and not args.no_uart))
            print(f"摄像头 {index+1}：{device}，请求 {mode[1]}x{mode[2]} {mode[0]} {mode[3]} FPS", flush=True)
        if args.headless:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                if any(camera.snapshot()[1] for camera in cameras):
                    break
                time.sleep(0.1)
        else:
            root = tk.Tk()
            app = App(root, cameras, args.duration)
            root.mainloop()
        for camera in cameras:
            result, error = camera.snapshot()
            print(f"摄像头 {camera.index+1} 已处理 {camera.processed} 帧；错误：{error}", flush=True)
            if error or camera.processed == 0:
                raise RuntimeError(error or f"摄像头 {camera.index+1} 未获取到有效画面")
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f"运行失败：{error}", flush=True)
        if root is not None and (app is None or not app.closed):
            messagebox.showerror("双摄像头识别", str(error), parent=root)
        return 1
    finally:
        if app is not None:
            app.close()
        elif root is not None:
            root.destroy()
        for camera in cameras:
            camera.stop()
        for camera in cameras:
            camera.join()


if __name__ == "__main__":
    raise SystemExit(main())
